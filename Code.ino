// ====== Blynk Template and Auth ======
#define BLYNK_TEMPLATE_ID "TMPL6fAoJTrst"
#define BLYNK_TEMPLATE_NAME "Irrigation system "
#define BLYNK_AUTH_TOKEN "F1iq8OJtpfT5DVQkhyqDYXatWuIv5Vt9"


#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>


// ====== WiFi Credentials ======
const char WIFI_SSID[] = "Readmi Node 9";
const char WIFI_PASS[] = "muyde2001";

// ====== Pins ======
#define DHTPIN D4          // DHT11 Data pin
#define DHTTYPE DHT11

#define RELAY1_PIN D6      // Relay controlling Motor1 (soil moisture)
#define RELAY2_PIN D7      // Relay controlling Motor2 (pH sensor)

// ADS1115 Channels
#define ADS_RAIN 0
#define ADS_SOIL 1
#define ADS_PH   2

// Blynk virtual pins as requested
#define VP_TEMP   V3
#define VP_HUM    V4
#define VP_SOIL   V1
#define VP_PH     V2
#define VP_RAIN   V5

#define VPIN_MOTOR1 V6
#define VPIN_MOTOR2 V7

// Thresholds
const float SOIL_LOW_THRESHOLD = 30.0;  // Soil moisture < 30%
const float SOIL_HIGH_THRESHOLD = 80.0; // Soil moisture > 80%
const float PH_LOW_THRESHOLD = 5.5;     // pH < 5.5
const float PH_HIGH_THRESHOLD = 8.5;    // pH > 8.5

// Rain detection ADC threshold (adjust based on your sensor)
const int RAIN_THRESHOLD = 10000;

Adafruit_ADS1115 ads;
DHT dht(DHTPIN, DHTTYPE);

// Flags to avoid repeated notifications
bool lowMoistureNotified = false;
bool highMoistureNotified = false;
bool lowPhNotified = false;
bool highPhNotified = false;
bool rainNotified = false;

// Store relay states for Blynk control
bool motor1State = false;
bool motor2State = false;

// ----------- Blynk handlers for motor controls ------------
BLYNK_WRITE(VPIN_MOTOR1) {
  motor1State = param.asInt();
  digitalWrite(RELAY1_PIN, motor1State ? LOW : HIGH); // Active LOW relay
}

BLYNK_WRITE(VPIN_MOTOR2) {
  motor2State = param.asInt();
  digitalWrite(RELAY2_PIN, motor2State ? LOW : HIGH); // Active LOW relay
}

// ----------- Helper functions -----------

float rawToVoltage(int16_t raw) {
  // ADS1115 gain ±4.096V => LSB = 0.125mV
  return raw * 0.000125;
}

float mapSoilVoltageToPercent(float voltage) {
  // Adjust these calibration values for your soil sensor
  const float soilDryVoltage = 3.0;
  const float soilWetVoltage = 1.0;

  if (soilDryVoltage == soilWetVoltage) return 0;
  float percentage = (soilDryVoltage - voltage) / (soilDryVoltage - soilWetVoltage) * 100.0;
  if (percentage < 0) percentage = 0;
  if (percentage > 100) percentage = 100;
  return percentage;
}


float voltageToPH(float voltage) {
  // Adjust these calibration voltages for your pH sensor
  const float ph4Voltage = 2.5;
  const float ph7Voltage = 2.0;

  if (fabs(ph7Voltage - ph4Voltage) < 1e-6) return 7.0;
  float slope = (7.0 - 4.0) / (ph7Voltage - ph4Voltage);
  float intercept = 4.0 - slope * ph4Voltage;
  float ph = slope * voltage + intercept;
  if (ph < 0) ph = 0;
  if (ph > 14) ph = 14;
  return ph+3;
}

// ----------- Setup -----------

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting...");

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, HIGH); // Relays OFF (active LOW)
  digitalWrite(RELAY2_PIN, HIGH);

  Wire.begin(D2, D1); // SDA, SCL for NodeMCU
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS1115");
    while (1) delay(1000);
  }
  ads.setGain(GAIN_ONE); // ±4.096V

  dht.begin();

  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS);
}

// ----------- Main loop -----------

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 5000; // ms

void loop() {
  Blynk.run();

  unsigned long now = millis();
  if (now - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = now;
    readSensorsAndUpdate();
  }
}

// ----------- Sensor read & Blynk update -----------

void readSensorsAndUpdate() {
  // Read ADS1115 channels
  int16_t rainRaw = ads.readADC_SingleEnded(ADS_RAIN);
  int16_t soilRaw = ads.readADC_SingleEnded(ADS_SOIL);
  int16_t phRaw   = ads.readADC_SingleEnded(ADS_PH);

  float rainVoltage = rawToVoltage(rainRaw);
  float soilVoltage = rawToVoltage(soilRaw);
  float phVoltage   = rawToVoltage(phRaw);

  float soilPercent = mapSoilVoltageToPercent(soilVoltage);
  float phValue = voltageToPH(phVoltage);

  bool rainDetected = rainRaw < RAIN_THRESHOLD;

  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Serial debug
  Serial.printf("Temp: %.1f C, Hum: %.1f %%, Soil: %.1f %%, pH: %.2f, Rain: %s\n",
                temperature, humidity, soilPercent, phValue, rainDetected ? "YES" : "NO");

  // Send sensor data to Blynk
  if (!isnan(temperature)) Blynk.virtualWrite(VP_TEMP, temperature);
  if (!isnan(humidity)) Blynk.virtualWrite(VP_HUM, humidity);
  Blynk.virtualWrite(VP_SOIL, soilPercent);
  Blynk.virtualWrite(VP_PH, phValue);
  Blynk.virtualWrite(VP_RAIN, rainDetected ? 1 : 0);

  // Notifications & flags
  if (soilPercent < SOIL_LOW_THRESHOLD && !lowMoistureNotified) {
    Blynk.logEvent("low_moisture", "Soil moisture is low (<30%)");
    lowMoistureNotified = true;
    highMoistureNotified = false;
  }
  else if (soilPercent > SOIL_HIGH_THRESHOLD && !highMoistureNotified) {
    Blynk.logEvent("high_moisture", "Soil moisture is high (>80%)");
    highMoistureNotified = true;
    lowMoistureNotified = false;
  }
  else if (soilPercent >= SOIL_LOW_THRESHOLD && soilPercent <= SOIL_HIGH_THRESHOLD) {
    lowMoistureNotified = false;
    highMoistureNotified = false;
  }

  if (phValue < PH_LOW_THRESHOLD && !lowPhNotified) {
    Blynk.logEvent("low_ph", "pH is low (<5.5)");
    lowPhNotified = true;
    highPhNotified = false;
  }
  else if (phValue > PH_HIGH_THRESHOLD && !highPhNotified) {
    Blynk.logEvent("high_ph", "pH is high (>8.5)");
    highPhNotified = true;
    lowPhNotified = false;
  }
  else if (phValue >= PH_LOW_THRESHOLD && phValue <= PH_HIGH_THRESHOLD) {
    lowPhNotified = false;
    highPhNotified = false;
  }

  if (rainDetected && !rainNotified) {
    Blynk.logEvent("rain_detected", "Rain detected! You may turn off motors.");
    rainNotified = true;
  }
  else if (!rainDetected) {
    rainNotified = false;
  }
}
