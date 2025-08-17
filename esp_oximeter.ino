#include <DFRobot_MultiGasSensor.h>
#include <MQUnifiedsensor.h>
#include "DHT.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// Configuration for WiFi
const char *ssid = "monumen tugu";
const char *password = "maulanaa123";
String serverURL = "http://203.100.57.59:3000/api/v1/air-monitor/add-air";

// ---------- SENSOR O2 ----------
#define I2C_ADDRESS 0x74
DFRobot_GAS_I2C gas(&Wire, I2C_ADDRESS);

// ---------- SENSOR CO & CO2 ----------
#define placa "ESP32S3"
#define Voltage_Resolution 3.3
#define pin 1 // Pin ADC (GPIO 36)
#define type "MQ-135"
#define ADC_Bit_Resolution 12 // ESP32S3 uses 12-bit ADC
#define RatioMQ135CleanAir 3.6
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

// ---------- SENSOR HUMIDITY & TEMPERATURE ----------
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ---------- RELAY ----------
#define RELAY_O2 5           // Relay for O₂ supply
#define RELAY_AIR_PURIFIER 6 // Relay for air purifier

// ---------- NORMAL SENSOR LIMITS ----------
#define O2_MIN 19.0  // Minimum normal O2
#define O2_MAX 21.0  // Maximum normal O2
#define CO_MAX 9     // Maximum normal CO (ppm)
#define CO2_MAX 1000 // Maximum normal CO2 (ppm)

// ---------- SENSOR PM2.5 & PM10 ----------
int measurePin = 2; // Using GPIO 36 as the ADC pin
int ledPower = 12;

unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;

// Declare variables globally
float CO = 0;
float CO2 = 0;
float O2 = 0;
float pm_25 = 0;
float pm_10 = 0;
float humidity = 0;
float temperature = 0;
bool isAirPurifierTriggeredManualOn = false;
bool isAirPurifierTriggeredManualOff = false;
bool isCo2TriggeredManualOn = false;
bool isCo2TriggeredManualOff = false;
WiFiClient client;

unsigned long previousMillis = 0;
const long interval = 60000; // 1 minute in milliseconds

// Add global variables to store trigger statuses
bool isO2Triggered = false;
bool isAirPurifierTriggered = false;

// Function to calculate AQI based on US EPA standard.
float calc_aqi_us(float concentration, String pollutant)
{
    float c_low[8], c_high[8], i_low[8], i_high[8];
    
    if (pollutant == "PM2.5")
    {
        c_low[0] = 0, c_low[1] = 12.1, c_low[2] = 35.5, c_low[3] = 55.5, c_low[4] = 150.5, c_low[5] = 250.5, c_low[6] = 350.5, c_low[7] = 500.5;
        c_high[0] = 12, c_high[1] = 35.4, c_high[2] = 55.4, c_high[3] = 150.4, c_high[4] = 250.4, c_high[5] = 350.4, c_high[6] = 500.4, c_high[7] = 1000.0;
        i_low[0] = 0, i_low[1] = 51, i_low[2] = 101, i_low[3] = 151, i_low[4] = 201, i_low[5] = 301, i_low[6] = 401, i_low[7] = 501;
        i_high[0] = 50, i_high[1] = 100, i_high[2] = 150, i_high[3] = 200, i_high[4] = 300, i_high[5] = 400, i_high[6] = 500, i_high[7] = 9999;
    }
    else if (pollutant == "PM10")
    {
        c_low[0] = 0, c_low[1] = 55, c_low[2] = 155, c_low[3] = 255, c_low[4] = 355, c_low[5] = 425, c_low[6] = 505, c_low[7] = 605;
        c_high[0] = 54, c_high[1] = 154, c_high[2] = 254, c_high[3] = 354, c_high[4] = 424, c_high[5] = 504, c_high[6] = 604, c_high[7] = 999;
        i_low[0] = 0, i_low[1] = 51, i_low[2] = 101, i_low[3] = 151, i_low[4] = 201, i_low[5] = 301, i_low[6] = 401, i_low[7] = 501;
        i_high[0] = 50, i_high[1] = 100, i_high[2] = 150, i_high[3] = 200, i_high[4] = 300, i_high[5] = 400, i_high[6] = 500, i_high[7] = 9999;
    }
    else if (pollutant == "CO")
    {
        c_low[0] = 0, c_low[1] = 4.4, c_low[2] = 9.4, c_low[3] = 12.4, c_low[4] = 15.4, c_low[5] = 30.4, c_low[6] = 35.4, c_low[7] = 40.4;
        c_high[0] = 4.3, c_high[1] = 9.3, c_high[2] = 12.3, c_high[3] = 15.3, c_high[4] = 30.3, c_high[5] = 35.3, c_high[6] = 40.3, c_high[7] = 100;
        i_low[0] = 0, i_low[1] = 51, i_low[2] = 101, i_low[3] = 151, i_low[4] = 201, i_low[5] = 301, i_low[6] = 401, i_low[7] = 501;
        i_high[0] = 50, i_high[1] = 100, i_high[2] = 150, i_high[3] = 200, i_high[4] = 300, i_high[5] = 400, i_high[6] = 500, i_high[7] = 9999;
    }
    else
    {
        return -1; // Invalid pollutant type
    }

    float c = concentration;
    int i = 0;
    while (c > c_high[i])
    {
        i++;
    }

    float aqi = ((i_high[i] - i_low[i]) / (c_high[i] - c_low[i])) * (c - c_low[i]) + i_low[i];
    return aqi;
}

void handleSerialCommand()
{
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n'); // Read input line
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, input);

    if (error)
    {
      Serial.println("Invalid JSON");
      return;
    }

    Serial.println("Received command: " + input);

    const char *action = doc["action"];
    const char *relayCode = doc["relay_code"];

    const int target = atoi(relayCode);

    if (strcmp(action, "turn_on_relay") == 0)
    {
      if (target == RELAY_O2)
      {
        isCo2TriggeredManualOn = true;
        isCo2TriggeredManualOff = false;
      }
      else
      {
        isAirPurifierTriggeredManualOn = true;
        isAirPurifierTriggeredManualOff = false;
      }
      Serial.println("Relay turned ON");
    }
    else if (strcmp(action, "turn_off_relay") == 0)
    {
      if (target == RELAY_O2)
      {
        isCo2TriggeredManualOff = true;
        isCo2TriggeredManualOn = false;
      }
      else
      {
        isAirPurifierTriggeredManualOff = true;
        isAirPurifierTriggeredManualOn = false;
      }
      Serial.println("Relay turned OFF");
    }
    else
    {
      Serial.println("Unknown action");
    }
  }
}

void setup()
{
  Serial.begin(115200);

  // WiFi Initialization
  WiFi.begin(ssid, password);

  // Initialize I2C
  Wire.begin(9, 8); // SDA = GPIO 9, SCL = GPIO 8

  // Initialize Relay
  pinMode(RELAY_O2, OUTPUT);
  pinMode(RELAY_AIR_PURIFIER, OUTPUT);
  digitalWrite(RELAY_O2, HIGH); // Turn off the relay initially
  digitalWrite(RELAY_AIR_PURIFIER, HIGH);

  // Initialize O2 Sensor
  while (!gas.begin())
  {
    delay(1000);
  }
  gas.changeAcquireMode(gas.PASSIVITY);
  gas.setTempCompensation(gas.OFF);

  // Initialize MQ-135 Sensor
  MQ135.setRegressionMethod(1);
  MQ135.init();
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++)
  {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    delay(100);
  }
  MQ135.setR0(calcR0 / 10);

  // Initialize DHT Sensor
  dht.begin();

  // Initialize PM2.5 & PM10 Sensor
  pinMode(ledPower, OUTPUT);
}

void loop()
{
  handleSerialCommand();
  unsigned long currentMillis = millis();

  // Read sensors
  O2 = gas.readGasConcentrationPPM();

  MQ135.update();
  MQ135.setA(605.18);
  MQ135.setB(-3.937);
  CO = MQ135.readSensor();

  MQ135.setA(110.47);
  MQ135.setB(-2.862);
  CO2 = MQ135.readSensor() + 400;

  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  digitalWrite(ledPower, LOW);
  delayMicroseconds(samplingTime);
  int voMeasured = analogRead(measurePin);
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower, HIGH);
  delayMicroseconds(sleepTime);

  float calcVoltage = voMeasured * (3.3 / 1024.0);
  float dustDensity = 0.17 * calcVoltage - 0.1;
  if (dustDensity < 0)
    dustDensity = 0.00;
  pm_25 = dustDensity * 11;
  pm_10 = dustDensity * 15;

  // *Trigger untuk Suplai O2 (Relay hanya ON selama 1 detik)*
  if ((isCo2TriggeredManualOn) || ((O2 < O2_MIN) && !isO2Triggered))
  {
    digitalWrite(RELAY_O2, LOW); // Aktifkan Relay (Trigger O2 Concentrator)
    Serial.println("ALERT! Kadar O2 di bawah normal. RELAY O2 ON");
    delay(1000);                  // Biarkan relay menyala selama 3 detik
    digitalWrite(RELAY_O2, HIGH); // Kembalikan ke OFF setelah 3 detik
    Serial.println("RELAY O2 OFF");
    isCo2TriggeredManualOn = false;
    isO2Triggered = true; // Tandai bahwa trigger sudah dilakukan
  }
  else if ((isCo2TriggeredManualOff) || ((O2 >= O2_MAX) && isO2Triggered))
  {
    digitalWrite(RELAY_O2, LOW); // Aktifkan Relay (Trigger untuk mematikan O2 Concentrator)
    Serial.println("Kadar O2 normal. RELAY O2 OFF");
    delay(1000);                  // Biarkan relay menyala selama 3 detik
    digitalWrite(RELAY_O2, HIGH); // Kembalikan ke OFF setelah 3 detik
    Serial.println("RELAY O2 OFF");
    isO2Triggered = false;
    isCo2TriggeredManualOff = false;
    Serial.println("Kadar O2 normal. RELAY OFF"); // Reset status
  }
  
  Serial.println("isCo2TriggeredManualOn: " + String(isCo2TriggeredManualOn));
  Serial.println("isCo2TriggeredManualOff: " + String(isCo2TriggeredManualOff));
  Serial.println("isAP2TriggeredManualOn: " + String(isAirPurifierTriggeredManualOn));
  Serial.println("isAP2TriggeredManualOff: " + String(isAirPurifierTriggeredManualOff));
  Serial.println("CO22: " + String(CO2));

  if ((!isAirPurifierTriggeredManualOff && isAirPurifierTriggeredManualOn) || (CO2 > CO2_MAX))
  {
    digitalWrite(RELAY_AIR_PURIFIER, LOW);
    // delay(1000);
    // digitalWrite(RELAY_AIR_PURIFIER, LOW);
    isAirPurifierTriggeredManualOn = false;

    Serial.println("ALERT! Kadar CO atau CO2 melebihi batas normal. RELAY ON");
  }
  else if (!isAirPurifierTriggeredManualOn && isAirPurifierTriggeredManualOff)
  {
    digitalWrite(RELAY_AIR_PURIFIER, HIGH);
    isAirPurifierTriggeredManualOff = false;
    Serial.println("Kadar CO dan CO2 normal. RELAY OFF");
  }

  // Kalkulasi AQI untuk PM2.5, PM10, dan CO
  float aqi_pm25 = calc_aqi_us(pm_25, "PM2.5");
  float aqi_pm10 = calc_aqi_us(pm_10, "PM10");
  float aqi_co = calc_aqi_us(CO, "CO");

  // Pilih nilai AQI maksimum
  float max_aqi = max(aqi_pm25, max(aqi_pm10, aqi_co));

  // Check which range the max AQI falls into
  String aqi_category;
  if (max_aqi >= 0 && max_aqi <= 50) {
    aqi_category = "Baik (Good)";
  } else if (max_aqi >= 51 && max_aqi <= 100) {
    aqi_category = "Sedang (Moderate)";
  } else if (max_aqi >= 101 && max_aqi <= 199) {
    aqi_category = "Tidak Sehat (Unhealthy)";
  } else {
    aqi_category = "Sangat Tidak Sehat (Very Unhealthy)";
  }

  // Send data to server every 1 minute
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    sendDataToServer();
  }

  const int cor = digitalRead(RELAY_O2);
  const int apr = digitalRead(RELAY_AIR_PURIFIER);

  // Debug prints
  Serial.print("CO: ");
  Serial.println(CO);
  Serial.print("CO2: ");
  Serial.println(CO2);
  Serial.print("PM25: ");
  Serial.println(pm_25);
  Serial.print("PM10: ");
  Serial.println(pm_10);
  Serial.print("TEMP: ");
  Serial.println(temperature);
  Serial.print("O2: ");
  Serial.println(O2);
  Serial.print("HUM: ");
  Serial.println(humidity);
  Serial.print("APR: ");
  Serial.println(apr);
  Serial.print("COR: ");
  Serial.println(cor);
  Serial.print("Air Quality Index: ");
  Serial.println(max_aqi, 0);
  Serial.print("Kategori: ");
  Serial.println(aqi_category);

  delay(1000);
}

void sendDataToServer()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;
    String httpRequestData = "{\"vehicle_id\":\"4\", \"co\":\"" + String(CO) +
                             "\", \"co2\":\"" + String(CO2) +
                             "\", \"o2\":\"" + String(O2) +
                             "\", \"pm_25\":\"" + String(pm_25) +
                             "\", \"humidity\":\"" + String(humidity) +
                             "\", \"temperature\":\"" + String(temperature) + "\"}";

    http.begin(client, serverURL);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(httpRequestData);
    if (httpResponseCode > 0)
    {
      String response = http.getString();
      Serial.println("HTTP Response Code: " + String(httpResponseCode));
      Serial.println("Response: " + response);
    }
    else
    {
      Serial.println("Error sending data: " + String(httpResponseCode));
    }
    http.end();
  }
  else
  {
    Serial.println("WiFi not connected");
  }
}