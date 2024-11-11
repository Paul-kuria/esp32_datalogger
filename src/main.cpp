#include <SPI.h>
#include "time.h"

#include <LoRa.h>
#include <TinyGPS++.h>
#include "DHTesp.h"
#include <SensirionI2cSht3x.h>

#include "helpers.h"
#include "scan_i2c.h"

// Instantiate Classes
TinyGPSPlus gps;
HardwareSerial GPS(1);
SensirionI2cSht3x sensor;
DHTesp dht;

static char errorMessage[64];
static int16_t error; 

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (GPS.available())
      gps.encode(GPS.read());
  } while (millis() - start < ms);
}

struct SensorData
{
  char timestamp[32];  
  uint8_t mac[6];  // Store MAC address as a 6-byte array
  bool isValid; 

  // DHT 22 air temp, humidity
  float airTempInCAverage;
  float airHumidityAverage;

  // SHT31 soil temp, humidity
  float soilTempInCAverage;
  float soilHumidityAverage;
  
  // GPS module
  double latitude;
  double longitude;
  float battery_level;
  float signal;
};



void start_sleep(){
  esp_sleep_enable_timer_wakeup(UpdateInterval);
  pinMode(BUILTIN_LED,INPUT);     // Set pin-5 to an input as sometimes PIN-5 is used for SPI-SS
  digitalWrite(BUILTIN_LED,HIGH); // In case it's on, turn LED off, as sometimes PIN-5 on some boards is used for SPI-SS and can be left low
  Serial.println("Starting deep-sleep period... awake for "+String(millis())+"mS");
  delay(8); // Enough time for the serial port to finish at 115,200 baud
  esp_deep_sleep_start();         // Sleep for the prescribed time
}

SensorData deviceData()
{
    SensorData data;

  // DHT22 - Air parameters
  float temperatureSum = 0.0f;
  float humiditySum = 0.0f;
  
  /* Read sensor data 3 times, averaging values */ 
  for (int i=0; i<3; ++i)
  {
    delay(1000); // wait 1 sec before next reading
    float temperature = dht.getTemperature();
    float humidity = dht.getHumidity();
    if (isnan(humidity) || isnan(temperature))
    {
      Serial.println(F("Failed to read from DHT sensor!"));
      data.airTempInCAverage = NAN;
      data.airHumidityAverage = NAN; \
      break;
      // return; 
    }
    else
    {
      temperatureSum += temperature;
      humiditySum += humidity;
    }
  }
  /*Calculate and assign averages*/
  data.airTempInCAverage = temperatureSum / 3.0f;
  data.airHumidityAverage = humiditySum / 3.0f;

// SHT31 - Soil parameters
  float soilTemperatureSum = 0.0f;
  float soilHumiditySum = 0.0f;
  float sTemperature = 0.0;
  float sHumidity = 0.0;
  
  /* Read sensor data 3 times, averaging values */ 
  for (int i=0; i<3; ++i)
  {
    delay(1000); // wait 1 sec before next reading
    error = sensor.blockingReadMeasurement(sTemperature, sHumidity);
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute blockingReadMeasurement(): ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        data.soilTempInCAverage = NAN;
        data.soilHumidityAverage = NAN; 
      // return; 
    }
    else
    {
      soilTemperatureSum += sTemperature;
      soilHumiditySum += sHumidity;

    }
  }
  /*Calculate and assign averages*/
  data.soilTempInCAverage = soilTemperatureSum / 3.0f;
  data.soilHumidityAverage = soilHumiditySum / 3.0f;

  // MAC
  esp_read_mac(data.mac, ESP_MAC_WIFI_STA);  // Read MAC address

  // GPS
  data.isValid = gps.location.isValid();
  data.latitude = gps.location.lat();
  data.longitude = gps.location.lng();
  // data.isValid = gps.location.isValid();
  // data.numSatellites = gps.satellites.value();
  // data.altitudeFeet = gps.altitude.feet();

  return data;
};

void Send_and_Display_Sensor_Data(){
  SensorData sensorData = deviceData();
  char macAddressString[18];  // Buffer to hold MAC address in string format
  sprintf(macAddressString, "%02X:%02X:%02X:%02X:%02X:%02X", 
        sensorData.mac[0], sensorData.mac[1], sensorData.mac[2], 
        sensorData.mac[3], sensorData.mac[4], sensorData.mac[5]);

  LoRa.beginPacket();     
  
  LoRa.print("MAC:");
  LoRa.println(macAddressString);  // Print the formatted MAC address

  LoRa.print("Latitude");
  LoRa.println(sensorData.latitude);
 
  LoRa.print("Longitude");
  LoRa.println(sensorData.longitude);

  LoRa.print("Temp: ");
  LoRa.println(sensorData.airTempInCAverage,1);

  LoRa.print("Humi: ");
  LoRa.println(sensorData.airHumidityAverage,1);

  LoRa.print("sTemp: ");
  LoRa.println(sensorData.soilTempInCAverage,1);

  LoRa.print("sHumi: ");
  LoRa.println(sensorData.soilHumidityAverage,1);

  LoRa.endPacket();                                                  // Confirm end of LoRa data packet
  LoRa.sleep();     

  Serial.printf("MAC address: %s \n", macAddressString);

  Serial.printf("Temperature: %.2f °C \n", sensorData.airTempInCAverage);
  Serial.printf("Humidity: %.2f % \n", sensorData.airHumidityAverage);
  Serial.printf("soil temperature: %.2f °C \n", sensorData.soilTempInCAverage);
  Serial.printf("soil humidity: %.2f % \n", sensorData.soilHumidityAverage);

  Serial.printf("valid: %s \n", sensorData.isValid ? "true" : "false");
  Serial.printf("Latitude: %.2f  \n", sensorData.latitude);
  Serial.printf("Longitude: %.2f  \n", sensorData.longitude);

  Serial.print("****************\n");
}

void setup()
{
  Serial.begin(115200);
  
  // Instantiate LORA
  LoRa.setPins(LORA_CS, LORA_RESETT, LORA_IRQ); // LoRa.setPins(CS, RESET, IRQ); 
  if (!LoRa.begin(868E6)) { // Set frequency to 433, 868 or 915MHz
    Serial.println("Could not find a valid LoRa transceiver, check pins used and wiring!");
    delay(1000);
  }

  // Instantiate GPS
  GPS.begin(9600, SERIAL_8N1, 34, 12);   //17-TX 18-RX

  // Instantiate DHT sensor (Start I2C communication)
  dht.setup(DHTPIN, DHTesp::DHT22);

  // Instantiate Soil sensor
  // /*------------------------------------*/
  Wire1.begin(I2C_SDA_2, I2C_SCL_2);
  sensor.begin(Wire1, SHT31_I2C_ADDR_44);
  sensor.stopMeasurement();
  delay(1);
  sensor.softReset();
  delay(100);
  uint16_t aStatusRegister = 0u;
  error = sensor.readStatusRegister(aStatusRegister);
  if (error != NO_ERROR) {
      Serial.print("Error trying to execute readStatusRegister(): ");
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
      return;
  }
  Serial.print("aStatusRegister: ");
  Serial.print(aStatusRegister);
  Serial.println();
  error = sensor.startPeriodicMeasurement(REPEATABILITY_MEDIUM,
                                          MPS_ONE_PER_SECOND);
  if (error != NO_ERROR) {
      Serial.print("Error trying to execute startPeriodicMeasurement(): ");
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
      return;
  }

  Serial.println("Sending data packet...");
  // start_sleep();
}

void loop()
{
  
  // Actual sensor values
  Send_and_Display_Sensor_Data();

  delay(10000);
}

