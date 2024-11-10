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
  // DHT 22 air temp, humidity
  String airTempInCAverage;
  String airHumidityAverage;

  // SHT31 soil temp, humidity
  String soilTempInCAverage;
  String soilHumidityAverage;
};

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
      data.airHumidityAverage = NAN; 
      // return; 
    }
    else
    {
      temperatureSum += temperature;
      humiditySum += humidity;
    }
  }
  /*Calculate and assign averages*/
  data.airTempInCAverage = String(temperatureSum / 3.0f, 2);
  data.airHumidityAverage = String(humiditySum / 3.0f, 2);

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
  data.soilTempInCAverage = String(soilTemperatureSum / 3.0f, 2);
  data.soilHumidityAverage = String(soilHumiditySum / 3.0f, 2);

  return data;
};

void setup()
{
  Serial.begin(115200);
  
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
}

void loop()
{
  
  // Actual sensor values
  SensorData SensorData = deviceData();

  Serial.println();
  Serial.printf("Temperature: %s degrees \n", SensorData.airTempInCAverage);
  Serial.printf("Humidity: %s % \n", SensorData.airHumidityAverage);

  Serial.printf("soil temperature: %s degrees \n", SensorData.soilTempInCAverage);
  Serial.printf("soil humidity: %s % \n", SensorData.soilHumidityAverage);
  Serial.print("****************\n");

  delay(10000);
}

