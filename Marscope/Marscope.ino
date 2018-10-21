//--------------------------------------------------------------------------------------------
// Marscope - 2018 NASA Space Apps Challenge
// 
// Project Page: https://2018.spaceappschallenge.org/challenges/can-you-build/make-sense-out-mars/teams/parker-space/project
//
// Note: This project uses the following hardware
//    - Teensy 3.6 (Teensy Modules needed)
//    - BME680
//    - CCS811
//
// The following libraries are required:
// - https://github.com/nox771/i2c_t3 (I2C Library for Teensy 3.6)
//
// Recommend Arduino v1.8.7 when installing the Teensy modules.
//--------------------------------------------------------------------------------------------

#include <LiquidCrystal.h>

#include "BlueDot_BME680_t3.h"
#include "CCS811_t3.h"

//--------------------------------------------------------------------------------------------
// Objects for data acquisition.
Adafruit_CCS811 ccsObject;
BlueDot_BME680 bmeObject = BlueDot_BME680();

typedef struct {
  // Data taken from CCS811 sensor:
  float co2_ppm;                  // Internal CO2 levels.
  float tvoc_ppb;                 // Internal TVOC levels.
  float in_temperature_degreesC;  // Internal Temperature.

  // Data taken from BME680 sensor:
  float ex_temperature_degreesC;  // External Temperature.
  float ex_humidity_percentage;   // External Humidity.
  float ex_pressure_kPA;          // External hPA.
  float altitudeCalc_m;           // Altitude from datum in meters.

  //MICS5525 
} SensorData;
SensorData data;

// LCD -related objects.
const int rs = 11, 
          en = 12, 
          d4 = 24, 
          d5 = 25, 
          d6 = 26, 
          d7 = 27;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//--------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  if (!LCD_setup())
  {
    Serial.println("Failed to initialize the LCD.");
    while(1);
  }
  
  if (!CCS_setup())
  {
    Serial.println("Failed to initialize the CCS811.");
    while(1);
  }

  if (!BME_setup())
  {
    Serial.println("Failed to initialize the BME680.");
    while(1);
  }
}

//--------------------------------------------------------------------------------------------
void loop() {
  char buffer[100];
  lcd.clear();
  if (CCS_measure())
  {
    lcd.setCursor(0, 0);
    snprintf(buffer, 100, "CO2: %.2f PPM", data.co2_ppm);
    lcd.print(buffer);
    
    lcd.setCursor(0, 1);
    snprintf(buffer, 100, "TVOC: %.2f PPB", data.tvoc_ppb);
    lcd.print(buffer);
    
    delay(2500);
    lcd.clear();
  }
  if (BME_measure())
  {
    lcd.setCursor(0, 0);
    snprintf(buffer, 100, "Temp: %.2f C", data.ex_temperature_degreesC);
    lcd.printf(buffer);

    lcd.setCursor(0, 1);
    snprintf(buffer, 100, "Hum.: %.2f%%", data.ex_humidity_percentage);
    lcd.printf(buffer);

    delay(2500);
    lcd.clear();

    lcd.setCursor(0, 0);
    snprintf(buffer, 100, "Pres.: %.2f kPA", data.ex_pressure_kPA);
    lcd.printf(buffer);

    lcd.setCursor(0, 1);
    snprintf(buffer, 100, "Alt..: %.2f m", data.altitudeCalc_m);
    lcd.printf(buffer);

    delay(2500);
    lcd.clear();
  }
}

//--------------------------------------------------------------------------------------------
bool LCD_setup()
{
  bool rc = true;
  // Set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  //lcd.autoscroll();

  return rc;
}

//--------------------------------------------------------------------------------------------
bool CCS_setup()
{
  bool rc = false;
  
  //calibrate temperature sensor
  if (ccsObject.begin())
  {
    while(!ccsObject.available());
    float temp = ccsObject.calculateTemperature();
    ccsObject.setTempOffset(temp - 25.0);

    rc = true;
  }

  return rc;
}

//--------------------------------------------------------------------------------------------
bool CCS_measure()
{
  bool rc = false;
  if (ccsObject.available())
  {
    data.in_temperature_degreesC = ccsObject.calculateTemperature();
    if (ccsObject.readData() == 0)
    {
      data.co2_ppm = ccsObject.geteCO2();
      data.tvoc_ppb = ccsObject.getTVOC();
      
      rc = true;
    }
  }

  return rc;
}

//--------------------------------------------------------------------------------------------
bool BME_setup() 
{
  bool rc = false;
  //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);    
  bmeObject.parameter.I2CAddress = 0x77;                                   //Choose I2C Addres
  bmeObject.parameter.sensorMode = 0b01;                                   //Default sensor mo
  bmeObject.parameter.IIRfilter = 0b100;                                   //Setting IIR Filter coefficient to 15 (default)
  bmeObject.parameter.humidOversampling = 0b101;                           //Setting Humidity Oversampling to factor 16 (default) 
  bmeObject.parameter.tempOversampling = 0b101;                            //Setting Temperature Oversampling factor to 16 (default)
  bmeObject.parameter.pressOversampling = 0b101;                           //Setting Pressure Oversampling to factor 16 (default) 
  bmeObject.parameter.pressureSeaLevel = 1013.25;                          //default value of 1013.25 hPa
  bmeObject.parameter.tempOutsideCelsius = 15;                             //default value of 15°C
  bmeObject.parameter.target_temp = 320;
  
  rc = (bmeObject.init() == 0x61);

  return rc;
}

//Calculates the altitude based on pressure -www.grc.nasa.gov/www/k-12/rocket/atmosmrm.html 
float AltitudeCalc()
{
  return log((bmeObject.readPressure() * 0.1)/0.699)/(-0.00009);
}
  
bool BME_measure()
{
  bool rc = true;
  bmeObject.writeCTRLMeas();
  
  data.ex_temperature_degreesC = bmeObject.readTempC();
  data.ex_humidity_percentage = bmeObject.readHumidity();
  data.ex_pressure_kPA = bmeObject.readPressure() * 0.1;
  data.altitudeCalc_m = AltitudeCalc(); 

  Serial.print("Measured temperature ");
  Serial.println(data.ex_temperature_degreesC);
  Serial.print("Measured humidity: ");
  Serial.println(data.ex_humidity_percentage);
  Serial.print("Measured pressure: ");
  Serial.println(data.ex_pressure_kPA);
  Serial.print("Calculated altitude: ");
  Serial.println(data.altitudeCalc_m);

  return rc;
} 
