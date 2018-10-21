//--------------------------------------------------------------------------------------------
// Marscope - 2018 NASA Space Apps Challenge
// 
// Project Page: https://2018.spaceappschallenge.org/challenges/can-you-build/make-sense-out-mars/teams/parker-space/project
//
// Note: This project uses the following hardware
//    - Teensy 3.6 (Teensy Modules needed)
//    - BME680
//    - CCS811
//    - MCIS5524
// The following libraries are required:
// - https://github.com/nox771/i2c_t3 (I2C Library for Teensy 3.6)
// 
// Recommend Arduino v1.8.7 when installing the Teensy modules.
// Authors: Hemant Gopalsing & Tamkin Rahman.
//--------------------------------------------------------------------------------------------

#include <LiquidCrystal.h>

#include "BlueDot_BME680_t3.h"
#include "CCS811_t3.h"

//--------------------------------------------------------------------------------------------
#define LCD_DELAY_ms 2500
#define LED_BLINK_DELAY_ms 1000

// Taken from: https://www.engineeringtoolbox.com/co2-comfort-level-d_1024.html
#define CO2_DANGER_ZONE_PPM 600 

#define METHANE_DETECTION_ZONE_PPB 3

#define RGB_GND_PIN   20
#define RGB_RED_PIN   23
#define RGB_GREEN_PIN 22
#define RGB_BLUE_PIN  21


#define V_ADC 2.56
#define VCC_Sensor 3.3
#define Analog_Pulldown 10
#define ADC_Resolution 13
#define SensorAnalog 34

//--------------------------------------------------------------------------------------------
typedef enum 
{
  DISPLAY_CO2_TVOC,
  DISPLAY_TEMP_HUMID,
  DISPLAY_PRESS_ALT,
  DISPLAY_METHANE,
  MAX_DISPLAY_MODES
} LcdDisplayModes;

typedef enum
{
  COND_CO2_HIGH,
  COND_CCS_ERROR,
  COND_BME_ERROR,
  COND_METHANE_HIGH,
  COND_NO_ERROR,
  MAX_CONDITIONS = COND_NO_ERROR 
} SystemConditions;


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
  float methane_ppb;              // External Methane levels.
} SensorData;
SensorData data;

//--------------------------------------------------------------------------------------------
bool conditions[MAX_CONDITIONS] = { false };

// Objects for data acquisition.
Adafruit_CCS811 ccsObject;
BlueDot_BME680 bmeObject = BlueDot_BME680();

// LCD -related objects.
const int rs = 11, 
          en = 12, 
          d4 = 24, 
          d5 = 25, 
          d6 = 26, 
          d7 = 27;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

float reference_PPM;

//--------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  reference_PPM = MethaneCalc();

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

  RGB_init();
}

//--------------------------------------------------------------------------------------------
void loop() 
{
  LCD_loop();
  RGB_loop();
}

//--------------------------------------------------------------------------------------------
SystemConditions current_condition()
{
  SystemConditions result = COND_NO_ERROR;
  for (int ix = 0; ix < MAX_CONDITIONS; ix++)
  {
    if (conditions[ix])
    {
      result = (SystemConditions)ix;

      Serial.print("Current condition is: ");
      Serial.println(ix);
      break; // Break out of this for loop.
    }
  }

  return result;
}

//--------------------------------------------------------------------------------------------
void RGB_init()
{
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);
  pinMode(RGB_GND_PIN, OUTPUT);
  digitalWrite(RGB_GND_PIN, LOW);
}

//--------------------------------------------------------------------------------------------
void RGB_loop()
{
  static unsigned long last = 0;
  static bool on = true;
  
  unsigned long now = millis();
  SystemConditions condition = current_condition();

  int R = 0;
  int B = 0;
  int G = 0;
  if ((now - last) > LED_BLINK_DELAY_ms)
  {
    last = now;
    if (on)
    {
      switch(condition)
      {
        case(COND_NO_ERROR):
        {
          // Green
          R = 0;
          G = 255;
          B = 0;

          Serial.println("No error encountered.");
          break;
        }
        case(COND_CO2_HIGH):
        {
          // Red
          R = 255;
          G = 0;
          B = 0;
          break;
        }
        case(COND_CCS_ERROR):
        {
          // Yellow
          R = 255;
          G = 255;
          B = 0;
          break;
        }
        case(COND_BME_ERROR):
        {
          // Turqoise
          R = 0;
          G = 255;
          B = 255;
          break;
        }
        case(COND_METHANE_HIGH):
        {
          // White
          R = 255;
          G = 255;
          B = 255;
          break;
        }
      }
      analogWrite(RGB_RED_PIN,   R);
      analogWrite(RGB_GREEN_PIN, G);
      analogWrite(RGB_BLUE_PIN,  B);

      on = false;
    }
    else
    {
      analogWrite(RGB_RED_PIN,   0);
      analogWrite(RGB_GREEN_PIN, 0);
      analogWrite(RGB_BLUE_PIN,  0);

      on = true;
    }
  }
}

//--------------------------------------------------------------------------------------------
void LCD_loop()
{
  static char buffer[100];
  static LcdDisplayModes lcd_state = DISPLAY_CO2_TVOC;
  static unsigned long last = 0;
  unsigned long now = millis();

  if ((now - last) > LCD_DELAY_ms)
  {
    lcd.clear();
    
    last = now;
    switch(lcd_state)
    {
      case(DISPLAY_CO2_TVOC):
      {
        conditions[COND_CCS_ERROR] = !CCS_measure();
        if(!conditions[COND_CCS_ERROR])
        {
          lcd.setCursor(0, 0);
          snprintf(buffer, 100, "CO2: %.2f PPM", data.co2_ppm);
          lcd.print(buffer);
          
          lcd.setCursor(0, 1);
          snprintf(buffer, 100, "TVOC: %.2f PPB", data.tvoc_ppb);
          lcd.print(buffer);

          lcd_state = DISPLAY_TEMP_HUMID;
        }
        break;
      }
      case(DISPLAY_TEMP_HUMID):
      {
        conditions[COND_BME_ERROR] = !BME_measure();
        if(!conditions[COND_BME_ERROR])
        {
          lcd.setCursor(0, 0);
          snprintf(buffer, 100, "Temp: %.2f C", data.ex_temperature_degreesC);
          lcd.printf(buffer);
      
          lcd.setCursor(0, 1);
          snprintf(buffer, 100, "Hum.: %.2f%%", data.ex_humidity_percentage);
          lcd.printf(buffer);

          lcd_state = DISPLAY_PRESS_ALT;
        }
        break;
      }
      case(DISPLAY_PRESS_ALT):
      {
        conditions[COND_BME_ERROR] = !BME_measure();
        if(!conditions[COND_BME_ERROR])
        {
          lcd.setCursor(0, 0);
          snprintf(buffer, 100, "Pres.: %.2f kPA", data.ex_pressure_kPA);
          lcd.printf(buffer);
      
          lcd.setCursor(0, 1);
          snprintf(buffer, 100, "Alt.: %.0f m", data.altitudeCalc_m);
          lcd.printf(buffer);

          lcd_state = DISPLAY_METHANE;
        }
        break;
      }
      case(DISPLAY_METHANE):
      {
        lcd.setCursor(0, 0);
        data.methane_ppb = MethaneCalc();
        conditions[COND_METHANE_HIGH] = ((data.methane_ppb) >= METHANE_DETECTION_ZONE_PPB);
        snprintf(buffer, 100, "CH4: %.2f PPB", data.methane_ppb);
        lcd.printf(buffer);

        lcd_state = DISPLAY_CO2_TVOC;
        break;
      }
      default:
      {
        lcd.setCursor(0, 0);
        lcd.printf("ERROR: Unexpected state.");
        break;
      }
    }
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
      conditions[COND_CO2_HIGH] = (data.co2_ppm >= CO2_DANGER_ZONE_PPM);
      
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

//--------------------------------------------------------------------------------------------
//Calculates the altitude based on pressure -www.grc.nasa.gov/www/k-12/rocket/atmosmrm.html 
float AltitudeCalc()
{
  return log((bmeObject.readPressure() * 0.1)/0.699)/(-0.00009);
}

//--------------------------------------------------------------------------------------------
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

//--------------------------------------------------------------------------------------------
float MethaneCalc()
{
  /*The MCIS sensor is sensitive to : i) Methane
   The adafruit sensor has a 10 kOhm pulldown at the analog output. The gas resistance RS and the 10Kohm forms a
   voltage dividor and we can calculate RS.
   The ratio RS/RO can be calculated from those values.
   The graph of RS/RO was plotted using the graph from the datasheet and after the graph was linearized using LN the following
   relationship was obtained: i) CH4 ------> Y = -0.00004 X + 0.83 , where Y = ln(PPB) and X = ln(RS/RO)
  */
  analogReadResolution(ADC_Resolution);
  int Raw_ADCval = analogRead(SensorAnalog);
  
  Serial.println(Raw_ADCval);
  
  float Corrected_Voltage = (Raw_ADCval*(V_ADC/pow(2.0,ADC_Resolution)));
  float RSRO_ratio = log(((VCC_Sensor/Corrected_Voltage)-1.0)*Analog_Pulldown);
  float PPB = exp(-0.00004*(RSRO_ratio) +0.83);

  return PPB;
}
