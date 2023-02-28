/*********
  BJTM Technologies
  23-02-2023  
*********/

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "EmonLib.h"
#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <driver/adc.h>


BlynkTimer timer;

#define EEPROM_SIZE 1

EnergyMonitor emon;
#define CUR_ADC_INPUT 34
#define VOL_ADC_INPUT 35
// #define emonTxV3 1
// #define ESP32
// Force EmonLib to use 10bit ADC resolution
#define ADC_BITS 12
#define ADC_COUNTS (1 << ADC_BITS)

// #define vCalibration 66.2
#define vCalibration 260
// #define currCalibration 0.80
#define currCalibration 0.075
// #define currCalibration 10 
// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;

float kWh = 0;
unsigned long lastmillis = millis();

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

void myTimerEvent() {
  emon.calcVI(20, 2000);
  kWh = kWh + emon.apparentPower * (millis() - lastmillis) / 3600000000.0;
  yield();
  Serial.print("Vrms: ");
  Serial.print(emon.Vrms, 2);
  Serial.print("V");
  EEPROM.write(0, emon.Vrms);
  delay(100);

  Serial.print("\tIrms: ");
  Serial.print(emon.Irms, 4);
  Serial.print("A");
  EEPROM.write(1, emon.Irms);

  Serial.print("\tIrms Cal: ");
  Serial.print(emon.calcIrms(1480), 4);
  Serial.print("A");

  Serial.print("\tAnalogRead: ");
  Serial.print(analogRead(34));
  Serial.print(" Digits");

  Serial.print("\tAnalogRead: ");
  Serial.print(((analogRead(34) * 5.0) / 4096));
  Serial.print(" Digits");
  delay(100);

  Serial.print("\tPower: ");
  Serial.print(emon.apparentPower, 4);
  Serial.print("W");
  EEPROM.write(2, emon.apparentPower);
  delay(100);

  Serial.print("\tkWh: ");
  Serial.print(kWh, 5);
  Serial.println("kWh");
  EEPROM.write(3, kWh);

  lcd.clear();

  lcd.setCursor(0, 0);
  // lcd.print("Vrms:");
  if (emon.Vrms > 50) {
    lcd.print(emon.Vrms, 2);
    lcd.print("V");
  } else {

    lcd.print("0.00");
    lcd.print("V");
  }
  lcd.setCursor(8, 0);
  // lcd.print("Irms:");
  lcd.print(emon.Irms, 3);
  lcd.print("A");
  lcd.setCursor(0, 1);
  // lcd.print("Power:");
  lcd.print(emon.apparentPower, 2);
  lcd.print("W");
  lcd.setCursor(8, 1);
  // lcd.print("kWh:");
  lcd.print(kWh, 3);
  lcd.print("Kwh");

  lastmillis = millis();
}

void setup() {
   //Setup the ADC
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
  // analogSetPinAttenuation(CUR_ADC_INPUT, ADC_11db);
  // analogSetPinAttenuation(VOL_ADC_INPUT, ADC_11db);
  analogReadResolution(ADC_BITS);
  pinMode(CUR_ADC_INPUT, INPUT);
  pinMode(VOL_ADC_INPUT, INPUT);

  Serial.begin(115200);
  // initialize LCD
  lcd.init();
  // turn on LCD backlight
  lcd.backlight();

  // eepromState();
  EEPROM.begin(EEPROM_SIZE);
  emon.voltage(35, vCalibration, 0.8);  // Voltage: input pin, calibration, phase_shift
  emon.current(34, currCalibration);    // Current: input pin, calibration.

  timer.setInterval(4000L, myTimerEvent);

  lcd.setCursor(3, 0);
  lcd.print("IoT Energy");
  lcd.setCursor(5, 1);
  lcd.print("Meter");
  delay(3000);
  lcd.clear();
}

void loop() {
  timer.run();
  // myTimerEvent();
  // delay(5000);
}