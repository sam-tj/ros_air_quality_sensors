#ifndef MQUnifiedsensor_H
#define MQUnifiedsensor_H

#include <math.h>
#include "freertos/task.h"
#include <stdint.h>

/***********************Software Related Macros************************************/

#define ADC_RESOLUTION 12 // for 10bit analog to digital converter.
#define retries 2
#define retry_interval 20

int Ro_inf = 9999;
int _pin;
bool _firstFlag = false;
int _ADC_Bit_Resolution = 12;
int _regressionMethod = 1; // 1 -> Exponential || 2 -> Linear
float _adc, _a, _b, _sensor_volt;
float RS_air, _ratio, _PPM, _RS_Calc;
// char _type[6];
// char _placa[20];
bool _isMQ303A = false;
float _correctionFactor = 0.0;
bool _injected = false;
float _ratioInput = 0;
float _VOLT_RESOLUTION = 3.09; // 5.0; // if 3.3v use 3.3
float _R0 = 10;
float _RL = 10;                // Value in KiloOhms
bool _read = true;
int _value = 0;

// MQUnifiedsensor(String Placa = "Arduino", float Voltage_Resolution = 5, int ADC_Bit_Resolution = 10, int pin = 1, String type = "CUSTOM MQ");
// MQUnifiedsensor(String Placa = "Arduino", String type = "CUSTOM MQ");
void MQUnifiedsensor(float Voltage_Resolution, int ADC_Bit_Resolution, int pin);

// Functions to set values
// void init();
void update();
void externalADCUpdate(float volt);
void setR0(float R0);
void setRL(float RL);
//void setA(float a);
//void setB(float b);
// void setRegressionMethod(int regressionMethod);
void setVoltResolution(float VOLT_RESOLUTION);
// void serialDebug(bool onSetup = false); // Show on serial port information about sensor
void setADC(int value); // For external ADC Usage

// user functions
float calibrate(float ratioInCleanAir);
float readSensor(bool isMQ303A, float correctionFactor, bool injected);
float readSensorR0Rs();
// float validateEcuation(float ratioInput);

// get function for info
// float getA();
// float getB();
float getR0();
float getRL();
float getVoltResolution();
// String getRegressionMethod();
float getVoltage(bool read, bool injected, int value);
// float stringTofloat(String &str);

// functions for testing
float getRS();
float setRsR0RatioGetPPM(float value);

float readCO2();
float readCO();
float readALCOHOL();
float readAMMONIUM();
float readTOULENE();
float readACETONE();

#endif // MQUnifiedsensor_H