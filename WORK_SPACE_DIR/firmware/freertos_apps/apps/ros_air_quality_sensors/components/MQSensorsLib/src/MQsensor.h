#ifndef MQsensor_H_H
#define MQsensor_H_H

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
bool _isMQ303A = false;
float _correctionFactor = 0.0;
bool _injected = false;
float _ratioInput = 0;
float _VOLT_RESOLUTION = 3.09; // 5.0; // if 3.3v use 3.3
float _R0 = 10;
float _RL = 10;                // Value in KiloOhms
bool _read = true;
int _value = 0;

void MQsensor(float Voltage_Resolution, int ADC_Bit_Resolution, int pin);
void updateMQ();
void externalADCUpdate(float volt);
void setR0(float R0);
void setRL(float RL);
void setVoltResolution(float VOLT_RESOLUTION);
void setADC(int value); // For external ADC Usage
float calibrate(float ratioInCleanAir);
float readSensor(bool isMQ303A, float correctionFactor, bool injected);
float readSensorR0Rs();
float getR0();
float getRL();
float getVoltResolution();
float getVoltage(bool read, bool injected, int value);
float getRS();
float setRsR0RatioGetPPM(float value);
float readCO2();
float readCO();
float readALCOHOL();
float readAMMONIUM();
float readTOULENE();
float readACETONE();

#endif // MQsensor_H_H