#include "MQsensor.h"

void MQsensor(float Voltage_Resolution, int ADC_Bit_Resolution, int pin)
{
  _pin = pin;
  _VOLT_RESOLUTION = Voltage_Resolution;
  _ADC_Bit_Resolution = ADC_Bit_Resolution;
}
void setR0(float R0)
{
  _R0 = R0;
}
void setRL(float RL)
{
  _RL = RL;
}
void setADC(int value)
{
  _sensor_volt = (value)*_VOLT_RESOLUTION / ((pow(2, _ADC_Bit_Resolution)) - 1);
  _adc = value;
}
void setVoltResolution(float voltage_resolution)
{
  _VOLT_RESOLUTION = voltage_resolution;
}
void setRegressionMethod(int regressionMethod)
{
  _regressionMethod = regressionMethod;
}
float getR0()
{
  return _R0;
}
float getRL()
{
  return _RL;
}
float getVoltResolution()
{
  return _VOLT_RESOLUTION;
}
void updateMQ()
{
  _sensor_volt = getVoltage( _read ,  _injected,  _value );
}
void externalADCUpdate(float volt)
{
  _sensor_volt = volt;
}
float readSensor(bool isMQ303A, float correctionFactor, bool injected)
{
  // More explained in: https://jayconsystems.com/blog/understanding-a-gas-sensor
  if (isMQ303A)
  {
    _VOLT_RESOLUTION = _VOLT_RESOLUTION - 0.45; // Calculations for RS using mq303a sensor look wrong #42
  }
  _RS_Calc = ((_VOLT_RESOLUTION * _RL) / _sensor_volt) - _RL; // Get value of RS in a gas
  if (_RS_Calc < 0)
    _RS_Calc = 0; // No negative values accepted.
  if (!injected)
    _ratio = _RS_Calc / _R0; // Get ratio RS_gas/RS_air
  _ratio += correctionFactor;
  if (_ratio <= 0)
    _ratio = 0; // No negative values accepted or upper datasheet recomendation.
  if (_regressionMethod == 1)
    _PPM = _a * pow(_ratio, _b); // <- Source excel analisis https://github.com/miguel5612/MQSensorsLib_Docs/tree/master/Internal_design_documents
  else
  {
    // https://jayconsystems.com/blog/understanding-a-gas-sensor <- Source of linear ecuation
    double ppm_log = (log10(_ratio) - _b) / _a; // Get ppm value in linear scale according to the the ratio value
    _PPM = pow(10, ppm_log);                    // Convert ppm value to log scale
  }
  if (_PPM < 0)
    _PPM = 0; // No negative values accepted or upper datasheet recomendation.
  // if(_PPM > 10000) _PPM = 99999999; //No negative values accepted or upper datasheet recomendation.
  return _PPM;
}
float readSensorR0Rs()
{
  // More explained in: https://jayconsystems.com/blog/understanding-a-gas-sensor
  _RS_Calc = ((_VOLT_RESOLUTION * _RL) / _sensor_volt) - _RL; // Get value of RS in a gas
  if (_RS_Calc < 0)
    _RS_Calc = 0;          // No negative values accepted.
  _ratio = _R0 / _RS_Calc; // Get ratio RS_air/RS_gas <- INVERTED for MQ-131 issue 28 https://github.com/miguel5612/MQSensorsLib/issues/28
  if (_ratio <= 0)
    _ratio = 0; // No negative values accepted or upper datasheet recomendation.
  if (_regressionMethod == 1)
    _PPM = _a * pow(_ratio, _b); // <- Source excel analisis https://github.com/miguel5612/MQSensorsLib_Docs/tree/master/Internal_design_documents
  else
  {
    // https://jayconsystems.com/blog/understanding-a-gas-sensor <- Source of linear ecuation
    double ppm_log = (log10(_ratio) - _b) / _a; // Get ppm value in linear scale according to the the ratio value
    _PPM = pow(10, ppm_log);                    // Convert ppm value to log scale
  }
  if (_PPM < 0)
    _PPM = 0; // No negative values accepted or upper datasheet recomendation.
  // if(_PPM > 10000) _PPM = 99999999; //No negative values accepted or upper datasheet recomendation.
  return _PPM;
}
float calibrate(float ratioInCleanAir)
{
  // More explained in: https://jayconsystems.com/blog/understanding-a-gas-sensor
  /*
  V = I x R
  VRL = [VC / (RS + RL)] x RL
  VRL = (VC x RL) / (RS + RL)
  Así que ahora resolvemos para RS:
  VRL x (RS + RL) = VC x RL
  (VRL x RS) + (VRL x RL) = VC x RL
  (VRL x RS) = (VC x RL) - (VRL x RL)
  RS = [(VC x RL) - (VRL x RL)] / VRL
  RS = [(VC x RL) / VRL] - RL
  */
  float RS_air;                                             // Define variable for sensor resistance
  float R0;                                                 // Define variable for R0
  RS_air = ((_VOLT_RESOLUTION * _RL) / _sensor_volt) - _RL; // Calculate RS in fresh air
  if (RS_air < 0)
    RS_air = 0;                  // No negative values accepted.
  R0 = RS_air / ratioInCleanAir; // Calculate R0
  if (R0 < 0)
    R0 = 0; // No negative values accepted.
  return R0;
}
float getVoltage(bool read, bool injected, int value)
{
  float voltage;
  if (read)
  {
    float avg = 0.0;
    for (int i = 0; i < retries; i++)
    {
      _adc = adc1_get_raw(_pin);
      avg += _adc;
      vTaskDelay(retry_interval / portTICK_RATE_MS);
    }
    voltage = (avg / retries) * _VOLT_RESOLUTION / ((pow(2, _ADC_Bit_Resolution)) - 1);
  }
  else if (!injected)
  {
    voltage = _sensor_volt;
  }
  else
  {
    voltage = (value)*_VOLT_RESOLUTION / ((pow(2, _ADC_Bit_Resolution)) - 1);
    _sensor_volt = voltage; // to work on testing
  }
  return voltage;
}
float setRsR0RatioGetPPM(float value)
{
  _ratio = value;
  return readSensor(false, 0, true);
}
float getRS()
{
  // More explained in: https://jayconsystems.com/blog/understanding-a-gas-sensor
  _RS_Calc = ((_VOLT_RESOLUTION * _RL) / _sensor_volt) - _RL; // Get value of RS in a gas
  if (_RS_Calc < 0)
    _RS_Calc = 0; // No negative values accepted.
  return _RS_Calc;
}
float readCO2()
{
  _a = 110.47;
  _b = -2.862;
  float temp = readSensor(_isMQ303A, _correctionFactor , _injected );
  return temp;
}
float readCO()
{
  _a = 605.18;
  _b = -3.937;
  float temp = readSensor(_isMQ303A, _correctionFactor , _injected );
  return temp;
}
float readALCOHOL()
{
  _a = 77.255;
  _b = -3.18;
  float temp = readSensor(_isMQ303A, _correctionFactor , _injected );
  return temp;
}
float readAMMONIUM()
{
  _a = 102.2;
  _b = -2.473;
  float temp = readSensor(_isMQ303A, _correctionFactor , _injected );
  return temp;
}
float readTOULENE()
{
  _a = 44.947;
  _b = -3.445;
  float temp = readSensor(_isMQ303A, _correctionFactor , _injected );
  return temp;
}
float readACETONE()
{
  _a = 34.668;
  _b = -3.369;
  float temp = readSensor(_isMQ303A, _correctionFactor , _injected );
  return temp;
}
