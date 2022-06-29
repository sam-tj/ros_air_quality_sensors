#include "MQUnifiedsensor.h"

void MQUnifiedsensor(float Voltage_Resolution, int ADC_Bit_Resolution, int pin)
{
  _pin = pin;
  _VOLT_RESOLUTION = Voltage_Resolution;
  _ADC_Bit_Resolution = ADC_Bit_Resolution;
}

// MQUnifiedsensor(String Placa, float Voltage_Resolution, int ADC_Bit_Resolution, int pin, String type) {
//   this->_pin = pin;
//   Placa.toCharArray(this->_placa, 20);
//   type.toCharArray(this->_type, 6);
//   //this->_type = type; //MQ-2, MQ-3 ... MQ-309A
//   //this->_placa = Placa;
//   this-> _VOLT_RESOLUTION = Voltage_Resolution;
//   this-> _ADC_Bit_Resolution = ADC_Bit_Resolution;
// }
// MQUnifiedsensor(String Placa, String type) {
//   Placa.toCharArray(this->_placa, 20);
//   type.toCharArray(this->_type, 6);
// }
// void init()
// {
//   pinMode(_pin, INPUT);
// }
// void setA(float a)
// {
//   _a = a;
// }
// void setB(float b)
// {
//   _b = b;
// }

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
  // this->_regressionMethod = regressionMethod;
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
// String getRegressionMethod()
// {
//   if(_regressionMethod == 1) return "Exponential";
//   else return "Linear";
// }
// float getA()
// {
//   return _a;
// }
// float getB()
// {
//   return _b;
// }
// void serialDebug(bool onSetup)
// {
//   if(onSetup)
//   {
//     Serial.println();
//     Serial.println("************************************************************************************************************************************************");
//     Serial.println("MQ sensor reading library for arduino");

//     Serial.println("Note: remember that all the parameters below can be modified during the program execution with the methods:");
//     Serial.println("setR0, setRL, setA, setB where you will have to send as parameter the new value, example: mySensor.setR0(20); //R0 = 20KΩ");

//     Serial.println("Authors: Miguel A. Califa U - Yersson R. Carrillo A - Ghiordy F. Contreras C");
//     Serial.println("Contributors: Andres A. Martinez - Juan A. Rodríguez - Mario A. Rodríguez O ");

//     Serial.print("Sensor: "); Serial.println(_type);
//     Serial.print("Supply voltage: "); Serial.print(_VOLT_RESOLUTION); Serial.println(" VDC");
//     Serial.print("ADC Resolution: "); Serial.print(_ADC_Bit_Resolution); Serial.println(" Bits");
//     Serial.print("R0: "); Serial.print(_R0); Serial.println(" KΩ");
//     Serial.print("RL: "); Serial.print(_RL); Serial.println(" KΩ");

//     Serial.print("Model: "); if(_regressionMethod == 1) Serial.println("Exponential"); else Serial.println("Linear");
//     Serial.print(_type); Serial.print(" -> a: "); Serial.print(_a); Serial.print(" | b: "); Serial.println(_b);

//     Serial.print("Development board: "); Serial.println(_placa);
//   }
//   else
//   {
//     if(!_firstFlag)
//     {
//       Serial.print("| ********************************************************************"); Serial.print(_type); Serial.println("*********************************************************************|");
//       Serial.println("|ADC_In | Equation_V_ADC | Voltage_ADC |        Equation_RS        | Resistance_RS  |    EQ_Ratio  | Ratio (RS/R0) | Equation_PPM |     PPM    |");
//       _firstFlag = true;  //Headers are printed
//     }
//     else
//     {
//       Serial.print("|"); Serial.print(_adc);  Serial.print("| v = ADC*"); Serial.print(_VOLT_RESOLUTION); Serial.print("/"); Serial.print((pow(2, _ADC_Bit_Resolution)) - 1); Serial.print("  |    "); Serial.print(_sensor_volt);
//       Serial.print("     | RS = ((" ); Serial.print(_VOLT_RESOLUTION ); Serial.print("*RL)/Voltage) - RL|      "); Serial.print(_RS_Calc); Serial.print("     | Ratio = RS/R0|    ");
//       Serial.print(_ratio);  Serial.print( "       |   ");
//       if(_regressionMethod == 1) Serial.print("ratio*a + b");
//       else Serial.print("pow(10, (log10(ratio)-b)/a)");
//       Serial.print("  |   "); Serial.print(_PPM); Serial.println("  |");
//     }
//   }
// }

void update()
{
  _sensor_volt = getVoltage( _read ,  _injected,  _value );
}
void externalADCUpdate(float volt)
{
  _sensor_volt = volt;
}
// float validateEcuation(float ratioInput)
// {
//   // Serial.print("Ratio input: "); Serial.println(ratioInput);
//   // Serial.print("a: "); Serial.println(_a);
//   // Serial.print("b: "); Serial.println(_b);
//   // Usage of this function: Unit test on ALgorithmTester example;
//   if (_regressionMethod == 1)
//     _PPM = _a * pow(ratioInput, _b);
//   else
//   {
//     // https://jayconsystems.com/blog/understanding-a-gas-sensor
//     double ppm_log = (log10(ratioInput) - _b) / _a; // Get ppm value in linear scale according to the the ratio value
//     _PPM = pow(10, ppm_log);                        // Convert ppm value to log scale
//   }
//   // Serial.println("Regression Method: "); Serial.println(_regressionMethod);
//   // Serial.println("Result: "); Serial.println(_PPM);
//   return _PPM;
// }
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
/*
float stringTofloat(String & str)
{
  return atof( str.c_str() );
}
*/
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
