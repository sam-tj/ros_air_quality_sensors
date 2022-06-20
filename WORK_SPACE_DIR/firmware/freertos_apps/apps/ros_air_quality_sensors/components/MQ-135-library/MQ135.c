#include "MQ135.h"
#include <math.h>
#include "freertos/task.h"

void MQ135pin(int pin)
{
  _pin = pin;
}
float begin()
{
  Ro = MQCalibration();
  return Ro;
}

float *readMQ135()
{

  co2_mq135 = MQGetGasPercentage(MQRead() / Ro, GAS_CARBON_DIOXIDE);
  co = MQGetGasPercentage(MQRead() / Ro, GAS_CARBON_MONOXIDE);
  alcohol = MQGetGasPercentage(MQRead() / Ro, GAS_ALCOHOL);
  ammonium = MQGetGasPercentage(MQRead() / Ro, GAS_AMMONIUM);
  toulene = MQGetGasPercentage(MQRead() / Ro, GAS_TOLUENE);
  acetone = MQGetGasPercentage(MQRead() / Ro, GAS_ACETONE);

  lastReadTime = esp_timer_get_time() / 1000;
  float values[6] = {co2_mq135, co, alcohol, ammonium, toulene, acetone};
  return values;
}

float readCO2()
{
  if (esp_timer_get_time() / 1000 < (lastReadTime + 10000) && co2_mq135 != 0)
  {
    return co2_mq135;
  }
  else
  {
    return co2_mq135 = MQGetGasPercentage(MQRead() / 10, GAS_CARBON_DIOXIDE);
  }
}
float readCO()
{
  if (esp_timer_get_time() / 1000 < (lastReadTime + 10000) && co != 0)
  {
    return co;
  }
  else
  {
    return co = MQGetGasPercentage(MQRead() / 10, GAS_CARBON_MONOXIDE);
  }
}
float readALCOHOL()
{
  if (esp_timer_get_time() / 1000 < (lastReadTime + 10000) && alcohol != 0)
  {
    return alcohol;
  }
  else
  {
    return alcohol = MQGetGasPercentage(MQRead() / 10, GAS_ALCOHOL);
  }
}
float readAMMONIUM()
{
  if (esp_timer_get_time() / 1000 < (lastReadTime + 10000) && ammonium != 0)
  {
    return ammonium;
  }
  else
  {
    return ammonium = MQGetGasPercentage(MQRead() / 10, GAS_AMMONIUM);
  }
}
float readTOULENE()
{
  if (esp_timer_get_time() / 1000 < (lastReadTime + 10000) && toulene != 0)
  {
    return toulene;
  }
  else
  {
    return toulene = MQGetGasPercentage(MQRead() / 10, GAS_TOLUENE);
  }
}
float readACETONE()
{
  if (esp_timer_get_time() / 1000 < (lastReadTime + 10000) && acetone != 0)
  {
    return acetone;
  }
  else
  {
    return acetone = MQGetGasPercentage(MQRead() / 10, GAS_ACETONE);
  }
}

float MQResistanceCalculation(int raw_adc)
{
  return (((float)RL_VALUE * (1023 - raw_adc) / raw_adc));
}

float MQCalibration()
{
  float val = 0;

  for (int i = 0; i < CALIBARAION_SAMPLE_TIMES; i++)
  { //take multiple samples
    val += MQResistanceCalculation(adc1_get_raw(_pin));
    vTaskDelay(CALIBRATION_SAMPLE_INTERVAL / portTICK_RATE_MS);
  }
  val = val / CALIBARAION_SAMPLE_TIMES; //calculate the average value

  val = val / RO_CLEAN_AIR_FACTOR; //divided by RO_CLEAN_AIR_FACTOR yields the Ro
                                   //according to the chart in the datasheet
  return val;
}

float MQRead()
{
  float rs = 0;
  int val = adc1_get_raw(_pin);

  for (int i = 0; i < READ_SAMPLE_TIMES; i++)
  {
    rs += MQResistanceCalculation(val);
    vTaskDelay(READ_SAMPLE_INTERVAL / portTICK_RATE_MS);
  }

  rs = rs / READ_SAMPLE_TIMES;
  return rs;
}

float MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if (gas_id == GAS_CARBON_DIOXIDE)
  {
    return MQGetPercentage(rs_ro_ratio, CO2Curve);
  }
  else if (gas_id == GAS_CARBON_MONOXIDE)
  {
    return MQGetPercentage(rs_ro_ratio, COCurve);
  }
  else if (gas_id == GAS_ALCOHOL)
  {
    return MQGetPercentage(rs_ro_ratio, ALCOHOLCurve);
  }
  else if (gas_id == GAS_AMMONIUM)
  {
    return MQGetPercentage(rs_ro_ratio, AMMONIUMCurve);
  }
  else if (gas_id == GAS_TOLUENE)
  {
    return MQGetPercentage(rs_ro_ratio, TOLUENECurve);
  }
  else if (gas_id == GAS_ACETONE)
  {
    return MQGetPercentage(rs_ro_ratio, ACETONECurve);
  }
  return 0;
}

float MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10, ((pcurve[0] * (log10(rs_ro_ratio))) + pcurve[1])));
}
