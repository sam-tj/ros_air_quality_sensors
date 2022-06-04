#include "MQ2.h"
#include <math.h>
#include "freertos/task.h"

void MQ2pin(int pin)
{
  _pin = pin;
}
float begin()
{
  Ro = MQCalibration();
  return Ro;
}

float *readMQ2()
{

  lpg = MQGetGasPercentage(MQRead() / Ro, GAS_LPG);
  co = MQGetGasPercentage(MQRead() / Ro, GAS_CO);
  smoke = MQGetGasPercentage(MQRead() / Ro, GAS_SMOKE);
  lastReadTime = esp_timer_get_time() / 1000;
  float values[3] = {lpg, co, smoke};
  return values;
}

float readLPG()
{
  if (esp_timer_get_time() / 1000 < (lastReadTime + 10000) && lpg != 0)
  {
    return lpg;
  }
  else
  {
    return lpg = MQGetGasPercentage(MQRead() / 10, GAS_LPG);
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
    return co = MQGetGasPercentage(MQRead() / 10, GAS_CO);
  }
}

float readSmoke()
{
  if (esp_timer_get_time() / 1000 < (lastReadTime + 10000) && smoke != 0)
  {
    return smoke;
  }
  else
  {
    return smoke = MQGetGasPercentage(MQRead() / 10, GAS_SMOKE);
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
  if (gas_id == GAS_LPG)
  {
    return MQGetPercentage(rs_ro_ratio, LPGCurve);
  }
  else if (gas_id == GAS_CO)
  {
    return MQGetPercentage(rs_ro_ratio, COCurve);
  }
  else if (gas_id == GAS_SMOKE)
  {
    return MQGetPercentage(rs_ro_ratio, SmokeCurve);
  }
  return 0;
}

int MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10, (((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}
