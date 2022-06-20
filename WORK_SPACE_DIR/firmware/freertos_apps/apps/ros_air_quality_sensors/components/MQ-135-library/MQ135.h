#ifndef MQ135_h
#define MQ135_h

#endif

void MQ135pin(int pin);

int _pin;
int RL_VALUE = 22.5; //define the load resistance on the board, in kilo ohms
int RO_CLEAN_AIR_FACTOR = 3.6;
int CALIBARAION_SAMPLE_TIMES = 5;
int CALIBRATION_SAMPLE_INTERVAL = 50;
int READ_SAMPLE_INTERVAL = 50;
int READ_SAMPLE_TIMES = 5;

float CO2Curve[2] = {-2.890, 2.055};
float COCurve[2] = {-3.891, 2.750};
float ALCOHOLCurve[2] = {-3.181, 1.895};
float AMMONIUMCurve[2] = {-2.469, 2.005};
float TOLUENECurve[2] = {-3.479, 1.658};
float ACETONECurve[2] = {-3.452, 1.542};
float Ro = 10;

int GAS_CARBON_DIOXIDE = 0;
int GAS_CARBON_MONOXIDE = 1;
int GAS_ALCOHOL = 2;
int GAS_AMMONIUM = 3;
int GAS_TOLUENE = 4;
int GAS_ACETONE = 5;

float co2_mq135 = 0;
float co = 0;
float alcohol = 0;
float ammonium = 0;
float toulene = 0;
float acetone = 0;

float MQRead();
float *readMQ135();
float readCO2();
float readCO();
float readALCOHOL();
float readAMMONIUM();
float readTOULENE();
float readACETONE();
float begin();
float MQGetGasPercentage(float rs_ro_ratio, int gas_id);
float MQGetPercentage(float rs_ro_ratio, float *pcurve);
float MQCalibration();
float MQResistanceCalculation(int raw_adc);

int lastReadTime = 0;

//#endif
