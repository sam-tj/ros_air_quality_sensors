#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <time.h>
#include <sys/time.h>
#include "esp_sntp.h"

#include <custom_message/msg/sensdata.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_system.h"
#endif

#include "components/mhz19b/mhz19b.c" //wrong impementation
#include "components/MQSensorsLib/src/MQsensor.c"

#define ADC1_CHANNEL_4 ADC1_CHANNEL_4 //ADC1 channel 4 is GPIO32 (ESP32)
#define Voltage_Resolution 3.09
#define RatioMQ135CleanAir 3.6

#define RCCHECK(fn)                                                                      \
	{                                                                                    \
		rcl_ret_t temp_rc = fn;                                                          \
		if ((temp_rc != RCL_RET_OK))                                                     \
		{                                                                                \
			printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
			vTaskDelete(NULL);                                                           \
		}                                                                                \
	}
#define RCSOFTCHECK(fn)                                                                    \
	{                                                                                      \
		rcl_ret_t temp_rc = fn;                                                            \
		if ((temp_rc != RCL_RET_OK))                                                       \
		{                                                                                  \
			printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
		}                                                                                  \
	}

static const adc_bits_width_t width = ADC_WIDTH_BIT_12; //do change down as well
#define ADC_Bit_Resolution 12

// pin definition
gpio_num_t adc_gpio_num1;

esp_err_t r1;
rcl_publisher_t publisher;
custom_message__msg__Sensdata msg;

time_t now;
struct tm timeinfo;

float R0val = 0;

int16_t co2;
int16_t temp;
mhz19b_dev_t dev;
char version[6];
uint16_t range;
bool autocal;

int16_t counter = 0;

static void message_init(void);
static void initialize_mhz19b(void);
static void mhz19b_warmup(void);
static void mhz19b_reading(void);
void initialize_adc(void);
static void initialize_sntp(void);
static void obtain_time(void);

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	obtain_time();
	mhz19b_reading();
	updateMQ();
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		const unsigned int PUB_MSG_CAPACITY = 31;
		msg.counter = counter++;

		msg.time.data = malloc(PUB_MSG_CAPACITY);
		snprintf(msg.time.data, PUB_MSG_CAPACITY, "Time now hh: %d mm: %d ss: %d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
		msg.time.capacity = PUB_MSG_CAPACITY;

		msg.co2 = co2;
		msg.temperature = temp;
		msg.r0value = R0val;
		if (R0val < Ro_inf)
		{
			msg.co2_mq135 = readCO2()+400;
			msg.co = readCO();
			msg.alcohol = readALCOHOL();
			msg.ammonium = readAMMONIUM();
			msg.toulene = readTOULENE();
			msg.acetone = readACETONE();
		}

		//print values in serial monitor
		printf("\nanalog read = %d", adc1_get_raw(ADC1_CHANNEL_4));

		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		free(msg.time.data);
	}
}

void appMain(void *arg)
{
	if (timeinfo.tm_year < (2016 - 1900))
	{
		printf("Time is not set yet. Connecting to WiFi and getting time over NTP.\n");
		initialize_sntp();
	}
	initialize_adc();
	initialize_mhz19b();
	mhz19b_warmup();

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "sensor_publisher", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(custom_message, msg, Sensdata),
		"sensors"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	message_init();

	while (1)
	{
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node))
	RCCHECK(rcl_node_fini(&node))

	vTaskDelete(NULL);
}

static void message_init(void)
{
	msg.time.data = "";
	msg.counter = 0;
	msg.co2 = 0;
	msg.temperature = 0;
	msg.r0value = 0;
	msg.co2_mq135 = 0;
	msg.co = 0;
	msg.alcohol = 0;
	msg.ammonium = 0;
	msg.toulene = 0;
	msg.acetone = 0;
}

static void initialize_mhz19b(void)
{
	// sesnsor init
	RCCHECK(mhz19b_init(&dev, UART_NUM_1, 12, 13));

	int retry = 0;
	const int retry_count = 5;

	while (!mhz19b_detect(&dev) && ++retry < retry_count)
	{
		printf("MHZ-19B not detected, waiting...(%d/%d)\n", retry, retry_count);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	if (mhz19b_is_ready(&dev))
	{
		mhz19b_get_version(&dev, version);
		printf("MHZ-19B firmware version: %s", version);

		printf("MHZ-19B set range and autocal");

		mhz19b_set_range(&dev, MHZ19B_RANGE_5000);
		mhz19b_set_auto_calibration(&dev, false);

		mhz19b_get_range(&dev, &range);
		printf("range: %d", range);

		mhz19b_get_auto_calibration(&dev, &autocal);
		printf("autocal: %s", autocal ? "ON" : "OFF");
	}
}

static void mhz19b_warmup(void)
{
	if (mhz19b_is_ready(&dev))
	{
		//memcpy(msg.data.data, array_init, sizeof(array_init_size));

		while (mhz19b_is_warming_up(&dev, true)) // use smart warming up detection
		{
			printf("MHZ-19B is warming up\n");
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
	}
}

static void mhz19b_reading(void)
{
	if (mhz19b_is_ready(&dev))
	{
		mhz19b_read_co2(&dev, &co2, &temp);
	}
}

//ADC initialization
void initialize_adc(void)
{
	r1 = adc1_pad_get_io_num(ADC1_CHANNEL_4, &adc_gpio_num1);
	assert(r1 == ESP_OK);
	printf("ADC1 channel_4 %d @ GPIO %d.\n", ADC1_CHANNEL_4, adc_gpio_num1);
	printf("ADC1 init...\n");
	adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_11db);
	adc1_config_width(width);
	MQsensor(Voltage_Resolution, ADC_Bit_Resolution, ADC1_CHANNEL_4);
	setRegressionMethod(1);
	setRL(20);
	float calcR0 = 0;
	for (int i = 1; i <= 10; i++)
	{
		updateMQ(); // Update data, the arduino will read the voltage from the analog pin
		calcR0 += calibrate(RatioMQ135CleanAir);
	}
	setR0(calcR0 / 10);
	if (isinf(calcR0) || calcR0 == 0)
	{
		R0val = Ro_inf;
	}
	else
	{
		R0val = calcR0;
	}
}

static void initialize_sntp(void)
{
	sntp_setoperatingmode(SNTP_OPMODE_POLL);
	sntp_setservername(0, "192.168.0.11"); // update ip with respect to the ntp server
	sntp_init();
	int retry = 0;
	const int retry_count = 5;
	while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count)
	{
		printf("Waiting for system time to be set... (%d/%d)\n", retry, retry_count);
		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}
	time(&now);
	localtime_r(&now, &timeinfo);
	setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1); // update time zone according to requirements ( https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv )
	tzset();
	localtime_r(&now, &timeinfo);
	//strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
}

static void obtain_time(void)
{
	time(&now);
	localtime_r(&now, &timeinfo);
}