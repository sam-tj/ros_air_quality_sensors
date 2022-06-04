#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

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
#include "components/MQ-2-sensor-library/MQ2.c"

#define ADC1_CHANNEL_4 ADC1_CHANNEL_4 //ADC1 channel 4 is GPIO32 (ESP32)

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

static const adc_bits_width_t width = ADC_WIDTH_BIT_10;

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
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{

		const unsigned int PUB_MSG_CAPACITY = 30;
		msg.counter = counter++;		
		//msg.time = malloc(PUB_MSG_CAPACITY);
		//snprintf(msg.time, PUB_MSG_CAPACITY, "Time now  hh:" );
		msg.co2 = co2;
		msg.temperature = temp;
		msg.r0value = R0val;
		msg.lpg = readLPG();
		msg.co = readCO();
		msg.smoke = readSmoke();

		//print values in serial monitor
		printf("\nanalog read = %d", adc1_get_raw(ADC1_CHANNEL_4));

		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
	}
}

void appMain(void *arg)
{
	if (timeinfo.tm_year < (2016 - 1900))
	{
		printf("Time is not set yet. Connecting to WiFi and getting time over NTP.");
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
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
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

static void initialize_mhz19b(void)
{
	// sesnsor init
	RCCHECK(mhz19b_init(&dev, UART_NUM_1, 12, 13));

	while (!mhz19b_detect(&dev))
	{
		printf("MHZ-19B not detected, waiting...\n");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

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

static void mhz19b_warmup(void)
{
	//memcpy(msg.data.data, array_init, sizeof(array_init_size));

	while (mhz19b_is_warming_up(&dev, true)) // use smart warming up detection
	{
		printf("MHZ-19B is warming up\n");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
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
	MQ2pin(ADC1_CHANNEL_4);
	R0val = begin();
}

static void initialize_sntp(void)
{
	sntp_setoperatingmode(SNTP_OPMODE_POLL);
	sntp_setservername(0, "192.168.0.11"); // update ip with respect to the ntp server
	sntp_init();
	int retry = 0;
	const int retry_count = 15;
	while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count)
	{
		printf("Waiting for system time to be set... (%d/%d)", retry, retry_count);
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