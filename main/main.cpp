#define DEBUG_ENABLE
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
//#include "ESP_MAIN.h"
#include "HallSensor.h"
#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
#include "driver/gpio.h"
//#include "esp_check.h"
#ifdef DEBUG_ENABLE
#define DEBUG(x, ...) printf(x, ##__VA_ARGS__)
#define DEBUGLN() printf("\n")
//#define DEBUGLN(x, ...) printf("%s\n", x, ##__VA_ARGS__)
#define DEBUGF(x, ...) printf(x , ##__VA_ARGS__)
#endif
#define PIN_LED_BLUE GPIO_NUM_2
#define PIN_LED_RED GPIO_NUM_4
#define PIN_BTN 0
#define KALMAN_KOEF 0.5f
#define AVER_COUNT 64
#define MAX_THOLD 60//33	//for 160mhz
#define MIN_THOLD 40//27

static bool led;
static int Kalman(int res) {
	static float old_res = 0;
    res = KALMAN_KOEF * res + (1 - KALMAN_KOEF) * old_res;
    return old_res = res;
}

void led_positive() { gpio_set_level(PIN_LED_RED, led = 1); gpio_set_level(PIN_LED_BLUE, 0); }
void led_negative() { gpio_set_level(PIN_LED_BLUE, led = 1); gpio_set_level(PIN_LED_RED, 0); }
void led_off() { if(led) { gpio_set_level(PIN_LED_BLUE, 0);  gpio_set_level(PIN_LED_RED, 0); led = 0; }  }

void app_main() 
{
	gpio_config(&(gpio_config_t) { (1ULL << PIN_LED_BLUE) | (1ULL << PIN_LED_RED), GPIO_MODE_INPUT_OUTPUT }); //error logs internal
	gpio_config(&(gpio_config_t) { (1ULL << PIN_BTN) , GPIO_MODE_INPUT }); //gpio_set_drive_capability(PIN_LED_BLUE, GPIO_DRIVE_CAP_3);
    led = 1; gpio_set_level(PIN_LED_BLUE, 1); gpio_set_level(PIN_LED_RED, 1);
    hall_sensor_init();
	vTaskDelay(pdMS_TO_TICKS(1000));
	led_off();
	DEBUG("Compiled: " __DATE__ "\t" __TIME__ "\n");
	DEBUGF("THRESHOLD: from %d to %d\nStart reading hall sensor ...\n", MIN_THOLD, MAX_THOLD);
    for (uint32_t result = 0, lf = 0, i;;vTaskDelay(pdMS_TO_TICKS(10))) {
		//int timer = uS;
		for (i = 0; i < AVER_COUNT; i++){
			result += Kalman(hall_sensor_read());
		}
		result /= AVER_COUNT;
		//ESP_LOGI("time","diff %lu\n",(uint32_t)uS - timer); //2895 //~45 uSec for 1 measure //160mhz
		//result = Kalman(result);
		if(result > MAX_THOLD ) led_positive(); 
		else if (result < MIN_THOLD) led_negative();
		else { led_off(); if(gpio_get_level(PIN_BTN)) continue; }
		DEBUGF("%li ", result);
		if(++lf == 16) { lf = 0; DEBUGLN(); }
	}
}
