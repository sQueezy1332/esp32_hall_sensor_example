#include "ESP_MAIN.h"
#include "HallSensor.h"
#include "driver/gpio.h"
#define PIN_LED_BLUE GPIO_NUM_2
#define PIN_LED_RED GPIO_NUM_4
#define PIN_BTN 0
#define KALMAN_KOEF 0.5f
#define AVER_COUNT 64
#define MAX_THOLD 35	//for 160mhz
#define MIN_THOLD 26

int Kalman(int res){
    static float old = res;
    res = KALMAN_KOEF * res + (1 - KALMAN_KOEF) * old;
    return old = res;
}
bool led;

void led_positive() {
	digitalWrite(PIN_LED_RED, led = 1);
	digitalWrite(PIN_LED_BLUE, 0);
}

void led_negative() {
	digitalWrite(PIN_LED_BLUE, led = 1);
	digitalWrite(PIN_LED_RED, 0);
}
void led_off() {
	led = 0;
	digitalWrite(PIN_LED_BLUE, 0); 
	digitalWrite(PIN_LED_RED, 0); //-1 is true
}

extern "C" void app_main() 
{
    main_init();
    pinMode(PIN_LED_BLUE, OUTPUT);
	pinMode(PIN_LED_RED, OUTPUT);
	pinMode(PIN_BTN, INPUT);
	//gpio_set_drive_capability(PIN_LED_BLUE, GPIO_DRIVE_CAP_3);
    digitalWrite(PIN_LED_BLUE, 1);
    hall_sensor_init();
	delay(1000);
	DEBUGF("THRESHOLD: %u - %u\nStart reading hall sensor ...\n",MIN_THOLD, MAX_THOLD);
	digitalWrite(PIN_LED_BLUE, 0);
    for (int result = 0, lf = 0, i;;delay(10)) {
		//int timer = uS;
		for (i = 0; i < AVER_COUNT; i++){
			result += hall_sensor_read();
		}
		result /= AVER_COUNT;
		//ESP_LOGI("time","diff %lu\n",(uint32_t)uS - timer); //2895 //~45 uSec for 1 measure
		result = Kalman(result);
		char magnet = result > MAX_THOLD ? 1 : result < MIN_THOLD ? -1 : 0;
		if(magnet || !digitalRead(PIN_BTN)) { 
			switch (magnet) {
			case (char)-1:led_negative(); break; //gpio_set_direction(PIN_LED, GPIO_MODE_INPUT); 
			case 1: led_positive(); break;//gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT); 
			default: led_off(); break;
			}
			DEBUG(result); DEBUG(' ');
			if(++lf == 20) { lf = 0; DEBUGLN();}
		} else if(led) { led_off(); }
    }
}
