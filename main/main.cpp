#define DEBUG_ENABLE
#include "ESP_MAIN.h"
#include "HallSensor.h"
#include "driver/gpio.h"
#define PIN_LED_BLUE GPIO_NUM_2
#define PIN_LED_RED GPIO_NUM_4
#define PIN_BTN 0
#define KALMAN_KOEF 0.5f
#define AVER_COUNT 64
#define MAX_THOLD 33	//for 160mhz
#define MIN_THOLD 27

int Kalman(int res){
    static float old = res;
    res = KALMAN_KOEF * res + (1 - KALMAN_KOEF) * old;
    return old = res;
}
bool led;

void led_positive() { digitalWrite(PIN_LED_RED, led = 1); digitalWrite(PIN_LED_BLUE, 0); }
void led_negative() { digitalWrite(PIN_LED_BLUE, led = 1); digitalWrite(PIN_LED_RED, 0); }
void led_off() { if(led) { digitalWrite(PIN_LED_BLUE, 0);  digitalWrite(PIN_LED_RED, 0);} led = 0; }

extern "C" void app_main() 
{
    main_init();
	//Serial.onReceive(uart_cb, true); //Serial.setRxTimeout(2);
    pinMode(PIN_LED_BLUE, OUTPUT);
	pinMode(PIN_LED_RED, OUTPUT);
	pinMode(PIN_BTN, INPUT);
	//gpio_set_drive_capability(PIN_LED_BLUE, GPIO_DRIVE_CAP_3);
    digitalWrite(PIN_LED_BLUE, led = 1);digitalWrite(PIN_LED_RED, 1);
    hall_sensor_init();
	delay(1000);
	led_off();
	DEBUGF("THRESHOLD: %u - %u\nStart reading hall sensor ...\n",MIN_THOLD, MAX_THOLD);
    for (int result = 0, lf = 0, i;;delay(10)) {
		//int timer = uS;
		for (i = 0; i < AVER_COUNT; i++){
			result += hall_sensor_read();
		}
		result /= AVER_COUNT;
		//ESP_LOGI("time","diff %lu\n",(uint32_t)uS - timer); //2895 //~45 uSec for 1 measure //160mhz
		result = Kalman(result);
		char magnet = result > MAX_THOLD ? 1 : result < MIN_THOLD ? -1 : 0;
			switch (magnet) {
			case (char)-1:led_negative(); break; //gpio_set_direction(PIN_LED, GPIO_MODE_INPUT); 
			case 1: led_positive(); break;//gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT); 
			default: led_off(); 
				if(digitalRead(PIN_BTN)) continue;
				break;
			}
			DEBUG(result); DEBUG(' ');
			if(++lf == 20) { lf = 0; DEBUGLN();}
    }
}
