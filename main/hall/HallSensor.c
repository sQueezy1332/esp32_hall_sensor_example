//#include "HallSensor.h"
#define CONFIG_ADC_SUPPRESS_DEPRECATE_WARN 1
#include "driver/adc.h"
#include "esp_private/adc_share_hw_ctrl.h"
#include "freertos/portmacro.h"
#include "hal/adc_ll.h"
#include "hal/adc_hal_common.h"

extern portMUX_TYPE rtc_spinlock;
// duplicate this because it is static in adc_legacy.h
static esp_err_t adc_hal_convert(adc_unit_t adc_n, int channel, int *out_raw)
{
    uint32_t event = (adc_n == ADC_UNIT_1) ? ADC_LL_EVENT_ADC1_ONESHOT_DONE : ADC_LL_EVENT_ADC2_ONESHOT_DONE;
    adc_oneshot_ll_clear_event(event);
    adc_oneshot_ll_disable_all_unit();
    adc_oneshot_ll_enable(adc_n);
    adc_oneshot_ll_set_channel(adc_n, channel);
    adc_oneshot_ll_start(adc_n); // was adc_hal_onetime_start(adc_n);
    while (adc_oneshot_ll_get_event(event) != true) {;}
    *out_raw = adc_oneshot_ll_get_raw_result(adc_n);
    if (adc_oneshot_ll_raw_check_valid(adc_n, *out_raw) == false) return ESP_ERR_INVALID_STATE;
    //HW workaround: when enabling periph clock, this should be false
    adc_oneshot_ll_disable_all_unit();
    return ESP_OK;
}

static int adc_hal_hall_result(void)
{
    int Sens_Vp0, Sens_Vn0,Sens_Vp1,Sens_Vn1;
    // convert for 4 times with different phase and outputs
    adc_ll_hall_phase_disable(); // hall phase
    adc_hal_convert(ADC_UNIT_1, ADC_CHANNEL_0, &Sens_Vp0);
    adc_hal_convert(ADC_UNIT_1, ADC_CHANNEL_3, &Sens_Vn0);
    adc_ll_hall_phase_enable();
    adc_hal_convert(ADC_UNIT_1, ADC_CHANNEL_0, &Sens_Vp1);
    adc_hal_convert(ADC_UNIT_1, ADC_CHANNEL_3, &Sens_Vn1);
    return (Sens_Vp1 - Sens_Vp0) - (Sens_Vn1 - Sens_Vn0);
}

/* static  */int hall_sensor_read(void)    //hall sensor without LNA //_get_value()
{
    //adc_power_acquire();
    portENTER_CRITICAL(&rtc_spinlock); // was ADC_ENTER_CRITICAL();
    /* disable other peripherals. */
    //adc_ll_amp_disable();
    //RTCIO.hall_sens.xpd_hall = 1;  // was adc_hal_hall_enable();
    // set controller
    //adc_ll_set_controller(ADC_UNIT_1, ADC_LL_CTRL_RTC);  // was adc_hal_set_controller(ADC_UNIT_1, ADC_CTRL_RTC);
    int hall_value = adc_hal_hall_result();
    //RTCIO.hall_sens.xpd_hall = 0;  // was adc_hal_hall_disable();
    portEXIT_CRITICAL(&rtc_spinlock); // was ADC_EXIT_CRITICAL();
    //adc_power_release();
    return hall_value;
}

void hall_sensor_init(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_0);
    adc_ll_amp_disable();
    RTCIO.hall_sens.xpd_hall = 1;  // was adc_hal_hall_enable();
    adc_ll_set_controller(ADC_UNIT_1, ADC_LL_CTRL_RTC);  // was adc_hal_set_controller(ADC_UNIT_1, ADC_CTRL_RTC);
}

void hall_sensor_deinit(void) {
    portENTER_CRITICAL(&rtc_spinlock);
    RTCIO.hall_sens.xpd_hall = 0;  // was adc_hal_hall_disable();
    portEXIT_CRITICAL(&rtc_spinlock);
}

/* int hall_sensor_read(void) {
    //adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);
    //adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_0);
    return hall_sensor_get_value();
} */
