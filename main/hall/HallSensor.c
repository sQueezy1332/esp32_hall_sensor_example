//#include "HallSensor.h"
#include "esp_idf_version.h"
//#include "esp_log.h"
#if (ESP_IDF_VERSION_MAJOR < 6)
#define CONFIG_ADC_SUPPRESS_DEPRECATE_WARN 1
#include "driver/adc.h"
#else
#include "esp_adc/adc_oneshot.h" //dont work
#endif
//https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/docs/en/migration-guides/release-5.x/5.0/peripherals.rst#L59
//#include "esp_private/adc_share_hw_ctrl.h" //adc_power_acquire
#include "freertos/portmacro.h"
#include "hal/adc_ll.h"
//#include "hal/adc_hal_common.h"
__unused static const char* TAG = "ADC";
extern portMUX_TYPE rtc_spinlock;
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
// duplicate this because it is static in adc_legacy.h
static esp_err_t adc_hal_convert(adc_unit_t adc_n, int channel, int *out_raw)
{
    const uint32_t event = (adc_n == ADC_UNIT_1) ? ADC_LL_EVENT_ADC1_ONESHOT_DONE : ADC_LL_EVENT_ADC2_ONESHOT_DONE;
    adc_oneshot_ll_clear_event(event);  //For compatibility (empty for esp32)
    adc_oneshot_ll_disable_all_unit();  //For compatibility
    adc_oneshot_ll_enable(adc_n);       //For compatibility
    adc_oneshot_ll_set_channel(adc_n, channel);
    adc_oneshot_ll_start(adc_n); // adc_hal_onetime_start(adc_n);
    while (adc_oneshot_ll_get_event(event) != true) {;}
    *out_raw = adc_oneshot_ll_get_raw_result(adc_n);
    if (adc_oneshot_ll_raw_check_valid(adc_n, *out_raw) == false) return ESP_ERR_INVALID_STATE; // No arbiter, don't need check data
    //HW workaround: when enabling periph clock, this should be false
    adc_oneshot_ll_disable_all_unit(); //For compatibility (empty for esp32)
    return ESP_OK;
}

static int adc_hal_hall_result(void)
{
    int Sens_Vp0, Sens_Vn0, Sens_Vp1, Sens_Vn1;
    // convert for 4 times with different phase and outputs
    adc_ll_hall_phase_disable(); // hall phase
    adc_hal_convert(ADC_UNIT_1, ADC_CHANNEL_0, &Sens_Vp0);
    adc_hal_convert(ADC_UNIT_1, ADC_CHANNEL_3, &Sens_Vn0);
    adc_ll_hall_phase_enable();
    adc_hal_convert(ADC_UNIT_1, ADC_CHANNEL_0, &Sens_Vp1);
    adc_hal_convert(ADC_UNIT_1, ADC_CHANNEL_3, &Sens_Vn1);
    return (Sens_Vp1 - Sens_Vp0) - (Sens_Vn1 - Sens_Vn0);
}

int hall_sensor_read(void)    //hall sensor without LNA //_get_value()
{
    //adc_power_acquire();
    portENTER_CRITICAL(&rtc_spinlock); //ADC_ENTER_CRITICAL();
    //adc_ll_amp_disable(); // disable other peripherals.           //moved to init
    //RTCIO.hall_sens.xpd_hall = 1;  //adc_hal_hall_enable();  //moved to init
    //adc_ll_set_controller(ADC_UNIT_1, ADC_LL_CTRL_RTC);  // adc_hal_set_controller(ADC_UNIT_1, ADC_CTRL_RTC); //moved to init
    int hall_value = adc_hal_hall_result();
    //RTCIO.hall_sens.xpd_hall = 0;  // adc_hal_hall_disable();
    portEXIT_CRITICAL(&rtc_spinlock); // ADC_EXIT_CRITICAL();
    //adc_power_release();
    return hall_value;
}

void hall_sensor_init(void) {
#if (ESP_IDF_VERSION_MAJOR < 6)
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_0);
#else
	adc_oneshot_unit_handle_t adc1_handle;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&(adc_oneshot_unit_init_cfg_t) { ADC_UNIT_1 }, &adc1_handle));
    adc_oneshot_chan_cfg_t config = { ADC_ATTEN_DB_0, ADC_BITWIDTH_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config));
    adc_ll_amp_disable();
    adc_ll_hall_enable();    // adc_hal_hall_enable(); //RTCIO.hall_sens.xpd_hall = 1;
    adc_ll_set_controller(ADC_UNIT_1, ADC_LL_CTRL_RTC);     // adc_hal_set_controller(ADC_UNIT_1, ADC_CTRL_RTC);
#endif
}

void hall_sensor_deinit(void) {
    portENTER_CRITICAL(&rtc_spinlock);
    adc_ll_hall_disable();  //adc_hal_hall_disable(); //RTCIO.hall_sens.xpd_hall = 0;   
    portEXIT_CRITICAL(&rtc_spinlock);
}


/* int hall_sensor_read(void) {
    //adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);
    //adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_0);
    return hall_sensor_get_value();
} */
