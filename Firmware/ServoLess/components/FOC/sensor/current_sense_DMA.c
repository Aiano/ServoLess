#include "current_sense.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_continuous.h"

#define CS_CONV_MODE ADC_CONV_SINGLE_UNIT_2  // Only use ADC2
#define CS_ADC_UNIT ADC_UNIT_2
#define CS_OUTPUT_FORMAT ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define CS_SAMPLE_FREQ 500
#define CS_CHANNEL_NUM 2
#define CS_READ_LEN 256

static const char *TAG = "current sense";

static adc_channel_t channel[2] = {ADC_CHANNEL_1, ADC_CHANNEL_2};
static adc_continuous_handle_t handle = NULL;

static bool check_valid_data(const adc_digi_output_data_t *data)
{
    if (data->type2.channel >= SOC_ADC_CHANNEL_NUM(ADC_UNIT_2)) {
        return false;
    }
    return true;
}

esp_err_t ret;
uint8_t result[CS_READ_LEN] = {0};
uint32_t ret_num = 0;
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    ret = adc_continuous_read(handle, result, CS_READ_LEN, &ret_num, 0);
    if(ret == ESP_OK){
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *p = (void*)&result[i];
            if (check_valid_data(p)) {
                ESP_LOGI(TAG, "Unit: %d, Channel: %d, Value: %x", 2, p->type2.channel, p->type2.data);
            }else{
                ESP_LOGI(TAG, "Invalid data");
            }
        }
    }
    return true;
}

/**
 * Init continuous ADC driver
 */
void cs_init(){
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = CS_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = CS_SAMPLE_FREQ,
        .conv_mode = CS_CONV_MODE,
        .format = CS_OUTPUT_FORMAT,
    };

    adc_digi_pattern_config_t adc_pattern[CS_CHANNEL_NUM] = {0};
    dig_cfg.pattern_num = CS_CHANNEL_NUM;
    for (int i = 0; i < CS_CHANNEL_NUM; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_0;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = CS_ADC_UNIT;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    return;
}