#include "current_sense.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define CS_CHANNEL1 ADC_CHANNEL_1
#define CS_CHANNEL3 ADC_CHANNEL_2

const static char *TAG = "current sense";
const static float mV_to_A = 1 / (1000.0 * CS_INA_GAIN * CS_SAMPLE_RESISTOR);

int cs_adc_raw[2][10];
int cs_adc_offset_voltage[2];
float cs_current[3];
adc_oneshot_unit_handle_t adc2_handle;
adc_cali_handle_t adc2_cali_handle = NULL;
bool do_calibration;

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

void cs_init()
{
    //-------------ADC2 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    //-------------ADC2 Calibration Init---------------//

    do_calibration = example_adc_calibration_init(ADC_UNIT_2, ADC_ATTEN_DB_11, &adc2_cali_handle);

    //-------------ADC2 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, CS_CHANNEL1, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, CS_CHANNEL3, &config));
}

void cs_read()
{
    // Read Motor CS1
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, CS_CHANNEL1, &cs_adc_raw[0][0]));
    if (do_calibration)
    {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_handle, cs_adc_raw[0][0], &cs_adc_offset_voltage[0]));
    }

    // Read Motor CS3
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, CS_CHANNEL3, &cs_adc_raw[1][0]));
    if (do_calibration)
    {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_handle, cs_adc_raw[1][0], &cs_adc_offset_voltage[1]));
    }

    // Use KCL to calculate the second channel voltage
    cs_current[0] = (cs_adc_offset_voltage[0]- CS_OFFSET_VOLTAGE) * mV_to_A;
    cs_current[2] = (cs_adc_offset_voltage[1]- CS_OFFSET_VOLTAGE) * mV_to_A;
    cs_current[1] = - cs_current[0] - cs_current[2];
}