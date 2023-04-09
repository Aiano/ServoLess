#include "esp32_mcu.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_log.h"

const static char *TAG = "esp32_mcu";

static mcpwm_cmpr_handle_t comparators[3];

void _init3PWM()
{
    /**
     * FOC 3PWM driver needs 1 timer and 3 comparators
     * to generate 3 phases symmetry synchronized PWM waves
     **/

    ESP_LOGI(TAG, "Create timers");
    mcpwm_timer_handle_t timer;
    mcpwm_timer_config_t timer_config = {
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .group_id = 0,
        .resolution_hz = TIMER_RESOLUTION_HZ,
        // period_ticks should be 2 times compare range, when count mode is UP and DOWN
        .period_ticks = 2 * TIMER_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    ESP_LOGI(TAG, "Create operators");
    mcpwm_oper_handle_t operators[3];
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator should be in the same group of the above timers
    };
    for (int i = 0; i < 3; ++i)
    {
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));
    }

    ESP_LOGI(TAG, "Connect timers and operators with each other");
    for (int i = 0; i < 3; i++)
    {
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], timer));
    }

    ESP_LOGI(TAG, "Create comparators");
    mcpwm_comparator_config_t compare_config = {
        .flags.update_cmp_on_tez = true,
    };
    for (int i = 0; i < 3; i++)
    {
        ESP_ERROR_CHECK(mcpwm_new_comparator(operators[i], &compare_config, &comparators[i]));
        // init compare for each comparator
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[i], (uint32_t)(TIMER_PERIOD / 2)));
    }

    ESP_LOGI(TAG, "Create generators");
    mcpwm_gen_handle_t generators[3];
    const int gen_gpios[3] = {MOTOR_IN1_GPIO, MOTOR_IN2_GPIO, MOTOR_IN3_GPIO};
    mcpwm_generator_config_t gen_config = {};
    for (int i = 0; i < 3; i++)
    {
        gen_config.gen_gpio_num = gen_gpios[i];
        ESP_ERROR_CHECK(mcpwm_new_generator(operators[i], &gen_config, &generators[i]));
    }

    ESP_LOGI(TAG, "Set generator actions on timer and compare event");
    for (int i = 0; i < 3; i++)
    {
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generators[i],
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i], MCPWM_GEN_ACTION_LOW),
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, comparators[i], MCPWM_GEN_ACTION_HIGH),
                    MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

    }

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void _enableDriver(){
    gpio_reset_pin(MOTOR_EN_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(MOTOR_EN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_EN_GPIO, 1);
}

/**
 * @brief write duty cycle for 3 phase motor
 * @param dc_a [0,1]
 * @param dc_b [0,1]
 * @param dc_c [0,1]
 * @param params
 */
void _writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c)
{
    // transform duty cycle from [0,1] to [0,_PWM_RANGE]
    if(dc_a < 0) dc_a = 0;
    else if(dc_a > 1) dc_a = 1;
    if(dc_b < 0) dc_b = 0;
    else if(dc_b > 1) dc_b = 1;
    if(dc_c < 0) dc_c = 0;
    else if(dc_c > 1) dc_c = 1;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[0], TIMER_PERIOD * dc_a));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[1], TIMER_PERIOD * dc_b));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[2], TIMER_PERIOD * dc_c));
}
