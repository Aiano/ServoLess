file(GLOB SOURCES FOC.c FOC_utils.c FOC_LPF.c FOC_PID.c sensor/AS5048A.c sensor/current_sense.c hardware_specific/esp32_mcu.c)

# This component uses mcpwm (which contained in component "driver")
# When header uses mcpwm, add "REQUIRES driver"
# When source uses mcpwm, add "PRIV_REQUIRES driver" unless it has been quoted in "REQUIRES"

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "." "hardware_specific" "sensor"
                    PRIV_REQUIRES driver esp_adc)
