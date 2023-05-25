#include "AS5048A.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

// SPI related macros
#define AS5048A_SPI_HOST SPI2_HOST
#define AS5048A_SPI_MOSI 4
#define AS5048A_SPI_MISO 3
#define AS5048A_SPI_SCLK 2
#define AS5048A_SPI_CS 1

// AS5048A register address
const uint16_t AS5048A_NOP = 0x0000;
const uint16_t AS5048A_CLEAR_ERROR_FLAG = 0x0001;
const uint16_t AS5048A_PROGRAMMING_CONTROL = 0x0003;
const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_HIGH = 0x0016;
const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_LOW = 0x0017;
const uint16_t AS5048A_DIAG_AGC = 0x3FFD;
const uint16_t AS5048A_MAGNITUDE = 0x3FFE;
const uint16_t AS5048A_ANGLE = 0x3FFF;

static const char *TAG = "AS5048A";
static spi_device_handle_t spi_dev_handle;

/**
 * Init AS5048 related SPI
 */
void AS5048A_init()
{
    spi_bus_config_t buscfg = {
        .miso_io_num = AS5048A_SPI_MISO,
        .mosi_io_num = AS5048A_SPI_MOSI,
        .sclk_io_num = AS5048A_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // Clock out at 10 MHz
        .command_bits = 0,
        .address_bits = 0,
        .mode = 1,                      // SPI mode 1, CPOL=0, CPHA=1
        // Time between CS falling edge and CLK rising edge should
        // be greater than 350ns, so can't simply use harware spi CS.
        .spics_io_num = -1, // CS pin
        .queue_size = 1,                // We want to be able to queue 7 transactions at a time
    };
    
    // CS GPIO init
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << AS5048A_SPI_CS,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    gpio_set_level(AS5048A_SPI_CS, 1);
    ESP_LOGI(TAG, "CS GPIO initialized.");

    // Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(AS5048A_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "Initialized the SPI bus.");

    // Attach AS5048A to the SPI bus
    ESP_ERROR_CHECK(spi_bus_add_device(AS5048A_SPI_HOST, &devcfg, &spi_dev_handle));
    ESP_LOGI(TAG, "Attached AS5048A to the SPI bus.");

    // Clear all errors
    AS5048A_clear_error();

    // Get gain
    uint8_t gain = AS5048A_get_gain();
    ESP_LOGI(TAG, "Gain: %d", gain);
}

/**
 * Utility function to calculate even parity bit from information bit
 */
static uint16_t cal_even_parity_bit(uint16_t info_bits)
{
    uint16_t cnt = 0;
    for (uint8_t i = 0; i < 16; i++)
    {
        if (info_bits & 0x1)
        {
            cnt++;
        }
        info_bits >>= 1;
    }
    return cnt & 0x1;
}

/**
 * Utility function to swap higher 8 bit and lower 8 bit.
 */
static uint16_t swap_2bytes(uint16_t src){
    uint16_t dst = (src >> 8) & 0x00ff;
    dst |= (src << 8) & 0xff00;
    return dst;
}

/**
 * Read a register from the sensor
 */
static uint16_t read(uint16_t reg_addr)
{
    uint16_t rx_buffer = 0; 
    reg_addr |= (1 << 14);                           // PWn = 1(read)
    reg_addr |= cal_even_parity_bit(reg_addr) << 15; // Even parity bit
    reg_addr = swap_2bytes(reg_addr); // Swap 2 bytes
    spi_transaction_t transaction = {
        .flags = 0,
        .length = 16,
        .rxlength = 16,
        .tx_buffer = &reg_addr,
        .rx_buffer = &rx_buffer,
    };
    
    // Time between CS falling edge and CLK rising edge should
    // be greater than 350ns, so can't simply use harware spi CS.
    
    gpio_set_level(AS5048A_SPI_CS, 0); 
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_dev_handle, &transaction));
    gpio_set_level(AS5048A_SPI_CS, 1);
    
    return swap_2bytes(rx_buffer);
}

/**
 * Write data to a register
 */
static void write(uint16_t reg_addr, uint16_t data){
    // PWn = 0(write)
    reg_addr |= cal_even_parity_bit(reg_addr) << 15; // Even parity bit
    reg_addr = swap_2bytes(reg_addr); // Swap 2 bytes
    spi_transaction_t transaction = {
        .flags = 0,
        .length = 16,
        .rxlength = 16,
        .tx_buffer = &reg_addr,
    };

    // Time between CS falling edge and CLK rising edge should
    // be greater than 350ns, so can't simply use harware spi CS.
    gpio_set_level(AS5048A_SPI_CS, 0); 
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_dev_handle, &transaction));
    gpio_set_level(AS5048A_SPI_CS, 1);

    uint16_t old_reg_data = 0; 
    // R = 0
    data |= cal_even_parity_bit(data) << 15; // Even parity bit
    uint16_t swap_data = swap_2bytes(data);
    transaction.tx_buffer = &swap_data;
    transaction.rx_buffer = &old_reg_data;
    gpio_set_level(AS5048A_SPI_CS, 0); 
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_dev_handle, &transaction));
    gpio_set_level(AS5048A_SPI_CS, 1);
    old_reg_data = swap_2bytes(old_reg_data);

    uint16_t new_reg_data = read(AS5048A_NOP);

    if(data == new_reg_data){
        ESP_LOGI(TAG, "Write data successfully. Data: %d", data);
    }else{
        ESP_LOGE(TAG, "Fail to write data. Data: %d, new: %d", data, new_reg_data);
    }
}

/** 
 * Get angle from sensor in 14bits format
 * Range from 0 to 16383 (2^14 - 1)
 */
uint16_t AS5048A_get_position()
{
    uint16_t raw_data = read(AS5048A_ANGLE);

    uint16_t PAR = raw_data >> 15;
    uint16_t EF = (raw_data >> 14) - (PAR << 1);
    uint16_t angle = raw_data & 0x3FFF;
    
    // ESP_LOGI(TAG, "PAR: %d, EF: %d, Raw angle: %d", PAR, EF, angle);

    return angle;
}

/**
 * Clear error flag so that sensor can work normally
 */
void AS5048A_clear_error(){
    // All errors are cleared by access
    uint16_t error_data = read(AS5048A_CLEAR_ERROR_FLAG);

    uint16_t parity_error = error_data & 0x0004;
    uint16_t command_invalid = error_data & 0x0002;
    uint16_t framing_error = error_data & 0x0001;

    if(parity_error) ESP_LOGE(TAG, "Parity error.");
    else if(command_invalid) ESP_LOGE(TAG, "Command invalid.");
    else if(framing_error) ESP_LOGE(TAG, "Framing error.");
    else ESP_LOGI(TAG, "No error.");

    return;
}

/**
 * Write value to OTP register
 */
static void AS5048A_write_OTP(uint16_t pos){
    uint16_t higher_8bits = (pos >> 6) & 0x00ff;
    uint16_t lower_6bits = pos & 0x003f;

    write(AS5048A_OTP_REGISTER_ZERO_POS_HIGH, higher_8bits);
    write(AS5048A_OTP_REGISTER_ZERO_POS_LOW, lower_6bits);
}

/**
 * Set zero position in hardware level 
 */
void AS5048A_set_zero_position(){
    AS5048A_write_OTP(0); //  Write 0 into OTP zero position register to clear
    // Set zero postion
    for(int i = 0; i<2; i++){ // Wait until sensor send stable data
        AS5048A_get_position(); // Read angle information
    }
    AS5048A_write_OTP(AS5048A_get_position()); // Write previous read angle position into OTP zero position register
}

/**
 * Get diagnostics
 */
void AS5048A_get_diagnostics(uint8_t *comp_high, uint8_t *comp_low, uint8_t *COF, uint8_t *OCF, uint8_t * AGC){
    read(AS5048A_DIAG_AGC);
    uint16_t response = read(AS5048A_NOP); // Get response on the previous READ command
    
    *AGC = response & 0x00ff;
    response >>= 8;

    *OCF = response & 0x0001;
    response >>= 1;

    *COF = response & 0x0001;
    response >>= 1;

    *comp_low = response & 0x0001;
    response >>= 1;

    *comp_high = response & 0x0001;

    return;
}

/**
 * Get gain, which indicates magnetic field strength
 */
uint8_t AS5048A_get_gain(){
    uint8_t AGC;
    uint8_t comp_high;
    uint8_t comp_low;
    uint8_t useless;
    AS5048A_get_diagnostics(&comp_high, &comp_low, &useless, &useless, &AGC);

    if(comp_high){
        ESP_LOGW(TAG, "Magnetic field is too weak.");
    }
    if(comp_low){
        ESP_LOGW(TAG, "Magnetic field is too strong.");
    }

    return AGC;
}