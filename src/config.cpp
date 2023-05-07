#include "config.h"

// In case each transport packet needs to start with a specific header, define transport_header here.
// TRANSPORT_HEADER_SIZE needs to be defined in config.h
// For example, when TRANSPORT_HEADER_SIZE is defined as 3,  define transport_header for example as {0x1F, 0xCD, 0x01};
uint8_t transport_header[TRANSPORT_HEADER_SIZE] = {};

// i2s config for using the internal ADC
i2s_config_t i2s_adc_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_MIC_CHANNEL,
    .communication_format = I2S_COMM_FORMAT_I2S_LSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
};