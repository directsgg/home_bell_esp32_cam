#include <freertos/FreeRTOS.h>
#include <driver/i2s.h>
#include <driver/gpio.h>

// WiFi credentials
#define WIFI_SSID "ssid"
#define WIFI_PSWD "***"

//sample rate for the system
#define SAMPLE_RATE 32000

// I2S Microphone Settings

// Which channel is the I2S microphone on? I2S_CHANNEL_FMT_ONLY_LEFT or I2S_CHANNEL_FMT_ONLY_RIGHT
// Generally they will default to LEFT - but you may need to attach the L/R pin to GND
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT


// Analog Microphone Settings - ADC1_CHANNEL_5 is GPIO33
#define ADC_MIC_CHANNEL ADC1_CHANNEL_5

// transmit button
#define GPIO_TRANSMIT_BUTTON 2

// In case all transport packets need a header (to avoid interference with other applications or walkie talkie sets), 
// specify TRANSPORT_HEADER_SIZE (the length in bytes of the header) in the next line, and define the transport header in config.cpp
#define TRANSPORT_HEADER_SIZE 0
extern uint8_t transport_header[TRANSPORT_HEADER_SIZE];

// i2s config for using the internal ADC
extern i2s_config_t i2s_adc_config;
