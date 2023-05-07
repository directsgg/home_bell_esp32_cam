#include "ADCSampler.h"

ADCSampler::ADCSampler(adc_unit_t adcUnit, adc1_channel_t adcChannel, const i2s_config_t &i2s_config) : I2SSampler(I2S_NUM_0, i2s_config)
{
  m_adcUnit = adcUnit;
  m_adcChannel = adcChannel;
  isConfigured = false;
}

void ADCSampler::configureI2S()
{

  // init ADC pad
  i2s_set_adc_mode(m_adcUnit, m_adcChannel);

  if (!isConfigured)
  {
    // enable the adc
    i2s_adc_enable(m_i2sPort);
    isConfigured = true;
  }
}

int ADCSampler::read(int16_t *samples, int count)
{
  // read from i2s
  size_t bytes_read = 0;
  i2s_read(m_i2sPort, samples, sizeof(int16_t) * count, &bytes_read, portMAX_DELAY);
  int sampes_read = bytes_read / sizeof(int16_t);
  for (int i = 0; i < sampes_read; i++)
  {
    //samples[i] = (2048 - (uint16_t(samples[i]) & 0xfff)) * 15;
    samples[i] = (3548 - (uint16_t(samples[i]) & 0xfff)) * 15;
  }
  return sampes_read;
}