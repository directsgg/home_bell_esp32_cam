#include <Arduino.h>
#include <driver/i2s.h>
#include <WiFi.h>

#include "Application.h"
#include "ADCSampler.h"
#include "UdpTransport.h"
#include "OutputBuffer.h"
#include "config.h"

// #include "GenericDevBoardIndicatorLed.h"

#include "AudioTools.h"
#include "AudioLibs/Communication.h"
AudioInfo info(16000, 1, 16);
UDPStream udp(WIFI_SSID, WIFI_PSWD);
const int udpPort = 12345;
I2SStream out; // or ony other e.g AudioKitStream
StreamCopy copierOut(out, udp);

static void application_task(void *param)
{
  // delegate onto the application
  Application *application = reinterpret_cast<Application *>(param);
  application->loop();
}

Application::Application()
{
  m_output_buffer = new OutputBuffer(300 * 16);
  m_input = new ADCSampler(ADC_UNIT_1, ADC_MIC_CHANNEL, i2s_adc_config);

  m_transport = new UdpTransport(m_output_buffer);
  m_transport->set_header(TRANSPORT_HEADER_SIZE, transport_header);
}

void Application::begin()
{
  /*
  // bring up WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSWD);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  // this has a dramatic effect on packet RTT
  WiFi.setSleep(WIFI_PS_NONE);
  Serial.print("My IP Address is: ");
  Serial.println(WiFi.localIP());
  Serial.print("My MAC Address is: ");
  Serial.println(WiFi.macAddress());

  // do any setup of the transport
  m_transport->begin();

  */

  AudioLogger::instance().begin(Serial, AudioLogger::Warning);

  // start UDP receive
  udp.begin(udpPort);

  // start I2S
  auto config = out.defaultConfig(TX_MODE);
  config.copyFrom(info);
  config.buffer_size = 1024;
  config.buffer_count = 4;
  config.pin_ws=12;
  config.pin_bck=13;
  config.pin_data=15;
  out.begin(config);
  

  // setup the transmit button
  pinMode(GPIO_TRANSMIT_BUTTON, INPUT_PULLDOWN);

  // start the main task for the application
  TaskHandle_t task_handle;
  xTaskCreate(application_task, "application_task", 8192, this, 1, &task_handle);
}

// application task - coordinates everything
void Application::loop()
{
  int16_t *samples = reinterpret_cast<int16_t *>(malloc(sizeof(int16_t) * 128));
  // continue forever
  while (true)
  {
    // do we need tostart trasmitting?
    if (digitalRead(GPIO_TRANSMIT_BUTTON))
    {
      // stop the output as we're switching into transmit mode
      out.end();
      // start the input to get samples from the microphone
      m_input->start();
      Serial.println("Started transmitting");
      // transmit for at least 1 second or while the button is pushed
      unsigned long start_time = millis();
      while (millis() - start_time < 1000 || digitalRead(GPIO_TRANSMIT_BUTTON))
      {
        // read samples from the microphone
        int samples_read = m_input->read(samples, 128);
        // and send them over the transport
        for (int i = 0; i < samples_read; i++)
        {
          m_transport->add_sample(samples[i]);
        }
      }
      m_transport->flush();
      // finished transmitting stop the input and start the output
      Serial.println("Finished transmitting");
      m_input->stop();
      out.begin();
    }
    // while the transmit button is not pushed and 1 second has not elapsed
    Serial.print("Started Receiving");
    unsigned long start_time = millis();
    while (millis() - start_time < 1000 || !digitalRead(GPIO_TRANSMIT_BUTTON))
    {
      copierOut.copy();
    }
    //digitalWrite(I2S_SPEAKER_SD_PIN, LOW);
    Serial.println("Finished Receiving");
  }
}