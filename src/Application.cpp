#include <Arduino.h>
#include <driver/i2s.h>
#include <WiFi.h>
// audio
#include "Application.h"
#include "ADCSampler.h"
#include "UdpTransport.h"
#include "OutputBuffer.h"
#include "config.h"
#include "AudioTools.h"
#include "AudioLibs/Communication.h"

// camera
#include <esp_system.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include "driver/gpio.h"

#include "esp_camera.h"
#include "esp_https_server.h"
#include "esp_timer.h"
#include "camera_pins.h"
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

// audio
UDPStream udp(WIFI_SSID, WIFI_PSWD);
const int udpPort = 12345;
IPAddress udpAddress(255, 255, 255, 255);

AudioInfo audioInfo(16000, 1, 16);
I2SStream outI2SAudio;
StreamCopy copierOutAudio(outI2SAudio, udp);

// camera
static httpd_handle_t SERVER_LIVE_STREAM = NULL;

static esp_err_t init_camera(void)
{
  camera_config_t configCamera;
  configCamera.ledc_channel = LEDC_CHANNEL_0;
  configCamera.ledc_timer = LEDC_TIMER_0;
  configCamera.pin_d0 = Y2_GPIO_NUM;
  configCamera.pin_d1 = Y3_GPIO_NUM;
  configCamera.pin_d2 = Y4_GPIO_NUM;
  configCamera.pin_d3 = Y5_GPIO_NUM;
  configCamera.pin_d4 = Y6_GPIO_NUM;
  configCamera.pin_d5 = Y7_GPIO_NUM;
  configCamera.pin_d6 = Y8_GPIO_NUM;
  configCamera.pin_d7 = Y9_GPIO_NUM;
  configCamera.pin_xclk = XCLK_GPIO_NUM;
  configCamera.pin_pclk = PCLK_GPIO_NUM;
  configCamera.pin_vsync = VSYNC_GPIO_NUM;
  configCamera.pin_href = HREF_GPIO_NUM;
  configCamera.pin_sscb_sda = SIOD_GPIO_NUM;
  configCamera.pin_sscb_scl = SIOC_GPIO_NUM;
  configCamera.pin_pwdn = PWDN_GPIO_NUM;
  configCamera.pin_reset = RESET_GPIO_NUM;
  configCamera.xclk_freq_hz = 20000000;
  configCamera.pixel_format = PIXFORMAT_JPEG;
  // init with high specs to pre-allocate larger buffers
  //if (psramFound())
  //{
  //  configCamera.frame_size = FRAMESIZE_UXGA;
  //  configCamera.jpeg_quality = 10;
  //  configCamera.fb_count = 2;
  //  configCamera.grab_mode = CAMERA_GRAB_WHEN_EMPTY; // CAMERA_GRAB_LATEST. Sets when buffers should be filled
  //}
  //else
  //{
    configCamera.frame_size = FRAMESIZE_SVGA;
    configCamera.jpeg_quality = 12;
    configCamera.fb_count = 1;
  //}

  // camera init
  esp_err_t err = esp_camera_init(&configCamera);
  if (err != ESP_OK)
  {
    Serial.print("Camera init failed with error: ");
    Serial.println(err);
    while(true) {};
    return err;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID)
  {
    s->set_vflip(s, 1);       // flip it back
    s->set_brightness(s, 1);  // up the blightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

  Serial.println("Camera init");
  return ESP_OK;
}

esp_err_t jpg_stream_httpd_handler(httpd_req_t *req)
{
  camera_fb_t *fb = NULL;
  struct timeval _timestamp;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len;
  uint8_t *_jpg_buf;
  char *part_buf[128];
  static int64_t last_frame = 0;
  if (!last_frame)
  {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK)
  {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "60");

  while (true)
  {
    fb = esp_camera_fb_get();
    if (!fb)
    {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
      break;
    }
    else
    {
      _timestamp.tv_sec = fb->timestamp.tv_sec;
      _timestamp.tv_usec = fb->timestamp.tv_usec;
    }
    if (fb->format != PIXFORMAT_JPEG)
    {
      bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
      esp_camera_fb_return(fb);
      fb = NULL;
      if (!jpeg_converted)
      {
        Serial.println("JPEG compression failed");
        res = ESP_FAIL;
      }
    }
    else
    {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }

    if (res == ESP_OK)
    {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK)
    {
      size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK)
    {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (fb)
    {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    }
    else if (_jpg_buf)
    {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK)
    {
      Serial.println("Send frame failed");
      break;
    }

    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;
  }
  last_frame = 0;

  return res;
}

httpd_handle_t start_live_stream_server(void)
{
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  Serial.print("server ");
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 16;

  httpd_uri_t stream_uri = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = jpg_stream_httpd_handler,
      .user_ctx = NULL};

  httpd_handle_t stream_httpd = NULL;

  if (httpd_start(&stream_httpd, &config) == ESP_OK)
  {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("start");
  }
  return stream_httpd;
}

static void stop_live_stream_server(httpd_handle_t server)
{
  // stop the httpd server
  httpd_stop(server);
  Serial.println("server stop");
}

static void startModoCamara() {
  SERVER_LIVE_STREAM = start_live_stream_server();
}

static void stopModoCamara() {
  stop_live_stream_server(SERVER_LIVE_STREAM);
  SERVER_LIVE_STREAM = NULL;
}

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
  // m_transport->set_header(TRANSPORT_HEADER_SIZE, transport_header);
}

void Application::begin()
{
  // camara
  //  initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  //init_camera();


  // audio
  //AudioLogger::instance().begin(Serial, AudioLogger::Warning);

  // start UDP receive
  udp.begin(udpAddress, udpPort);
  // start I2S
  auto outI2SConfig = outI2SAudio.defaultConfig(TX_MODE);
  outI2SConfig.copyFrom(audioInfo);
  outI2SConfig.port_no = I2S_NUM_1;
  outI2SConfig.buffer_size = 1024;
  outI2SConfig.buffer_count = 4;
  outI2SConfig.pin_ws = 13;
  outI2SConfig.pin_bck = 15;
  outI2SConfig.pin_data = 14;
  outI2SAudio.begin(outI2SConfig);

  // camara
  startModoCamara();

  //esp_camera_deinit();
  // setup the transmit button
  pinMode(GPIO_TRANSMIT_BUTTON, INPUT_PULLDOWN);
  // setup the config button
  pinMode(GPIO_CONTROL_BUTTON, INPUT_PULLDOWN);
  // start the main task for the application
  TaskHandle_t task_handle;
  xTaskCreate(application_task, "application_task", 8192, this, 1, &task_handle);
}

// application task - coordinates everything
void Application::loop()
{
  int16_t *samples = reinterpret_cast<int16_t *>(malloc(sizeof(int16_t) * 128));
  // audio buffer for samples we need to send
  int m_buffer_size = 1436;
  int m_index = 0;
  int m_header_size;
  uint8_t *m_buffer = (uint8_t *)malloc(m_buffer_size);
  // continue forever
  /*if(!digitalRead(GPIO_CONTROL_BUTTON)) {
    outI2SAudio.end();
  }*/
  while (true)
  {
    // ¿necesitamos comenzar a transmitir audio o video?
    if (digitalRead(GPIO_CONTROL_BUTTON))
    {
      // detener la cámara ya que estamos cambiando al modo de audio
      //stopModoCamara();
      // iniciar el modo audio 
      Serial.println("Iniciar modo audio");
      //outI2SAudio.begin();
      // modo audio durante al menos 1 segundo o mientras se presiona el botón de control
      unsigned long start_time_control_mode = millis();
      while (millis() - start_time_control_mode < 1000 || digitalRead(GPIO_CONTROL_BUTTON))
      {
        // todo modo audio
        // do we need to start transmitting or receiving?
        if (digitalRead(GPIO_TRANSMIT_BUTTON))
        {
          // stop the output as we're switching into transmit mode
          outI2SAudio.end();
          // start the input to get samples from the microphone
          m_input->start();
          Serial.print("Started transmitting");
          // transmit for at least 1 second or while the button is pushed
          unsigned long start_time = millis();
          while (millis() - start_time < 1000 || digitalRead(GPIO_TRANSMIT_BUTTON))
          {
            // read samples from the microphone
            int samples_read = m_input->read(samples, 128);
            // and send them over the transport
            for (int i = 0; i < samples_read; i++)
            {
              // m_transport->add_sample(samples[i]);
              m_buffer[m_index + m_header_size] = (samples[i] + 32768) >> 8;
              m_index++;
              // have we reached a full packet?
              if ((m_index + m_header_size) == m_buffer_size)
              {
                // send
                udp.write(m_buffer, m_index);
                m_index = 0;
              }
            }
          }
          // m_transport->flush();
          if (m_index > 0)
          {
            // send
            udp.write(m_buffer, m_index);
            m_index = 0;
          }
          // finished transmitting stop the input and start the output
          Serial.println("Finished transmitting");
          m_input->stop();
          outI2SAudio.begin();
        }
        // while the transmit button is not pushed and 1 second has not elapsed
        Serial.print("Started Receiving");
        unsigned long start_time = millis();
        while (millis() - start_time < 1000 || !digitalRead(GPIO_TRANSMIT_BUTTON))
        {
          copierOutAudio.copy();
        }
        // digitalWrite(I2S_SPEAKER_SD_PIN, LOW);
        Serial.println("Finished Receiving");
      }
      // terminado modo, detener modo audio y comenzar modo camara
      //outI2SAudio.end();
      Serial.println("Finalizar modo audio");
    }
    // mientras no se presione el botón de configuracion y no haya transcurrido 1 segundo
    Serial.println("Iniciar modo camara");
    unsigned long start_time_control_mode = millis();
    while (millis() - start_time_control_mode < 1000 || !digitalRead(GPIO_CONTROL_BUTTON))
    {
      // todo modo camara
    }
    Serial.println("Finalizar modo camara");
  }
}