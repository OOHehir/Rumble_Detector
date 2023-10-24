/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 ** NOTE: If you run into TFLite arena allocation issue.
 **
 ** This may be due to may dynamic memory fragmentation.
 ** Try defining "-DEI_CLASSIFIER_ALLOCATION_STATIC" in boards.local.txt (create
 ** if it doesn't exist) and copy this file to
 ** `<ARDUINO_CORE_INSTALL_PATH>/arduino/hardware/<mbed_core>/<core_version>/`.
 **
 ** See
 ** (https://support.arduino.cc/hc/en-us/articles/360012076960-Where-are-the-installed-cores-located-)
 ** to find where Arduino installs cores on your machine.
 **
 ** If the problem persists then there's not enough memory for this model and application.
 */

// If your target is limited in memory remove this macro to save 10K RAM
#define EIDSP_QUANTIZE_FILTERBANK 0

/* Includes ---------------------------------------------------------------- */
#include <Arduino.h>
#include <trumpet_inferencing.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2s.h"
#include "WAVFileWriter.h"
#include "SDCard.h"
#include <stdlib.h>
#include <cstdio>
#include "I2SMEMSSampler.h"
#include "I2SSampler.h"
#include "esp_heap_caps.h"

#include <WiFi.h>
#include <PubSubClient.h>
#include "ei_inference.h"
#include "config_build.h"

static void capture_samples(void *arg);
static int i2s_deinit(void);
static bool microphone_inference_start(uint32_t n_samples);
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr);
static bool microphone_inference_record(void);
static int i2s_init(uint32_t sampling_rate);

static inference_t inference;
static const uint32_t sample_buffer_size = 2048;
static signed short sampleBuffer[sample_buffer_size];
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool record_status = true;

// are you using an I2S microphone - comment this out if you want to use an analog mic and ADC input
#define USE_I2S_MIC_INPUT

#ifdef USE_I2S_MIC_INPUT
I2SMEMSSampler *input = nullptr;
#endif

#ifdef SDCARD_WRITING_ENABLED
  #define SDCARD_BUFFER 50 * 1024
  SDCard *sd_card = nullptr;
  WAVFileWriter *writer = nullptr;
  #define RECORDING_TIME 30
#endif

// sdcard (unused, as SDIO is fixed to its Pins)

#ifdef ELOC_BOARD
// For ELOC
#define PIN_NUM_MISO GPIO_NUM_2
#define PIN_NUM_CLK GPIO_NUM_14
#define PIN_NUM_MOSI GPIO_NUM_15
#define PIN_NUM_CS GPIO_NUM_14
#else
// For WROVER Board
#define PIN_NUM_MISO GPIO_NUM_4
#define PIN_NUM_CLK GPIO_NUM_27  // Conflicts with MTMS for JTAG
#define PIN_NUM_MOSI GPIO_NUM_18 // Conflicts with MTDO for JTAG
#define PIN_NUM_CS GPIO_NUM_25
#endif

// External Blink LED
#define LED_PIN 21

#define I2S_DATA_SCALING_FACTOR 1

// Replace with your network credentials
const char *ssid = "GL-MT300N-V2-eb1";
const char *password = "goodlife";

// Replace with your MQTT broker address
const char *mqtt_server = "192.168.8.115";
const uint16_t mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// Variables to store the LED state and the time it was last turned on
bool ledState = false;
unsigned long ledTurnedOnAt = 0;
unsigned long ledDuration = 1;

static const char *TAG = "main";

/**
 * @brief      Arduino setup function
 */

extern "C"
{
  void app_main(void);
}

void connectToWiFi()
{
  WiFi.begin(ssid, password);
  ei_printf("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    ei_printf(".");
  }
  ei_printf("\nConnected to Wi-Fi\n");
}

// void connectToMQTT()
// {
//   client.setServer(mqtt_server, 1883);
//   while (!client.connected())
//   {
//     ei_printf("Attempting MQTT connection...");
//     if (client.connect("ESP32Client"))
//     {
//       ei_printf("connected\n");
//     }
//     else
//     {
//       ei_printf("failed, rc=%d\n", client.state());
//       delay(5000);
//     }
//   }
// }

void setup()
{
  // put your setup code here, to run once:
  // Serial.begin(115200);
  // pinMode(LED_PIN, OUTPUT);

  // Initialize buzzer on GPIO 13
  // const int buzzer_pin = 13;
  // ledcSetup(0, 2000, 8); // channel 0, 2000 Hz, 8-bit resolution
  // ledcAttachPin(buzzer_pin, 0);
  // ledcWrite(0, 0); // initially off

  // comment out the below line to cancel the wait for USB connection (needed for native USB)
  // while (!Serial);
  // Serial.println("Edge Impulse Inferencing Demo");

  // Returns 4192123
  ESP_LOGI(TAG, "Available PSRAM: %d", ESP.getFreePsram());

  // summary of inferencing settings (from model_metadata.h)
  ESP_LOGI(TAG, "Inferencing settings:\n");
  ESP_LOGI(TAG, "\tInterval: ");
  ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
  ESP_LOGI(TAG, " ms.");
  ESP_LOGI(TAG, "\tFrame size: %d", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  ESP_LOGI(TAG, "\tSample length: %d ms.", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
  ESP_LOGI(TAG, "\tNo. of classes: %d", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

#ifdef SDCARD_WRITING_ENABLED
  sd_card = new SDCard("/sdcard", PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS);
  ei_sleep(200);

  if (sd_card == nullptr){
    ESP_LOGI(TAG, "Failed to mount SDCard");
  }
    
#endif

  if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false)
  {
    ESP_LOGE(TAG, "ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
    return;
  }

  ESP_LOGI(TAG, "Recording...");

  // Connect to Wi-Fi
  // connectToWiFi();

  // Connect to MQTT broker
  // connectToMQTT();
}

/**
 * @brief      Arduino main function. Runs the inferencing loop.
 */

// void sendMQTTMessage(const char *topic, const char *message)
// {
//   if (!client.connected())
//   {
//     connectToMQTT();
//   }
//   client.publish(topic, message);
// }

// Buzzer beeps
void beep(int n_times)
{
  for (int i = 0; i < n_times; i++)
  {
    ledcWrite(0, 128); // 50% duty cycle
    delay(100);        // 100 ms beep duration
    ledcWrite(0, 0);   // turn off buzzer
    delay(100);        // 100 ms pause between beeps
  }
}

// Function to check if the LED should be turned off
void updateLedState()
{
  if (ledState)
  {
    // blink the LED three times quickly
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
    // TODO:  sendMQTTMessage("sensor/elephant", "TRUMPET DETECTED");
    ledState = false; // Update ledState to indicate that the blinking is complete
  }
}

void app_main(void)
{

  initArduino();
  ESP_LOGI(TAG, "initArduino done");
  setup();

#ifdef SDCARD_WRITING_ENABLED
  static int file_idx = 0;
  static char file_name[100]; // needs to persist outside this scope??
  static FILE *fp = NULL;
#endif

  while (1)
  {

#ifdef SDCARD_WRITING_ENABLED

    if (fp == NULL)
    {
      sprintf(file_name, "/sdcard/eloc/test%d.wav", file_idx);
      ESP_LOGI(TAG, "Saving audio to %s", file_name);

      // open the file on the sdcard
      fp = fopen(file_name, "wb");

      if (fp == NULL)
      {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
      }
      else
      {
        // create a new wave file writer
        writer = new WAVFileWriter(fp, EI_CLASSIFIER_FREQUENCY);
      }

      if (writer == nullptr)
      {
        ESP_LOGE(TAG, "Failed to create WAVFileWriter");
      }
      else
      {
        // Block until properly registered
        // Otherwise will get error later
        while (input->register_wavFileWriter(writer) == false){
          ESP_LOGI(TAG, "Waiting for WAVFileWriter to register");
          ei_sleep(100);
        };
      }
    }

#endif // SDCARD_WRITING_ENABLED

    bool m = microphone_inference_record();
    if (!m)
    {
      ESP_LOGE(TAG, "ERR: Failed to record audio...");
      return;
    }

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = {0};

    // If changing to non-continuous ensure to use:
    // EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
    EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK)
    {
      ESP_LOGE(TAG, "ERR: Failed to run classifier (%d)", r);
      return;
    }

    // print the predictions
    ESP_LOGI(TAG, "Predictions ");
    ESP_LOGI(TAG, "(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
             result.timing.dsp, result.timing.classification, result.timing.anomaly);
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
    {
      ESP_LOGI(TAG, "    %s: %f", result.classification[ix].label, result.classification[ix].value);
    }
    // Check if the first classification value is higher than 0.5
    // if (result.classification[1].value > 0.6)
    // {
    //   ledState = true; // Set ledState to true to initiate the blinking sequence
    // }

    // // Update the LED state
    // updateLedState();

#ifdef SDCARD_WRITING_ENABLED

    if (writer != nullptr && writer->ready_to_save() == true)
    {

      writer->write();
    }

    //   if (writer->get_file_size() >= EI_CLASSIFIER_RAW_SAMPLE_COUNT * RECORDING_TIME)
    //   {
    //     // TODO: Figure out how to save active buffer portion
    //     // and finish the writing
    //     ESP_LOGI(TAG, "Finishing SD writing\n");
    //     writer->finish();
    //     fclose(fp);
    //     delete writer;
    //     file_idx++;
    //     fp = NULL;
    //     writer = nullptr;
    //   }
    // }

    if (writer != nullptr && writer->get_file_size() >= EI_CLASSIFIER_RAW_SAMPLE_COUNT * RECORDING_TIME)
      {
        // TODO: Figure out how to save active buffer portion
        // and finish the writing
        ESP_LOGI(TAG, "Finishing SD writing\n");
        writer->finish();
        fclose(fp);
        delete writer;
        file_idx++;
        fp = NULL;
        writer = nullptr;
      }

    // reset recording buffer
    // record_buffer_idx = 0;
#else

  // Feed the dog..
  delay(1);

#endif


#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ESP_LOGI(TAG, "    anomaly score: ");
    ESP_LOGI(TAG, result.anomaly);
#endif

  }
}

/**
 * This function is repeatedly called by capture_samples()
 * When sufficient samples are collected:
 *  1. inference.buf_ready = 1
 *  2. microphone_inference_record() is unblocked
 *  3. classifier is run in main loop()
 */
static void audio_inference_callback(uint32_t n_bytes)
{
  for (int i = 0; i < n_bytes >> 1; i++)
  {
    inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];

    if (inference.buf_count >= inference.n_samples)
    {
      inference.buf_select ^= 1;
      inference.buf_count = 0;
      inference.buf_ready = 1;
    }
  }
}

#ifdef USE_I2S_MIC_INPUT
/**
 * This is initiated by a task created in microphone_inference_record_start()
 * When periodically called it:
 *  1. reads data from I2S DMA buffer,
 *  2. scales it
 *  3. Calls audio_inference_callback()
 * @note: Explanation: https://forum.edgeimpulse.com/t/trouble-understanding-microphone-continuous/7416
 */
static void capture_samples(void *arg)
{

  ESP_LOGI(TAG, "%s", __func__);

  const int32_t i2s_bytes_to_read = (uint32_t)arg;

  // logical right shift divides a number by 2, throwing out any remainders
  // Need to divide by 2 because reading bytes into a int16_t buffer
  size_t i2s_samples_to_read = i2s_bytes_to_read >> 1;

  // Enter a continual loop to collect new data from I2S
  while (record_status)
  {
    int samples_read = input->read(i2s_samples_to_read);

    if (samples_read != i2s_samples_to_read){
      ESP_LOGW(TAG, "samples_read = %d, i2s_samples_to_read = %d", samples_read, i2s_samples_to_read);
    }

    // Buffers loaded in I2MEMSampler

    // Scale the data
    // for (int x = 0; x < i2s_samples_to_read; x++)
    // {
    //   sampleBuffer[x] = ((int16_t)sampleBuffer[x]) * I2S_DATA_SCALING_FACTOR;
    // }

    // if (record_status)
    // {
    //   audio_inference_callback(i2s_bytes_to_read);
    // }
    // else
    // {
    //   break;
    // }
  }

  //input->stop();
  vTaskDelete(NULL);
}
#else
// Not used
static void capture_samples(void *arg)
{
  const int32_t i2s_bytes_to_read = (uint32_t)arg;
  size_t bytes_read = i2s_bytes_to_read;

  while (record_status)
  {

    /* read data at once from i2s */
    i2s_read((i2s_port_t)1, (void *)sampleBuffer, i2s_bytes_to_read, &bytes_read, 100);

    if (bytes_read <= 0)
    {
      ESP_LOGI(TAG, "Error in I2S read : %d", bytes_read);
    }
    else
    {
      if (bytes_read < i2s_bytes_to_read)
      {
        ESP_LOGI(TAG, "Partial I2S read");
      }

      // scale the data (otherwise the sound is too quiet)
      for (int x = 0; x < i2s_bytes_to_read / 2; x++)
      {
        sampleBuffer[x] = (int16_t)(sampleBuffer[x]) * 11;
      }

      if (record_status)
      {
        audio_inference_callback(i2s_bytes_to_read);
      }
      else
      {
        break;
      }
    }
  }
  vTaskDelete(NULL);
}
#endif

/**
 * @brief      Init inferencing struct and setup/start PDM
 *
 * @param[in]  n_samples  The n samples
 *
 * @return     { description_of_the_return_value }
 */
static bool microphone_inference_start(uint32_t n_samples)
{
  inference.buffers[0] = (int16_t *)heap_caps_malloc(n_samples * sizeof(int16_t), MALLOC_CAP_SPIRAM);

  if (inference.buffers[0] == NULL)
  {
    ESP_LOGE(TAG, "Failed to allocate %d bytes for inference buffer", n_samples * sizeof(int16_t));
    return false;
  }

  inference.buffers[1] = (int16_t *)heap_caps_malloc(n_samples * sizeof(int16_t), MALLOC_CAP_SPIRAM);

  if (inference.buffers[1] == NULL)
  {
    ei_free(inference.buffers[0]);
    return false;
  }

  inference.buf_select = 0;
  inference.buf_count = 0;
  inference.n_samples = n_samples;
  inference.buf_ready = 0;

  if (i2s_init(EI_CLASSIFIER_FREQUENCY))
  {
    ESP_LOGI(TAG, "Failed to start I2S!");
  }

  ei_sleep(100);

  record_status = true;

  if (input == nullptr)
  {
    ESP_LOGE(TAG, "%s - MEMSampler == nullptr", __func__);
  }
  else
  {
    input->register_ei_inference(&inference, EI_CLASSIFIER_FREQUENCY);
  }

  // Stack size of 16K - experimentally determined
  // (void*)sample_buffer_size - pass in the number of samples to read
  // Should match 
  xTaskCreate(capture_samples, "CaptureSamples", 1024 * 16, (void*)sample_buffer_size, 10, NULL);

  return true;
}

/**
 * @brief  Wait on new data.
 *         Blocking function.
 *         Unblocked by audio_inference_callback() setting inference.buf_ready
 *
 * @return     True when finished
 */
static bool microphone_inference_record(void)
{
  ESP_LOGV(TAG, "Func: %s", __func__);

  bool ret = true;

  // TODO: Expect this to be set as loading from another point?
  // if (inference.buf_ready == 1)
  // {
  //   ESP_LOGE(TAG, "Error sample buffer overrun. Decrease the number of slices per model window "
  //       "(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)");
  //   ret = false;
  // }

  while (inference.buf_ready == 0)
  {
    // NOTE: Trying to write audio out here seems to leads to poor audio performance?
    // if(writer->buf_ready == 1){
    //   writer->write();
    // }
    // else
    // {
      delay(1);
    //}
  }

  inference.buf_ready = 0;
  
  return true;
}

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
  numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);

  return 0;
}

/**
 * @brief      Stop PDM and release buffers
 */
static void microphone_inference_end(void)
{
    i2s_deinit();
    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);
}


#ifdef USE_I2S_MIC_INPUT

static int i2s_init(uint32_t sampling_rate)
{
  // Start listening for audio: MONO, 32Bit
  i2s_config_t i2s_mic_Config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = sampling_rate,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,
      .dma_buf_count = 4,
      .dma_buf_len = 1024,
      .use_apll = true,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0};

  // i2s microphone pins
  // i2s_pin_config_t i2s_mic_pins = {
  //     .mck_io_num = I2S_PIN_NO_CHANGE,
  //     .bck_io_num = 18,
  //     .ws_io_num = 5,
  //     .data_out_num = I2S_PIN_NO_CHANGE,
  //     .data_in_num = 19
  // };

  i2s_pin_config_t i2s_mic_pins = {

#ifdef ELOC_BOARD
      .bck_io_num = 18,   // IIS_SCLK
      .ws_io_num = 5,     // IIS_LCLK   was 32
      .data_out_num = -1, // IIS_DSIN
      .data_in_num = 19,  // IIS_DOUT   was 33
#else
      // WROVER
      .bck_io_num = 26,   // IIS_SCLK
      .ws_io_num = 22,    // IIS_LCLK   was 32
      .data_out_num = -1, // IIS_DSIN
      .data_in_num = 21,  // IIS_DOUT   was 33
#endif
  };

  input = new I2SMEMSSampler(I2S_NUM_0, i2s_mic_pins, i2s_mic_Config);
  input->start(); // Actually installs the driver
  input->zero_dma_buffer(I2S_NUM_0);
  return 0;
}
#else
static int i2s_init(uint32_t sampling_rate)
{
  // Start listening for audio: MONO @ 8/16KHz
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
      .sample_rate = sampling_rate,
      .bits_per_sample = (i2s_bits_per_sample_t)16,
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = 512,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = -1,
  };
  i2s_pin_config_t pin_config = {
      .bck_io_num = 18,   // IIS_SCLK
      .ws_io_num = 5,     // IIS_LCLK
      .data_out_num = -1, // IIS_DSIN
      .data_in_num = 19,  // IIS_DOUT
  };
  esp_err_t ret = 0;

  ret = i2s_driver_install((i2s_port_t)1, &i2s_config, 0, NULL);
  if (ret != ESP_OK)
  {
    ei_printf("Error in i2s_driver_install");
  }

  ret = i2s_set_pin((i2s_port_t)1, &pin_config);
  if (ret != ESP_OK)
  {
    ei_printf("Error in i2s_set_pin");
  }

  ret = i2s_zero_dma_buffer((i2s_port_t)1);
  if (ret != ESP_OK)
  {
    ei_printf("Error in initializing dma buffer with 0");
  }

  return int(ret);
}
#endif
static int i2s_deinit(void)
{
  i2s_driver_uninstall((i2s_port_t)I2S_NUM_0); // stop & destroy i2s driver
  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif
