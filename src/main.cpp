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
#include "stdlib.h"
#include <cstdio>
#include "I2SMEMSSampler.h"
#include "I2SSampler.h"
#include "esp_heap_caps.h"

#include <WiFi.h>
#include <PubSubClient.h>

static void capture_samples(void *arg);
static int i2s_deinit(void);
static bool microphone_inference_start(uint32_t n_samples);
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr);
static bool microphone_inference_record(void);
static int i2s_init(uint32_t sampling_rate);

/** Audio buffers, pointers and selectors */
typedef struct
{
  int16_t *buffer;
  uint8_t buf_ready;
  uint32_t buf_count;
  uint32_t n_samples;
} inference_t;

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

// set the gain. 0 = No sound; 1 = No gain; Everything >1 equals more gain depending on the number
#define SDCARD_WRITING_ENABLED

// #define ELOC_BOARD
#ifdef SDCARD_WRITING_ENABLED
#define SDCARD_BUFFER 50 * 1024
SDCard *sd_card = nullptr;
WAVFileWriter *writer = nullptr;
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

void connectToMQTT()
{
  client.setServer(mqtt_server, 1883);
  while (!client.connected())
  {
    ei_printf("Attempting MQTT connection...");
    if (client.connect("ESP32Client"))
    {
      ei_printf("connected\n");
    }
    else
    {
      ei_printf("failed, rc=%d\n", client.state());
      delay(5000);
    }
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  // pinMode(LED_PIN, OUTPUT);

  // Initialize buzzer on GPIO 13
  // const int buzzer_pin = 13;
  // ledcSetup(0, 2000, 8); // channel 0, 2000 Hz, 8-bit resolution
  // ledcAttachPin(buzzer_pin, 0);
  // ledcWrite(0, 0); // initially off

  // comment out the below line to cancel the wait for USB connection (needed for native USB)
  while (!Serial);

  Serial.println("Edge Impulse Inferencing Demo");

  Serial.print("Available PSRAM:");

  // Returns 4192123
  Serial.println(ESP.getPsramSize());

  // summary of inferencing settings (from model_metadata.h)
  ei_printf("Inferencing settings:\n");
  ei_printf("\tInterval: ");
  ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
  ei_printf(" ms.\n");
  ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
  ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

  ei_printf("\nStarting continuous inference in 2 seconds...\n");
  ei_sleep(2000);

#ifdef SDCARD_WRITING_ENABLED
  ei_printf("Mounting SDCard on /sdcard\n");
  sd_card = new SDCard("/sdcard", PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS);
  ei_sleep(200);
  ei_printf("Mounted SDCard on /sdcard\n");
#endif

  if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false)
  {
    ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
    return;
  }

  ei_printf("Recording...\n");

  // Connect to Wi-Fi
  // connectToWiFi();

  // Connect to MQTT broker
  // connectToMQTT();
}

/**
 * @brief      Arduino main function. Runs the inferencing loop.
 */

void sendMQTTMessage(const char *topic, const char *message)
{
  if (!client.connected())
  {
    connectToMQTT();
  }
  client.publish(topic, message);
}

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

// void app_main(void) {
void loop()
{
  // setup();

  // initArduino();
  bool m = microphone_inference_record();
  if (!m)
  {
    ei_printf("ERR: Failed to record audio...\n");
    return;
  }

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
  signal.get_data = &microphone_audio_signal_get_data;
  ei_impulse_result_t result = {0};

  EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
  if (r != EI_IMPULSE_OK)
  {
    ei_printf("ERR: Failed to run classifier (%d)\n", r);
    return;
  }

  // print the predictions
  ei_printf("Predictions ");
  ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
  ei_printf(": \n");
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
  {
    ei_printf("    %s: ", result.classification[ix].label);
    ei_printf_float(result.classification[ix].value);
    ei_printf("\n");
  }
  // Check if the first classification value is higher than 0.5
  if (result.classification[1].value > 0.6)
  {
    ledState = true; // Set ledState to true to initiate the blinking sequence
  }

  // Update the LED state
  updateLedState();

#ifdef SDCARD_WRITING_ENABLED
  static int file_idx = 0;
  static int file_size = 0;
  static FILE *fp = NULL;

  // TODO: if (result.classification[1].value > 0.7 && writer == NULL)
  // Force recording
  if (writer == NULL)
  {
    char file_name[100];
    // const char* fname = "/sdcard/test.wav";
    sprintf(file_name, "/sdcard/test%d.wav", file_idx);
    ei_printf("writing audio at %s\n", file_name);
    // open the file on the sdcard
    fp = fopen(file_name, "wb");
    // create a new wave file writer
    writer = new WAVFileWriter(fp, EI_CLASSIFIER_FREQUENCY);
  }

  if (writer != NULL)
  {
    // write buffer
    // writer->write(recordBuffer, record_buffer_idx);
    // file_size += record_buffer_idx;

    if (file_size >= EI_CLASSIFIER_RAW_SAMPLE_COUNT * 10)
    {
      // and finish the writing
      writer->finish();
      fclose(fp);
      delete writer;
      file_idx++;
      fp = NULL;
      writer = NULL;
      file_size = 0;
    }
  }

  // reset recording buffer
  // record_buffer_idx = 0;
#endif

#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("    anomaly score: ");
  ei_printf_float(result.anomaly);
  ei_printf("\n");
#endif
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

  // ei_printf("audio_inference_callback()\n");

  for (int i = 0; i < n_bytes >> 1; i++)
  {
    inference.buffer[inference.buf_count++] = sampleBuffer[i];

#ifdef SDCARD_WRITING_ENABLED
    // if (record_buffer_idx < EI_CLASSIFIER_RAW_SAMPLE_COUNT * 10) {
    //     recordBuffer[record_buffer_idx++] = sampleBuffer[i];
    // } else {
    //     ei_printf("Warning: Record buffer is full, skipping sample\n");
    // }
#endif

    if (inference.buf_count >= inference.n_samples)
    {
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
 */
static void capture_samples(void *arg)
{

  ei_printf("capture_samples()\n");
  const int32_t i2s_bytes_to_read = (uint32_t)arg;

  // logical right shift divides a number by 2, throwing out any remainders
  // Need to divide by 2 because going from uint32_t to int16_t
  size_t i2s_samples_to_read = i2s_bytes_to_read >> 1;

  input->start();

  // Enter a continual loop to collect new data from I2S
  while (record_status)
  {
    int samples_read = input->read(i2s_samples_to_read);

    // Scale the data
    for (int x = 0; x < i2s_samples_to_read; x++)
    {
      sampleBuffer[x] = ((int16_t)sampleBuffer[x]) * I2S_DATA_SCALING_FACTOR;
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

  input->stop();
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
      ei_printf("Error in I2S read : %d", bytes_read);
    }
    else
    {
      if (bytes_read < i2s_bytes_to_read)
      {
        ei_printf("Partial I2S read");
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
  inference.buffer = (int16_t *)heap_caps_malloc(n_samples * sizeof(int16_t), MALLOC_CAP_SPIRAM);

  if (inference.buffer == NULL)
  {
    return false;
  }

  inference.buf_count = 0;
  inference.n_samples = n_samples; // n_samples = 1600
  inference.buf_ready = 0;

  if (i2s_init(EI_CLASSIFIER_FREQUENCY))
  {
    ei_printf("Failed to start I2S!");
  }

  ei_sleep(100);

  record_status = true;

  xTaskCreate(capture_samples, "CaptureSamples", 1024 * 16, (void *)sample_buffer_size, 10, NULL);

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
  ei_printf("microphone_inference_record()\n");

  bool ret = true;

  while (inference.buf_ready == 0)
  {
    delay(10);
  }

  inference.buf_ready = 0;
  return ret;
}

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
  numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);

  return 0;
}

/**
 * @brief      Stop PDM and release buffers
 */
static void microphone_inference_end(void)
{
  i2s_deinit();
  heap_caps_free(inference.buffer);
}

#ifdef USE_I2S_MIC_INPUT
static int i2s_init(uint32_t sampling_rate)
{
  // i2s config for reading from I2S
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

  input->register_wavfilewriter(writer);

  return ESP_OK;
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
  i2s_driver_uninstall((i2s_port_t)1); // stop & destroy i2s driver
  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif
