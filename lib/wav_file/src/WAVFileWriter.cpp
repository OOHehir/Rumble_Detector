#include "esp_log.h"
#include "WAVFileWriter.h"

static const char *TAG = "WAVFileWriter";
static pthread_mutex_t buffer_ready_to_save_mutex;

WAVFileWriter::WAVFileWriter(FILE *fp, int sample_rate)
{
  m_fp = fp;
  m_header.setSample_rate(sample_rate);
  // write out the header - we'll fill in some of the blanks later
  fwrite(&m_header, sizeof(wav_header_t), 1, m_fp);
  m_file_size = sizeof(wav_header_t);

  buffer_active = 0;
  buffer_idx[0] = 0;
  buffer_idx[1] = 0;
  
  // Avoid warnings about not initialising buffer
  buffer[0][0] = {0};
  buffer[1][0] = {0};

  if(pthread_mutex_init (&buffer_ready_to_save_mutex, NULL) != 0)
    ESP_LOGE(TAG, "Failed to initialise buffer_ready_to_save_mutex");

  //buffer_ready_to_save_lock = false;
}

void WAVFileWriter::swap_buffers(){

  buffer_active = buffer_active ? 0 : 1;
}

// bool WAVFileWriter::set_buffer_ready_to_save(bool value){

//   if (buffer_ready_to_save_lock == true)
//     return false;
//   else
//     buffer_ready_to_save_lock = true;

//   buffer_ready_to_save = value;
  
//   buffer_ready_to_save_lock = false;
  
//   return true;
// }

bool WAVFileWriter::set_buffer_ready_to_save(bool value){

  if(pthread_mutex_trylock(&buffer_ready_to_save_mutex) != 0){
    ESP_LOGE(TAG, "Failed to lock buffer_ready_to_save_mutex");
    return false;
  }
  
  buffer_ready_to_save = value;
  
  pthread_mutex_unlock(&buffer_ready_to_save_mutex);
  
  return true;
}

bool WAVFileWriter::buffer_is_full(){

  // Keep trying to set buffer_ready_to_save until successful
  while (set_buffer_ready_to_save(true) == false);
  
  this->swap_buffers();

  // ESP_LOGI(TAG, "buffer_is_full(), buffer_active: %d", buffer_active);

  return true;
}

void WAVFileWriter::write()
{
  // Need to consider the case where the buffer is not full, i.e end of recording
  // Therefore can't use buffer_size to determine how much to write

  auto buffer_inactive = buffer_active ? 0 : 1;

  ESP_LOGI(TAG, "Starting write");

  auto bytes = fwrite(buffer[buffer_inactive], sizeof(int16_t), buffer_idx[buffer_inactive], m_fp);
  m_file_size += sizeof(int16_t) * buffer_idx[buffer_inactive];

  buffer_idx[buffer_inactive] = 0;
  
  // Keep trying to set buffer_ready_to_save to false until successful
  ESP_LOGI(TAG, "Finishing write, bytes written: %d", bytes);

  while(set_buffer_ready_to_save(false) == false);

}

bool WAVFileWriter::finish()
{
  // Need to consider the case where recording is continuing & other buffer is being filled

  ESP_LOGI(TAG, "Finishing wav file size: %d", m_file_size);
  // now fill in the header with the correct information and write it again
  m_header.data_bytes = m_file_size - sizeof(wav_header_t);
  m_header.wav_size = m_file_size - 8;
  fseek(m_fp, 0, SEEK_SET);
  fwrite(&m_header, sizeof(wav_header_t), 1, m_fp);

  while(set_buffer_ready_to_save(false) == false);

  return true;
}