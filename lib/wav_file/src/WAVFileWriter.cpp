#include "esp_log.h"
#include "WAVFileWriter.h"

static const char *TAG = "WAVFileWriter";

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
  
  buffer[0][0] = {0};
  buffer[1][0] = {0};
}

bool WAVFileWriter::buffer_is_full(){
  
  this->swap_buffers();
  buffer_ready_to_save = true;

  return true;
}

void WAVFileWriter::swap_buffers(){
  // TODO: Need thread mutex??
  // swap buffers
  if (++buffer_active > 1)
    buffer_active = 0;
  
  buffer_idx[buffer_active] = 0;

  ESP_LOGI(TAG, "buffer_active = %d", buffer_active);

}

void WAVFileWriter::write()
{
  auto buffer_inactive = buffer_active ? 0 : 1;

  fwrite(buffer[buffer_inactive], sizeof(int16_t), buffer_idx[buffer_inactive], m_fp);
  m_file_size += sizeof(int16_t) * buffer_idx[buffer_inactive];

  // Don't swap buffers here, wait for buffer_is_full() to do it

  buffer_ready_to_save = false;

}

bool WAVFileWriter::finish()
{
  // Have to consider the case where file has reached its  
  // max size & other buffer is being filled
  ESP_LOGI(TAG, "Finishing wav file size: %d", m_file_size);
  // now fill in the header with the correct information and write it again
  m_header.data_bytes = m_file_size - sizeof(wav_header_t);
  m_header.wav_size = m_file_size - 8;
  fseek(m_fp, 0, SEEK_SET);
  fwrite(&m_header, sizeof(wav_header_t), 1, m_fp);

  m_file_size = 0;

  this->swap_buffers();

  return true;
}