#pragma once

#include <stdio.h>
#include "WAVFile.h"

class WAVFileWriter
{
private:
  u_int32_t m_file_size; // Size of the file in bytes

  FILE *m_fp;
  wav_header_t m_header;

public:
  /**
   * Use a double buffer system
   * Active buffer -> fill with data
   * Inactive buffer -> write to file
   * These are public to alow access from I2SMEMSSampler
   * @param buffer_active which buffer is active?
   * @param buffer_idx[2] index of next element to write to
   * @param buffer[2][16000] 2 buffers of 16000 elements each, 1 active, 1 inactive
   */
  size_t buffer_active = 0;
  bool buffer_ready_to_save = false;
  const static size_t buffer_size = 16000;
  size_t buffer_idx[2];
  signed short buffer[2][buffer_size];

  /**
   * @brief Construct a new WAVFileWriter object
   * @param fp file pointer
   * @param sample_rate
   */
  WAVFileWriter(FILE *fp, int sample_rate);

  /**
   * @brief Get the current file size
   * @return u_int32_t file size in bytes
   */
  u_int32_t get_file_size() { return m_file_size; }

  /**
   * @brief Set current file size to 0
   */
  void zero_file_size() { m_file_size = 0; }

  /**
   * @brief Get wether file ready to save
   * @return true if ready to save
   */
  bool ready_to_save() {return buffer_ready_to_save;}

  /**
   * @brief Called when a buffer is full
   * @note This will swap the buffers and set buffer_ready_to_save = true
   * 
   * @return true success
   * @return false 
   */
  bool buffer_is_full();

  /**
   * @brief Swap buffers
   *
   */
  void swap_buffers();

  /**
   * @brief Write samples to file
   */
  void write();

  /**
   * @brief Create header and write to file
   * @note This will reset the buffer_idx to 0
   * @return true success
   */
  bool finish();

  // /**
  //  * @brief Transfer contents from external buffer to internal buffer
  //  * @param external_buffer
  //  * @param external_buffer_size number of elements in external_buffer
  //  * @return true success
  //  * @return false
  //  */
  // bool pack_buffer(&external_buffer, size_t external_buffer_size);
};
