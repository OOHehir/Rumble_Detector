#pragma once

#include <stdio.h>
#include <pthread.h>
#include "WAVFile.h"


class WAVFileWriter
{
private:
  u_int32_t m_file_size; // Size of the file in bytes

  FILE *m_fp;
  wav_header_t m_header;
  
  // Indicate when a buffer is full and ready to save
  bool buffer_ready_to_save = false;

  // Indicate when a file is opened and buffer can be filled
  bool recording_in_progress = false;

  pthread_mutex_t buffer_ready_to_save_mutex;

  // Grab the bool to avoid clashes
  // Tried using  but slow to be initialised
  // As files need to be initiated quickly not suitable
  // Therefore using atomic<bool> instead
  // std::mutex buffer_ready_to_save_mutex;
  //std::atomic<bool> buffer_ready_to_save_lock;

public:
  /**
   * Use a double buffer system
   * Active buffer -> fill with data
   * Inactive buffer -> write to file (when buffer_ready_to_save == true)
   * These are public to alow efficient access
   * @param buffer_active which buffer is active?
   * @param buffer_idx index of next element to write to
   * @param buffer_size size of each buffer. Set to multiple of 512 bytes for efficient writing
   * @param buffer 2 buffers of 'x' elements each, 1 active, 1 inactive
   */

  size_t buffer_active = 0;
  
  size_t buffer_idx[2];

  // Buffer needs to be able to hold 500 -> 750 ms of data?
  // Each sample is 2 bytes
  // 500 ms @ 16kHz = 8000 samples = 16000 bytes / 512 = approx 31
  // 750 ms @ 16kHz = 12000 samples = 24000 bytes / 512 = approx 47
  static const size_t buffer_size = (512 * 31);   // NB: Using buffers of signed short (2 bytes)
  signed short buffer[2][buffer_size];

  /**
   * @brief Construct a new WAVFileWriter object
   * @param fp file pointer
   * @param sample_rate
   */
  WAVFileWriter(FILE *fp, int sample_rate);

  /**
   * @brief Getter for buffer_ready_to_save
   * 
   * @return true ready to save
   * @return false 
   */
  bool ready_to_save() {return buffer_ready_to_save;} 

  /**
   * @brief Getter for current file size
   * 
   * @return auto 
   */
  u_int32_t get_file_size() {return m_file_size;}

  /**
   * @brief Swap buffers
   * @note Only call from I2SMEMSSampler::read() ???
   */
  void swap_buffers();

  /**
   * @brief Set the buffer ready to save object
   * @note Uses mutex to avoid clashes
   * @note returns immediately if mutex locked with false
   * @param value 
   * @return true value set successfully
   * @return false value not set (mutex locked)
   */
  bool set_buffer_ready_to_save(bool value);

  /**
   * @brief Call when buffer is full
   * @note Will set buffer_ready_to_save to true & calls swap buffers()
   * @return true acquired mutex
   * @return false failed to acquire mutex
   */
  bool buffer_is_full();

  /**
   * @brief Will write the inactive buffer to file
   * @note Will set buffer_idx to 0 of the inactive buffer
   * @note This will set buffer_ready_to_save = false;
   */
  void write();

  /**
   * @brief Create header and write to file
   * @note This will reset all buffer_idx to 0
   * @note This will set buffer_active to 0
   * @note This will set buffer_ready_to_save to false
   * @return true success
   */
  bool finish();

};
