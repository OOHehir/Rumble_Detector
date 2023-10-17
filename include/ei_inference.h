/**
 * @file ei_inference.h
 * @brief Inference structure
 * 
 * @date 2021-05-05
 * 
*/

#ifndef _EI_INFERENCE_H_
#define _EI_INFERENCE_H_

#include <stdint.h>

/** Audio buffers, pointers and selectors */
typedef struct
{
  int16_t *buffer;
  uint8_t buf_ready;
  uint32_t buf_count;
  uint32_t n_samples;
} inference_t;


#endif // _EI_INFERENCE_H_