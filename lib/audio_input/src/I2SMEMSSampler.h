#pragma once

#include "I2SSampler.h"
#include "WAVFileWriter.h"
#include "/home/projects/Rumble_Detector/include/ei_inference.h"
class I2SMEMSSampler : public I2SSampler
{
private:
    i2s_pin_config_t m_i2sPins;
    bool m_fixSPH0645;
    WAVFileWriter *writer = nullptr;

    // Set some reasonable values as default
    uint32_t i2s_sampling_rate = 16000;
    uint32_t ei_sampling_freq = 16000;
    
    // Handle scenario where EI_CLASSIFIER_FREQUENCY != I2S sample rate
    // Will skip packing EI buffer at this rate
    // i.e. if I2S sample rate = 16000 Hz & EI_CLASSIFIER_FREQUENCY = 4000Hz
    // ei_skip_rate = 4
    int ei_skip_rate = 1;
    inference_t inference;

protected:
    bool configureI2S();

public:
    I2SMEMSSampler(
        i2s_port_t i2s_port,
        i2s_pin_config_t &i2s_pins,
        i2s_config_t i2s_config,
        bool fixSPH0645 = false);
    
    /**
     * @brief Register an external WAVFileWriter
     * @note This object will fill the WAVFileWriter buffer during reads
     * 
     * @param writer WAVFileWriter object
     * @return true success
     */
    virtual bool register_wavFileWriter(WAVFileWriter *writer);

    /**
     * @brief Register a edge impulse inference structure
     * @note This object will fill the inference buffer during reads
     * 
     * @param ext_inference the Edge Impulse inference structure
     * @param ext_ei_sampling_freq the sampling frequency of the Edge Impulse model
     * @return true success
    */
    virtual bool register_ei_inference(inference_t ext_inference, int ext_ei_sampling_freq);
    
    /**
     * @brief Read I2S samples from DMA buffer
     * @return The number of SAMPLES (i.e. not bytes) read
    */
    virtual int read(int count);
};
