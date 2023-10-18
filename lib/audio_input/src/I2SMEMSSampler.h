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

    uint32_t i2s_sampling_rate;

    uint32_t ei_sampling_freq;
    
    // Handle scenario where EI_CLASSIFIER_FREQUENCY != I2S sample rate
    // Will skip packing EI buffer at this rate
    // i.e. if I2S sample rate = 16000 Hz & EI_CLASSIFIER_FREQUENCY = 4000Hz
    // ei_skip_rate = 4
    int ei_skip_rate;
    inference_t inference;

protected:
    void configureI2S();

public:
    I2SMEMSSampler(
        i2s_port_t i2s_port,
        i2s_pin_config_t &i2s_pins,
        i2s_config_t i2s_config,
        bool fixSPH0645 = false);
    
    /**
     * @brief Register an external WAVFileWriter
     * 
     * @param writer WAVFileWriter object
     * @return true success
     * @return false failure
     */
    virtual bool register_wavFileWriter(WAVFileWriter *writer);

    /**
     * @brief Register a edge impulse inference structure
     * @return true success
     *  
    */
    virtual bool register_ei_inference(inference_t ext_inference, int ext_ei_sampling_freq);
    
    /**
     * @brief Read I2S samples from DMA buffer &
    */
    virtual int read(int count);
};
