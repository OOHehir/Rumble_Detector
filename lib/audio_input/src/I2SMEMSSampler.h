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
    int ei_sampling_freq;
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
    
    virtual int read(int count);
};
