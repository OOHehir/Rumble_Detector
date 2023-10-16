#pragma once

#include "I2SSampler.h"
#include "WAVFileWriter.h"

class I2SMEMSSampler : public I2SSampler
{
private:
    i2s_pin_config_t m_i2sPins;
    bool m_fixSPH0645;
    WAVFileWriter *writer;

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
    
    virtual int read(int count);
};
