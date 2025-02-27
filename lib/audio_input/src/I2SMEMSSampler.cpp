#include "I2SMEMSSampler.h"
#include "soc/i2s_reg.h"

I2SMEMSSampler::I2SMEMSSampler(
    i2s_port_t i2s_port,
    i2s_pin_config_t &i2s_pins,
    i2s_config_t i2s_config,
    bool fixSPH0645) : I2SSampler(i2s_port, i2s_config)
{
    m_i2sPins = i2s_pins;
    m_fixSPH0645 = fixSPH0645;
}

void I2SMEMSSampler::configureI2S()
{
    if (m_fixSPH0645)
    {
        // FIXES for SPH0645
        REG_SET_BIT(I2S_TIMING_REG(m_i2sPort), BIT(9));
        REG_SET_BIT(I2S_CONF_REG(m_i2sPort), I2S_RX_MSB_SHIFT);
    }

    i2s_set_pin(m_i2sPort, &m_i2sPins);
}

int I2SMEMSSampler::read(int16_t *samples, int count)
{
    // read from i2s
    int32_t *raw_samples = (int32_t *)malloc(sizeof(int32_t) * count);
    size_t bytes_read = 0;
    
    i2s_read(m_i2sPort, raw_samples, sizeof(int32_t) * count, &bytes_read, portMAX_DELAY);
    
    // samples_read is the number of 32 bit samples read = 4096
    // bytes_read is the number of bytes read = 1024
    
    int samples_read = bytes_read / sizeof(int32_t);
    
    for (int i = 0; i < samples_read; i++)
    {
        // For the SPH0645LM4H-B MEMS microphone
        // The Data Format is I2S, 24-bit, 2’s compliment, MSB first.
        // The data precision is 18 bits; unused bits are zeros.

        // We need to store data in 16 bits so need to drop lower 16 bits
        
        samples[i] = raw_samples[i] >> (32 - 16);
    }

    free(raw_samples);
    return samples_read;
}
