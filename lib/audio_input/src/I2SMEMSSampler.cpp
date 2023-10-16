#include "I2SMEMSSampler.h"
#include "soc/i2s_reg.h"
#include "esp_log.h"

static const char *TAG = "I2SMEMSSampler";

I2SMEMSSampler::I2SMEMSSampler(
    i2s_port_t i2s_port,
    i2s_pin_config_t &i2s_pins,
    i2s_config_t i2s_config,
    bool fixSPH0645) : I2SSampler(i2s_port, i2s_config)
{
    m_i2sPins = i2s_pins;
    m_fixSPH0645 = fixSPH0645;

    writer = nullptr;
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

bool I2SMEMSSampler::register_wavfilewriter(WAVFileWriter *ext_writer){
    writer = ext_writer;

    if (writer == nullptr){
        ESP_LOGE(TAG, "register_wavfilewriter() - WAVFileWriter not registered");
        return false;
    }else{
        return true;
    }
        
}

int I2SMEMSSampler::read(int count)
{
    //ESP_LOGI(TAG, "I2SMEMSSampler::read()");

    if(writer == nullptr){
        ESP_LOGE(TAG, "read() - WAVFileWriter not registered");
        return 0;
    }

    // read from i2s
    int32_t *raw_samples = (int32_t *)malloc(sizeof(int32_t) * count);

    if (raw_samples == NULL)
    {
        ESP_LOGE(TAG, "Could not allocate memory for samples");
        return 0;
    }

    size_t bytes_read = 0;
    
    i2s_read(m_i2sPort, raw_samples, sizeof(int32_t) * count, &bytes_read, portMAX_DELAY);
    
    // bytes_read = number of bytes read = 4096
    // samples_read = number of 32 bit samples read = 1024
    
    int samples_read = bytes_read / sizeof(int32_t);

    //ESP_LOGI(TAG, "samples_read = %d", samples_read);  
    
    for (int i = 0; i < samples_read; i++)
    {   
        
        //ESP_LOGI(TAG, "buffer_idx[] = %d", writer->buffer_idx[writer->buffer_active]);
        //ESP_LOGI(TAG, "buffer_active = %d", writer->buffer_active);
        
        // For the SPH0645LM4H-B MEMS microphone
        // The Data Format is I2S, 24-bit, 2’s compliment, MSB first.
        // The data precision is 18 bits; unused bits are zeros.

        // We need to store data in 16 bits so need to drop lower 16 bits

        // Store into wav file buffer
        if (writer->buffer_idx[writer->buffer_active] < writer->buffer_size){
            writer->buffer[writer->buffer_active][writer->buffer_idx[writer->buffer_active]] = raw_samples[i] >> 11;
            writer->buffer_idx[writer->buffer_active]++;
        }
        else{
            writer->buffer_is_full();  // Swap buffers and set buffer_ready_to_save = true
            writer->write();
        }
        
        
        
        // Store into edge-impulse buffer
        // sampleBuffer[i] = raw_samples[i] >> 11;

    }

    free(raw_samples);
    return samples_read;
}
