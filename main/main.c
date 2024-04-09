#include <stdio.h>
#include "driver/i2s_std.h"
#include "driver/gpio.h"

//Multitasking und Delay
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "esp_dsp.h"

//Define Pins
#define MCK_0 GPIO_NUM_0
#define MCK_1 GPIO_NUM_3
#define WS_0  GPIO_NUM_32
#define WS_1  GPIO_NUM_2
#define BCK_0  GPIO_NUM_26
#define BCK_1  GPIO_NUM_16
#define DIN_0  GPIO_NUM_25
#define DIN_1  GPIO_NUM_5
#define DOUT_0  GPIO_NUM_14
#define DOUT_1  GPIO_NUM_22


//Define Audio Settings
#define BitWidth 24
#define Samplerate 96000
#define k_buffer 1280

//iir
float *coeff;
float *w;

//I2S-Controller 0  Handles
static i2s_chan_handle_t tx_handle_0;
static i2s_chan_handle_t rx_handle_0;

//I2S-Controller 1  Handles
static i2s_chan_handle_t tx_handle_1;
static i2s_chan_handle_t rx_handle_1;


//evtl ist output als 4 kanäle zu implementieren
void converting(int8_t *in, float *left, float *right){
    int j = 0;
    //seperating the channels and converting from 3x8bit int into 1x32bit float valued buffers
    for(int i = 0; i < k_buffer*BitWidth/8/(BitWidth/8 +1); i++){
        left[i] = ((float)(((in[j+1] << 16) & 0xFF0000) | ((in[j+2] << 8) & 0xFF00) | (in[j+3] & 0xFF)))/8388608.0f;
       
        right[i] = (float)(((in[j+5] << 16) & 0xFF0000) | ((in[j+6] << 8) & 0xFF00) | (in[j+7] & 0xFF))/8388608.0f;

        j += 2 * (BitWidth/8 + 1);
    }

}

void reconverting(int8_t *out, float *left, float *right){
    int j = 0;
    for(int i = 0; i < (k_buffer*BitWidth*2/8)/(2*(BitWidth/8 + 1)); i++){

            //Left
        float l = 8388607.0f*left[i];
        if(l > 8388607) l = 8388607;
        else if(l < -8388608) l = -8388608;
        out[j] = 0x00;
        out[j+1] = (int8_t)(((int32_t)(l) >> 16) & 0xFF);
        out[j+2] = (int8_t)(((int32_t)(l) >> 8) & 0xFF);
        out[j+3] = (int8_t)((int32_t)(l) & 0xFF);


            //Right
        float r = 8388608.0f*right[i];
        if(r > 8388607) r = 8388607;
        else if(r < -8388608) r = -8388608;
        out[j+4] = 0x00;
        out[j+5] = (int8_t)(((int32_t)(r) >> 16) & 0xFF);
        out[j+6] = (int8_t)(((int32_t)(r) >> 8) & 0xFF);
        out[j+7] = (int8_t)((int32_t)(r) & 0xFF);


        j+=2*(BitWidth/8 + 1);
    } 
}

//evtl output als 4 kanäle
void eq(float *ch1, float *ch2, float *ch3, float *ch4){
    float *temp3 = (float *)calloc((k_buffer*BitWidth*2/8)/(2*(BitWidth/8 + 1)), sizeof(float));
    
    for(int i = 0; i<(k_buffer*BitWidth*2/8)/(2*(BitWidth/8 + 1)); i++){
        ch1[i]*=1;
        //ch2[i]*=1.5f;
        temp3[i] = ch3[i];
    }
    dsps_addc_f32_ae32(ch2, ch2, (k_buffer*BitWidth*2/8)/(2*(BitWidth/8 + 1)), 1.2f, 1, 1);
    
    ESP_ERROR_CHECK(dsps_biquad_f32_ae32( temp3, ch3, (k_buffer*BitWidth*2/8)/(2*(BitWidth/8 + 1)), coeff, w));
    free(temp3);
}

void groupdelay(){

}

void i2s_init(){
    //vTaskDelay(2000);
    //I2S-Controller Full-Duplex Modus
    const i2s_chan_config_t chan_cfg_0 = {
                                            .id = I2S_NUM_0,
                                            .role = I2S_ROLE_MASTER, 
                                            .dma_desc_num = 6, 
                                            .dma_frame_num = 240, 
                                            .auto_clear = true, 
                                            .intr_priority = 0,
    };
    const i2s_chan_config_t chan_cfg_1 = {
                                            .id = I2S_NUM_1,
                                            .role = I2S_ROLE_MASTER, 
                                            .dma_desc_num = 6, 
                                            .dma_frame_num = 240, 
                                            .auto_clear = true, 
                                            .intr_priority = 0,
    };


    //I2S-Controller Full-Duplex in REGISTERED zustand bringen
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg_0, &tx_handle_0, &rx_handle_0));
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg_1, &tx_handle_1, &rx_handle_1));


    //Konfiguration aller Kanäle
    const i2s_std_config_t std_cfg_0 = {
        .clk_cfg = {
            .sample_rate_hz = Samplerate,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_384,
        },
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(BitWidth, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = MCK_0,
            .bclk = BCK_0,
            .ws = WS_0,
            .dout = DOUT_0,
            .din = DIN_0,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    const i2s_std_config_t std_cfg_1 = {
        .clk_cfg = {
            .sample_rate_hz = Samplerate,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_384,
        },
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(BitWidth, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = MCK_1,
            .bclk = BCK_1,
            .ws = WS_1,
            .dout = DOUT_1,
            .din = DIN_1,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };


    //I2S-Kanäle auf READY zustand bringen
    
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_0, &std_cfg_0));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_0, &std_cfg_0));
    
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_1, &std_cfg_1));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_1, &std_cfg_1));


    //I2S-Kanäle auf RUNNING zustand bringen
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle_0));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle_0));
    
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle_1));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle_1));
    

}

void i2s_loop(){
    
    //Defines a buffer as dynamic array with k_buffer*BitdWidth*2/8 length. 
    //When using 24bit stereo there are 3 times 8bit -> 24bit audio left channel followed by 0x00. Then comes the right channel.
    int8_t *data_0 = (int8_t *)calloc(1, k_buffer*BitWidth*2/8); 
    int8_t *data_1 = (int8_t *)calloc(1, k_buffer*BitWidth*2/8);
    

    //float data-buffer per channel
    float *chan0 = (float *)calloc((k_buffer*BitWidth*2/8)/(2*(BitWidth/8 + 1)), sizeof(float));
    float *chan1 = (float *)calloc((k_buffer*BitWidth*2/8)/(2*(BitWidth/8 + 1)), sizeof(float));
    float *chan2 = (float *)calloc((k_buffer*BitWidth*2/8)/(2*(BitWidth/8 + 1)), sizeof(float));
    float *chan3 = (float *)calloc((k_buffer*BitWidth*2/8)/(2*(BitWidth/8 + 1)), sizeof(float));


    while(true){
        
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2s_channel_read(rx_handle_0, data_0, k_buffer*BitWidth*2/8, NULL, 1000));
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2s_channel_read(rx_handle_1, data_1, k_buffer*BitWidth*2/8, NULL, 1000));
        
        //print part buffer
        //printf("\n\n");
        //printf("0x%02X L: 0x%02X 0x%02X 0x%02X   0x%02X R:0x%02X 0x%02X 0x%02X   ", data_0[0], data_0[1] & 0xFF, data_0[2] & 0xFF, data_0[3] & 0xFF, data_0[4], data_0[5] & 0xFF, data_0[6] & 0xFF, data_0[7] & 0xFF);
        //printf("0x%02X L: 0x%02X 0x%02X 0x%02X   0x%02X R:0x%02X 0x%02X 0x%02X \n", data_0[8], data_0[9] & 0xFF, data_0[10] & 0xFF, data_0[11] & 0xFF, data_0[12], data_0[13] & 0xFF, data_0[14] & 0xFF, data_0[15] & 0xFF);
    
        
    
        //Converting
        converting(data_0, chan0, chan1);
        converting(data_1, chan2, chan3);

        //Routing

        //EQ 
        eq(chan2, chan1, chan0, chan3);


        //Group Delay
        //groupdelay();

        //Reconverting
        reconverting(data_0, chan0, chan1);
        reconverting(data_1, chan2, chan3);

        //printf("0x%02X L: 0x%02X 0x%02X 0x%02X   0x%02X R:0x%02X 0x%02X 0x%02X   ", data_0[0], data_0[1] & 0xFF, data_0[2] & 0xFF, data_0[3] & 0xFF, data_0[4], data_0[5] & 0xFF, data_0[6] & 0xFF, data_0[7] & 0xFF);
        //printf("0x%02X L: 0x%02X 0x%02X 0x%02X   0x%02X R:0x%02X 0x%02X 0x%02X \n", data_0[8], data_0[9] & 0xFF, data_0[10] & 0xFF, data_0[11] & 0xFF, data_0[12], data_0[13] & 0xFF, data_0[14] & 0xFF, data_0[15] & 0xFF);

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2s_channel_write(tx_handle_0, data_0, k_buffer*BitWidth*2/8, NULL, NULL));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2s_channel_write(tx_handle_1, data_1, k_buffer*BitWidth*2/8, NULL, NULL));

    //vTaskDelay(2000);
    }
    free(data_0);
    free(data_1);
    free(chan0);
    free(chan1);
    free(chan2);
    free(chan3);
}

void testsetup(){
    //iir coeff
    coeff = (float *)calloc(5, 4);
    w = (float *)calloc(5, 4);
    for(int i = 0; i<5; i++){
        w[i] = 0;
    }
    ESP_ERROR_CHECK(dsps_biquad_gen_lpf_f32(coeff, 120.0f/Samplerate, 0.707f));
    //Implementieren als calloc
}

void app_main(void)
{
    //Turn on Blue LED to see if flashed correctly
    esp_rom_gpio_pad_select_gpio(2);
    gpio_set_direction(2, GPIO_MODE_OUTPUT);
    gpio_set_level(2, 1);

    testsetup();
    //i2s part
    i2s_init();
    i2s_loop();
    free(coeff);
    free(w);
}
