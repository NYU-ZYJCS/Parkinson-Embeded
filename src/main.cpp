#include <mbed.h>
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "stm32f429xx.h"
#include"KalmanFilter.h"
#include "analyse.h"
#include <LCD_DISCO_F429ZI.h>

KalmanFilter kf_x(0.01, 0.003);
KalmanFilter kf_y(0.01, 0.003);
KalmanFilter kf_z(0.01, 0.003);

// TODOs:
// [1] Get started with an SPI object instance and connect to the Gyroscope!
// [2] Read the XYZ axis from the Gyroscope and Visualize on the Teleplot. 
// [3] Fetching Data from the sensor via Polling vs Interrupt ?

// Define control register addresses and their configurations
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define SPI_FLAG 1
#define OUT_X_L 0x28

#define FFT_SIZE 64
#define SAMPLE_RATE 100 // 采样频率
#define PI 3.14159265358979323846

LCD_DISCO_F429ZI lcd;
const uint32_t BUFFER_SIZE = 20;
float32_t gx_buffer[BUFFER_SIZE], gy_buffer[BUFFER_SIZE], gz_buffer[BUFFER_SIZE];
uint32_t buffer_index = 0;


EventFlags flags;

void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}

void lcd_x_y_z(float gx, float gy, float gz)
{
  uint8_t message_gx[20]; // 分配足够的数组大小
  uint8_t message_gy[20];
  uint8_t message_gz[20];

  // 分别格式化每个轴的数据
  sprintf((char *)message_gx, "X: %f", gx);
  sprintf((char *)message_gy, "Y: %f", gy);
  sprintf((char *)message_gz, "Z: %f", gz);

  // 分别在不同的行显示每个轴的数据
  lcd.DisplayStringAt(0, LINE(6), (uint8_t *)&message_gx, CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(7), (uint8_t *)&message_gy, CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(8), (uint8_t *)&message_gz, CENTER_MODE);
}

#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)


int main()
{
    // Initialize the SPI object with specific pins.
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);

    // Buffers for sending and receiving data over SPI.
    uint8_t write_buf[32], read_buf[32];

    // Configure SPI format and frequency.
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Configure CTRL_REG1 register.
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    // Configure CTRL_REG4 register.
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    //FFT:
    arm_rfft_instance_f32 S;
    arm_cfft_radix4_instance_f32 S_CFFT;
    arm_rfft_init_f32(&S, &S_CFFT, FFT_SIZE, 0, 1);

    float32_t gx_fft_input[FFT_SIZE];
    float32_t gx_fft_output[FFT_SIZE * 2];


    while(1){

        int16_t raw_gx, raw_gy, raw_gz;
        float gx, gy, gz;

        // Prepare to read the gyroscope values starting from OUT_X_L
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        // Perform the SPI transfer to read 6 bytes of data (for x, y, and z axes)
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Convert the received data into 16-bit integers for each axis
        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t) read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t) read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t) read_buf[5]);

        // Print the raw values for debugging 
         //printf("RAW -> \t\t\tgx: %d \t gy: %d \t gz: %d \t\n", raw_gx, raw_gy, raw_gz);

            // printf(">x_axis: %d|g \n", raw_gx);
            // printf(">y_axis: %d|g \n", raw_gy);
            // printf(">z_axis: %d|g \n", raw_gz);

        // Convert raw data to actual values using a scaling factor
        gx = ((float) raw_gx) * SCALING_FACTOR ;
        
        gy = ((float) raw_gy) * SCALING_FACTOR;
  
        gz = ((float) raw_gz) * SCALING_FACTOR  ;

        // Print the actual values
        //printf("Actual -> \t\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f \t\n", gx, gy, gz);
        
        // 假设dt为采样间隔,单位为秒
        float dt = 0.1;

        // 对gx进行滤波
        float gx_filtered = kf_x.update(gx, 0, dt);

        // 对gy进行滤波  
        float gy_filtered = kf_y.update(gy, 0, dt);

        // 对gz进行滤波
        float gz_filtered = kf_z.update(gz, 0, dt);
        
        // Print the filtered values
        //printf("Filtered -> \t\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f \t\n", gx_filtered, gy_filtered, gz_filtered);
        
        // 分析时域特征
        // 将滤波后的数据存入缓冲区
        // gx_buffer[buffer_index] = gx_filtered;
        // gy_buffer[buffer_index] = gy_filtered;
        // gz_buffer[buffer_index] = gz_filtered;
        gx_buffer[buffer_index] = gx;
        gy_buffer[buffer_index] = gy;
        gz_buffer[buffer_index] = gz;
        buffer_index++;

        // 如果缓冲区已满,计算时域特征并清空缓冲区
        if (buffer_index == BUFFER_SIZE) {
            // // 计算gx的时域特征
            // float32_t gx_mean, gx_variance, gx_rms, gx_peak_to_peak;
            // time_domain_features(gx_buffer, BUFFER_SIZE, &gx_mean, &gx_variance, &gx_rms, &gx_peak_to_peak);
            // printf("Time-domain features for gx:\n");
            // printf("Mean: %.5f\n", gx_mean);
            // printf("Variance: %.5f\n", gx_variance);
            // printf("RMS: %.5f\n", gx_rms);
            // printf("Peak-to-peak: %.5f\n", gx_peak_to_peak);

            // // 计算gy的时域特征
            // float32_t gy_mean, gy_variance, gy_rms, gy_peak_to_peak;
            // time_domain_features(gy_buffer, BUFFER_SIZE, &gy_mean, &gy_variance, &gy_rms, &gy_peak_to_peak);
            // printf("Time-domain features for gy:\n");
            // printf("Mean: %.5f\n", gy_mean);
            // printf("Variance: %.5f\n", gy_variance);
            // printf("RMS: %.5f\n", gy_rms);
            // printf("Peak-to-peak: %.5f\n", gy_peak_to_peak);

            // // 计算gz的时域特征
            // float32_t gz_mean, gz_variance, gz_rms, gz_peak_to_peak;
            // time_domain_features(gz_buffer, BUFFER_SIZE, &gz_mean, &gz_variance, &gz_rms, &gz_peak_to_peak);
            // printf("Time-domain features for gz:\n");
            // printf("Mean: %.5f\n", gz_mean);
            // printf("Variance: %.5f\n", gz_variance);
            // printf("RMS: %.5f\n", gz_rms);
            // printf("Peak-to-peak: %.5f\n", gz_peak_to_peak);

            // 频谱分析
            // 将gx_buffer复制到gx_fft_input,如果gx_buffer长度小于FFT_SIZE,则补零
            memcpy(gx_fft_input, gx_buffer, sizeof(float32_t) * BUFFER_SIZE);
            if (BUFFER_SIZE < FFT_SIZE) {
                memset(&gx_fft_input[BUFFER_SIZE], 0, sizeof(float32_t) * (FFT_SIZE - BUFFER_SIZE));
            }

            // 执行FFT
            arm_rfft_f32(&S, gx_fft_input, gx_fft_output);

            // 计算频谱幅值
            float32_t gx_spectrum[FFT_SIZE / 2];
            arm_cmplx_mag_f32(gx_fft_output, gx_spectrum, FFT_SIZE / 2);

            // 频谱功率分析
            float32_t gx_power[FFT_SIZE / 2];
            arm_cmplx_mag_squared_f32(gx_fft_output, gx_power, FFT_SIZE / 2);
    
            // 频谱相位分析
            float32_t gx_phase[FFT_SIZE / 2];
            for (int i = 0; i < FFT_SIZE / 2; i++) {
                gx_phase[i] = atan2f(gx_fft_output[2*i+1], gx_fft_output[2*i]);
            }
    
            // 频谱峰值分析
            float32_t gx_peak = 0;
            uint32_t gx_peak_index = 0;
            arm_max_f32(gx_spectrum, FFT_SIZE / 2, &gx_peak, &gx_peak_index);
    
            // 频带能量分析
            float32_t gx_band_power[4] = {0}; // 假设我们将频谱分为4个频带
            for (int i = 0; i < FFT_SIZE / 2; i++) {
                if (i < FFT_SIZE / 8) {
                gx_band_power[0] += gx_power[i];
            } else if (i < FFT_SIZE / 4) {
                gx_band_power[1] += gx_power[i];
            } else if (i < FFT_SIZE * 3 / 8) {
                gx_band_power[2] += gx_power[i];
            } else {
                gx_band_power[3] += gx_power[i];
            }
            }

        
            // 打印频谱
            printf("Frequency Spectrum for gx:\n");
            for (int i = 0; i < FFT_SIZE / 2; i++) {
                float32_t freq = (float32_t)i * (SAMPLE_RATE / FFT_SIZE);
            printf("%4.0f Hz: %.5f, %.5f, %.5f\n", freq, gx_spectrum[i], gx_power[i], gx_phase[i]);
            }
    
            // 打印峰值
            printf("Peak frequency: %4.0f Hz, Peak magnitude: %.5f\n", gx_peak_index * (SAMPLE_RATE / FFT_SIZE), gx_peak);
    
            // 打印频带能量
            printf("Band power: %.5f, %.5f, %.5f, %.5f\n", gx_band_power[0], gx_band_power[1], gx_band_power[2], gx_band_power[3]);

             
            
            buffer_index = 0;  // 清空缓冲区


        }

        ThisThread::sleep_for(10ms);


    }
}

