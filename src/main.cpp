#include <mbed.h>
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "stm32f429xx.h"
#include "KalmanFilter.h"
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
#define CTRL_REG4 0x23
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define SPI_FLAG 1
#define OUT_X_L 0x28
#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)
#define CUTOFF_FREQUENCY 20

#define FFT_SIZE 256
#define SAMPLE_RATE 100 // 采样频率
#define PI 3.14159265358979323846

LCD_DISCO_F429ZI lcd;
const uint32_t BUFFER_SIZE = 50;
float32_t gx_buffer[BUFFER_SIZE], gy_buffer[BUFFER_SIZE], gz_buffer[BUFFER_SIZE];
uint32_t buffer_index = 0;
Timer t;
float time_elapsed;

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

void applyHanningWindow(float32_t *input, int size) {
    for (int i = 0; i < size; i++) {
        float window = 0.5 - 0.5 * cos(2 * M_PI * i / (size - 1));
        input[i] *= window;
    }
}

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

        // Convert raw data to actual values using a scaling factor
        gx = ((float) raw_gx) * SCALING_FACTOR ;
        gy = ((float) raw_gy) * SCALING_FACTOR;
        gz = ((float) raw_gz) * SCALING_FACTOR  ;

        gx_buffer[buffer_index] = gx;
        gy_buffer[buffer_index] = gy;
        gz_buffer[buffer_index] = gz;
        buffer_index++;

        // 如果缓冲区已满,计算时域特征并清空缓冲区
        if (buffer_index == BUFFER_SIZE) {
            // 频谱分析
            // 将gx_buffer复制到gx_fft_input,如果gx_buffer长度小于FFT_SIZE,则补零
            memcpy(gx_fft_input, gx_buffer, sizeof(float32_t) * BUFFER_SIZE);
            if (BUFFER_SIZE < FFT_SIZE) {
                memset(&gx_fft_input[BUFFER_SIZE], 0, sizeof(float32_t) * (FFT_SIZE - BUFFER_SIZE));
            }

            applyHanningWindow(gx_fft_input, FFT_SIZE);
            // 执行FFT
            arm_rfft_f32(&S, gx_fft_input, gx_fft_output);

            // FFT输出后的滤波处理
            for (int i = 0; i < FFT_SIZE / 2; i++) {
                float frequency = (float)i * SAMPLE_RATE / FFT_SIZE;
                if (frequency > CUTOFF_FREQUENCY) {
                    gx_fft_output[2*i] = 0;     // 实部置零
                    gx_fft_output[2*i + 1] = 0; // 虚部置零
                }
            }

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
    
            // 计算频谱峰值
            float32_t gx_peak = 0;
            uint32_t gx_peak_index = 0;
            arm_max_f32(gx_spectrum, FFT_SIZE / 2, &gx_peak, &gx_peak_index);

            // 计算频带能量
            float32_t gx_band_power[4] = {0}; // 假设我们将频谱分为4个频带
            for (int i = 0; i < FFT_SIZE / 2; i++) {
                int band = i / (FFT_SIZE / 8);
                if (band < 4) {  // 确保不会超出数组索引
                    gx_band_power[band] += gx_power[i];
                }
            }

            
        
            //打印频谱
            // printf("Frequency Spectrum for gx:\n");
            // for (int i = 0; i < FFT_SIZE / 2; i++) {
            //     float32_t freq = (float32_t)i * ((float32_t)SAMPLE_RATE / FFT_SIZE);
            //     printf("%4.0f Hz: %.5f, %.5f, %.5f\n", freq, gx_spectrum[i], gx_power[i], gx_phase[i]);
            // }
            // printf(">gx:%f\n", gx_filtered);
            // printf(">gy:%f\n", gy_filtered);
            // printf(">gz:%f\n", gz_filtered);
    
            // 打印峰值
            printf("Peak frequency: %4.0f Hz, Peak magnitude: %f\n", (float)gx_peak_index * ((float)SAMPLE_RATE / FFT_SIZE), gx_peak);
    
            // 打印频带能量
            printf("Band power: %.5f, %.5f, %.5f, %.5f\n", gx_band_power[0], gx_band_power[1], gx_band_power[2], gx_band_power[3]);

            buffer_index = 0;  // 清空缓冲区
        }
        ThisThread::sleep_for(10ms);
    }
}

