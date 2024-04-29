#include <mbed.h>
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "stm32f429xx.h"
#include "KalmanFilter.h"
#include "analyse.h"
#include <LCD_DISCO_F429ZI.h>


KalmanFilter kf_x(0.01, 0.03);
KalmanFilter kf_y(0.01, 0.03);
KalmanFilter kf_z(0.01, 0.03);

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
#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)



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
        gx = ((float) raw_gx) * SCALING_FACTOR;
        gy = ((float) raw_gy) * SCALING_FACTOR;
        gz = ((float) raw_gz) * SCALING_FACTOR;

        // Print the actual values
        //printf("Actual -> \t\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f \t\n", gx, gy, gz);
        lcd_x_y_z(gx, gy, gz);
        
        // 假设dt为采样间隔,单位为秒
        float dt = 0.1;

        // 对gx进行滤波
        float gx_filtered = kf_x.update(gx, 0, dt);

        // 对gy进行滤波  
        float gy_filtered = kf_y.update(gy, 0, dt);

        // 对gz进行滤波
        float gz_filtered = kf_z.update(gz, 0, dt);
        
        // Print the filtered values
        printf("Filtered -> \t\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f \t\n", gx_filtered, gy_filtered, gz_filtered);
        // 分析时域特征
        // 将滤波后的数据存入缓冲区
        gx_buffer[buffer_index] = gx_filtered;
        gy_buffer[buffer_index] = gy_filtered;
        gz_buffer[buffer_index] = gz_filtered;
        buffer_index++;

        // 如果缓冲区已满,计算时域特征并清空缓冲区
        if (buffer_index == BUFFER_SIZE) {
            // 计算gx的时域特征
            float32_t gx_mean, gx_variance, gx_rms, gx_peak_to_peak;
            time_domain_features(gx_buffer, BUFFER_SIZE, &gx_mean, &gx_variance, &gx_rms, &gx_peak_to_peak);
            printf("Time-domain features for gx:\n");
            printf("Mean: %.5f\n", gx_mean);
            printf("Variance: %.5f\n", gx_variance);
            printf("RMS: %.5f\n", gx_rms);
            printf("Peak-to-peak: %.5f\n", gx_peak_to_peak);

            // 计算gy的时域特征
            float32_t gy_mean, gy_variance, gy_rms, gy_peak_to_peak;
            time_domain_features(gy_buffer, BUFFER_SIZE, &gy_mean, &gy_variance, &gy_rms, &gy_peak_to_peak);
            printf("Time-domain features for gy:\n");
            printf("Mean: %.5f\n", gy_mean);
            printf("Variance: %.5f\n", gy_variance);
            printf("RMS: %.5f\n", gy_rms);
            printf("Peak-to-peak: %.5f\n", gy_peak_to_peak);

            // 计算gz的时域特征
            float32_t gz_mean, gz_variance, gz_rms, gz_peak_to_peak;
            time_domain_features(gz_buffer, BUFFER_SIZE, &gz_mean, &gz_variance, &gz_rms, &gz_peak_to_peak);
            printf("Time-domain features for gz:\n");
            printf("Mean: %.5f\n", gz_mean);
            printf("Variance: %.5f\n", gz_variance);
            printf("RMS: %.5f\n", gz_rms);
            printf("Peak-to-peak: %.5f\n", gz_peak_to_peak);

            buffer_index = 0;  // 清空缓冲区
        }
       
        // ThisThread::sleep_for(1000ms);


    }
}
