#include <mbed.h>
#include <cmath>
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "stm32f429xx.h"
#include "KalmanFilter.h"
#include "analyse.h"
#include <LCD_DISCO_F429ZI.h>

KalmanFilter kf_x(0.03, 0.005);
KalmanFilter kf_y(0.03, 0.005);
KalmanFilter kf_z(0.03, 0.005);

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

#define FFT_SIZE 128
#define SAMPLE_RATE 50 // 采样频率
#define PI 3.14159265358979323846

LCD_DISCO_F429ZI lcd;
const uint32_t BUFFER_SIZE = 50;
const uint32_t time_window = 10;
const uint32_t threshold = 7;
uint32_t buffer_index = 0;
uint32_t seconds = 0;

arm_rfft_instance_f32 S;
arm_cfft_radix4_instance_f32 S_CFFT;

EventFlags flags;

void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}


void applyHanningWindow(float32_t *input, int size) {
    for (int i = 0; i < size; i++) {
        float window = 0.5 - 0.5 * cos(2 * M_PI * i / (size - 1));
        input[i] *= window;
    }
}

bool check(int16_t freq, float32_t magnitude) {

    return freq >= 3 && freq <= 6;
}

pair<uint32_t, float32_t> findPeak(float32_t *buffer) {
    arm_rfft_init_f32(&S, &S_CFFT, FFT_SIZE, 0, 1);

    float32_t fft_input[FFT_SIZE];
    float32_t fft_output[FFT_SIZE * 2];


    // 频谱分析
    // 将gx_buffer复制到gx_fft_input,如果gx_buffer长度小于FFT_SIZE,则补零
    memcpy(fft_input, buffer, sizeof(float32_t) * BUFFER_SIZE);
    if (BUFFER_SIZE < FFT_SIZE) {
        memset(&fft_input[BUFFER_SIZE], 0, sizeof(float32_t) * (FFT_SIZE - BUFFER_SIZE));
    }

    applyHanningWindow(fft_input, FFT_SIZE);
    // 执行FFT
    arm_rfft_f32(&S, fft_input, fft_output);

    // FFT输出后的滤波处理
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float frequency = (float)i * SAMPLE_RATE / FFT_SIZE;
        if (frequency > CUTOFF_FREQUENCY) {
            fft_output[2*i] = 0;     // 实部置零
            fft_output[2*i + 1] = 0; // 虚部置零
        }
    }

    // 计算频谱幅值
    float32_t spectrum[FFT_SIZE / 2];
    arm_cmplx_mag_f32(fft_output, spectrum, FFT_SIZE / 2);

    // 计算频谱峰值
    float32_t peak = 0;
    uint32_t peak_index = 0;
    arm_max_f32(spectrum, FFT_SIZE / 2, &peak, &peak_index);

    return {peak_index, peak};
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

    float32_t gx_buffer[BUFFER_SIZE], gy_buffer[BUFFER_SIZE], gz_buffer[BUFFER_SIZE];
    uint32_t count = 0;

    while(1){
        int16_t raw_gx, raw_gy, raw_gz;
        float gx, gy, gz;
        int16_t tremors_per_second, tremors_sum;
        float32_t tremor_magnitude, tremor_magnitude_sum;

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

        lcd.Clear(LCD_COLOR_WHITE);
        uint8_t message_countdown[20];
        sprintf((char *)message_countdown, "Detecting... %d", 10 - seconds);
        lcd.DisplayStringAt(0, LINE(4), message_countdown, CENTER_MODE);

        uint8_t message_tremors_cur_second[20];
        uint8_t message_magnitude_cur_second[20];

        sprintf((char *)message_tremors_cur_second, "Tremors: %d/s", tremors_per_second);
        sprintf((char *)message_magnitude_cur_second, "Magnitude: %f", tremor_magnitude);
        lcd.DisplayStringAt(0, LINE(12), (uint8_t *)"Current Second's Data", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(13), (uint8_t *)message_tremors_cur_second, CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(14), (uint8_t *)message_magnitude_cur_second, CENTER_MODE);
        
        // 假设dt为采样间隔,单位为秒
        float dt = 0.05;
        // 对gx进行滤波
        float32_t gx_filtered = kf_x.update(gx, 0, dt);
        float32_t gy_filtered = kf_y.update(gy, 0, dt);
        float32_t gz_filtered = kf_z.update(gz, 0, dt);
        

        gx_buffer[buffer_index] = gx_filtered;
        gy_buffer[buffer_index] = gy_filtered;
        gz_buffer[buffer_index] = gz_filtered;
        buffer_index++;

        // 如果缓冲区已满,计算时域特征并清空缓冲区
        if (buffer_index == BUFFER_SIZE) {
            seconds++;
            // Call findPeak and store the results
            std::pair<uint32_t, float32_t> gx_peak_result = findPeak(gx_buffer);
            std::pair<uint32_t, float32_t> gy_peak_result = findPeak(gy_buffer);
            std::pair<uint32_t, float32_t> gz_peak_result = findPeak(gz_buffer);

            std::array<std::pair<int16_t, float>, 3> peak_results = {gx_peak_result, gy_peak_result, gz_peak_result};
            peak_results[0].first = round(gx_peak_result.first * ((float)SAMPLE_RATE / FFT_SIZE));
            peak_results[1].first = round(gy_peak_result.first * ((float)SAMPLE_RATE / FFT_SIZE));
            peak_results[2].first = round(gz_peak_result.first * ((float)SAMPLE_RATE / FFT_SIZE));

            std::sort(peak_results.begin(), peak_results.end(), [](const std::pair<int16_t, float> &a, const std::pair<int16_t, float> &b) {
                return a.second > b.second;
            });
            
            

            // int16_t tremors_per_second = max({gx_freq, gy_freq, gz_freq}), tremors_sum;
            // float32_t tremor_magnitude = max({gx_peak, gy_peak, gz_peak}), tremor_magnitude_sum;
            tremors_per_second = peak_results[0].first;
            tremor_magnitude = peak_results[0].second;

            if (tremors_per_second >= 3 && tremors_per_second <= 6) {
                ++count;
                tremors_sum += tremors_per_second;
                tremor_magnitude_sum += tremor_magnitude;
            }

            if (seconds == 10) {
                if (count >= threshold) {
                    uint8_t message_tremors[20];
                    uint8_t message_tremor_magnitude[20];

                    sprintf((char *)message_tremors, "Tremors: %4.0f/s", (float)tremors_sum / count);
                    sprintf((char *)message_tremor_magnitude, "Magnitude: %f", tremor_magnitude_sum / count);
                    lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"Pakinson Detected", CENTER_MODE);
                    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Average Data in 10s", CENTER_MODE);
                    lcd.DisplayStringAt(0, LINE(6), (uint8_t *)message_tremors, CENTER_MODE);
                    lcd.DisplayStringAt(0, LINE(7), (uint8_t *)message_tremor_magnitude, CENTER_MODE);
                } else {
                    lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"No Pakinson Detected", CENTER_MODE);
                }
                ThisThread::sleep_for(5s);
                lcd.Clear(LCD_COLOR_WHITE);
                seconds = 0;
                count = 0;
                tremors_sum = 0;
                tremor_magnitude_sum = 0;
            }
            buffer_index = 0;  // 清空缓冲区
        }
        ThisThread::sleep_for(20ms);
    }
}

