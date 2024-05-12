/*
    Course：ECE6483 – Real Time Embedded Systems
    Tremor Challenge Final Project
    Demo: https://youtu.be/h497cOOJKGI
    Team 17 Members: Yingjie Zhang, Yichen Zhang, Jiayu Li
    
NetID:yz10759: NAME :Yichen Zhang:
    1.Hardware setup and interface:
        Connect STM32F429I-DISC1 development board and L3GD20 gyroscope.
        Configure SPI interface and write functions for reading and writing gyroscope registers.
    2.Data acquisition and preprocessing:
        Write functions for reading raw data from the gyroscope.
        Implement data calibration to convert raw data to angular velocity values.
        Implement Kalman filter for filtering angular velocity data.
    3.Tremor detection algorithm:
        Study the characteristics of Parkinson's disease tremors and determine the frequency and amplitude thresholds for detecting tremors.
        Implement tremor detection function to determine if tremors are detected and the direction of the tremors.
        Optimize the tremor detection algorithm to improve detection accuracy and reliability.

NetID: yz10617, NAME: Yingjie Zhang:
    1.FFT analysis:
        Study and understand the FFT algorithm.
        Implement FFT analysis function, including data buffering, Hanning window, etc.
        Optimize FFT function.
    2.Result display:
        Design the interface for displaying results, including tremor frequency, direction, intensity, etc.
        Implement functions for displaying detection results on the LCD.
        Optimize display effects, such as reasonable layout and clear readability.
    3.Main loop and task scheduling:
        Design and implement the main loop structure.
        Call data acquisition, tremor detection, and result display functions in the main loop.
        Manage timing and task scheduling, update detection results every 10 seconds.

NetID: jl14882, NAME: Jiayu Li:
    1.System integration and testing:
        Integrate the code from Yingjie and Yichen to form a complete system.
        Design and execute test plans to verify the functionality and performance of the system.
        Identify and fix errors in the code based on test results.
    2.Demonstration:
        Prepare project demonstrations, including demonstration videos, etc.
        Summarize feedback received during the demonstration process and improve the system accordingly.
    3.Code optimization:
        Review the code and identify areas for optimization, such as magic numbers, duplicate code, etc.
        Optimize the code to improve code readability, maintainability, and efficiency.
        Refactor the code as necessary to make it more modular and structured.
*/


#include <mbed.h>
#include <cmath>
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "stm32f429xx.h"
#include "KalmanFilter.h"
#include <LCD_DISCO_F429ZI.h>

// apply Kalman filter to the gyroscope data
KalmanFilter kf_x(0.5, 0.005);
KalmanFilter kf_y(0.1, 0.005);
KalmanFilter kf_z(0.5, 0.005);

// LED1, LED2 indicates tremor detected
DigitalOut led2(LED2);
DigitalOut led1(LED1);

// Define control register addresses and their configurations

// Control register 1
#define CTRL_REG1 0x20 

// Control register 4
#define CTRL_REG4 0x23

// Configuration for control register 1
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1

// Configuration for control register 4
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0

// SPI flag
#define SPI_FLAG 1

// Output register addresses
#define OUT_X_L 0x28

// Scaling factor for converting raw data to actual values
#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)

// Cutoff frequency for the low-pass filter
#define CUTOFF_FREQUENCY 20

// FFT parameters
#define FFT_SIZE 128

// Sampling frequency
#define SAMPLE_RATE 50

// PI
#define PI 3.14159265358979323846

// Tremor magnitude thresholds
const float32_t TREMOR_MAGNITUDE_LOW_Z = 2500;
const float32_t TREMOR_MAGNITUDE_MEDIUM_Z = 5000;
const float32_t TREMOR_MAGNITUDE_HIGH_Z = 9000;

const float32_t TREMOR_MAGNITUDE_LOW_X = 2000;
const float32_t TREMOR_MAGNITUDE_MEDIUM_X = 4000;
const float32_t TREMOR_MAGNITUDE_HIGH_X = 8000;

// LCD display
LCD_DISCO_F429ZI lcd;

// Buffer size for FFT
const uint32_t BUFFER_SIZE = 50;

// Threshold for tremor detection
const uint32_t time_window = 10;

// Threshold for tremor detection
const uint32_t threshold = 5;

// Buffer index
uint32_t buffer_index = 0;

// Seconds
uint32_t seconds = 0;

// FFT instance
arm_rfft_instance_f32 S;
arm_cfft_radix4_instance_f32 S_CFFT;

// Event flags for SPI communication
EventFlags flags; 

// SPI callback function
void spi_cb(int event) {
    flags.set(SPI_FLAG);
}

/*
    * @brief apply Hanning window to the input signal
    * @param input: input signal
    * @param size: size of the input signal
*/
void applyHanningWindow(float32_t *input, int size) {
    for (int i = 0; i < size; i++) {
        float window = 0.5 - 0.5 * cos(2 * M_PI * i / (size - 1));
        input[i] *= window;
    }
}

/*
    * @brief from the input signal, find the peak of the spectrum
    * @param buffer: input signal
    * @return peak_index: the index of the peak in the spectrum
*/
pair<uint32_t, float32_t> findPeak(float32_t *buffer) {
    // Initialize the RFFT and CFFT structures
    arm_rfft_init_f32(&S, &S_CFFT, FFT_SIZE, 0, 1); 

    // Input and output buffers for the FFT
    float32_t fft_input[FFT_SIZE];
    float32_t fft_output[FFT_SIZE * 2];


    // spectrum analysis
    // if the buffer size is smaller than FFT_SIZE, fill the rest with zeros
    memcpy(fft_input, buffer, sizeof(float32_t) * BUFFER_SIZE);
    if (BUFFER_SIZE < FFT_SIZE) {
        memset(&fft_input[BUFFER_SIZE], 0, sizeof(float32_t) * (FFT_SIZE - BUFFER_SIZE));
    }

    // apply Hanning window for decreasing the spectral leakage
    applyHanningWindow(fft_input, FFT_SIZE);

    // Perform the FFT
    arm_rfft_f32(&S, fft_input, fft_output);

    // a low-pass filter to remove high frequency noise
    // if the frequency is higher than the cutoff frequency, set the real and imaginary part to zero
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float frequency = (float)i * SAMPLE_RATE / FFT_SIZE;
        if (frequency > CUTOFF_FREQUENCY) {
            fft_output[2*i] = 0;
            fft_output[2*i + 1] = 0;
        }
    }  

    // spectral power
    float32_t power[FFT_SIZE / 2];

    // Calculate the power of the complex numbers
    arm_cmplx_mag_squared_f32(fft_output, power, FFT_SIZE / 2);

    // Calculate the spectral energy
    float32_t spectral_energy = 0;
    for (uint32_t i = 0; i < FFT_SIZE / 2; i++) {
        spectral_energy += power[i];
    } 

    // spectral magnitude
    float32_t spectrum[FFT_SIZE / 2];
    arm_cmplx_mag_f32(fft_output, spectrum, FFT_SIZE / 2);

    // find the peak of the spectrum
    float32_t peak = 0;
    uint32_t peak_index = 0;
    arm_max_f32(spectrum, FFT_SIZE / 2, &peak, &peak_index);

    // Return the peak index and spectral energy
    return {peak_index, spectral_energy};
}

int main()
{
    // Initialize the SPI object with specific pins
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);

    // Buffers for sending and receiving data over SPI
    uint8_t write_buf[32], read_buf[32];

    // Configure SPI format and frequency
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Configure CTRL_REG1 register
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    // Configure CTRL_REG4 register
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    // Buffers for storing the gyroscope data
    float32_t gx_buffer[BUFFER_SIZE], gz_buffer[BUFFER_SIZE], gy_buffer[BUFFER_SIZE];
    uint32_t count_x = 0;
    uint32_t count_z = 0;

    // Intensity of the tremor
    std::string Intensity;  
    
    while(1){
        // Raw data from the gyroscope
        int16_t raw_gx, raw_gy, raw_gz;
        float gx, gy, gz;
        
        // Variables for storing the tremor data
        int16_t tremor_sum_x, tremor_per_second_x;
        int16_t tremor_sum_z, tremor_per_second_z;
        
        // Variables for storing the tremor magnitude data
        float32_t tremor_magnitude_x, tremor_magnitude_sum_x;
        float32_t tremor_magnitude_z, tremor_magnitude_sum_z;

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

        // Clear the LCD screen
        lcd.Clear(LCD_COLOR_WHITE);

        // Display the gyroscope data
        uint8_t message_countdown[20];

        // Display the countdown message
        sprintf((char *)message_countdown, "Detecting... %d", 10 - seconds);

        // Display the message on the LCD
        lcd.DisplayStringAt(0, LINE(4), message_countdown, CENTER_MODE);


        // sampling time interval
        float dt = 0.05;

        // Kalman filter
        float32_t gx_filtered = kf_x.update(gx, 0, dt);
        float32_t gy_filtered = kf_y.update(gy, 0, dt);
        float32_t gz_filtered = kf_z.update(gz, 0, dt);
        
        // Store the filtered data in the buffer
        gx_buffer[buffer_index] = gx_filtered;
        gz_buffer[buffer_index] = gz_filtered;
        gy_buffer[buffer_index] = gy_filtered;
        buffer_index++;

        // If the buffer is full, perform FFT
        if (buffer_index == BUFFER_SIZE) {
            // Increment the time counter
            seconds++;

            // Call findPeak and store the results
            std::pair<uint32_t, float32_t> gx_peak_result = findPeak(gx_buffer);
            std::pair<uint32_t, float32_t> gz_peak_result = findPeak(gz_buffer);

            // calculate the tremor per second
            tremor_per_second_x = round(gx_peak_result.first * ((float)SAMPLE_RATE / FFT_SIZE));
            tremor_per_second_z = round(gz_peak_result.first * ((float)SAMPLE_RATE / FFT_SIZE));

            // get tremor magnitude
            tremor_magnitude_x = gx_peak_result.second;
            tremor_magnitude_z = gz_peak_result.second;

            // Print the tremor data
            printf("Tremors Vertical: %d/s, Magnitude X: %f\n", tremor_per_second_x, tremor_magnitude_x);
            printf("Tremors Horizontal: %d/s, Magnitude Z: %f\n", tremor_per_second_z, tremor_magnitude_z);
            printf("\n");

            // Check if the tremor data is within the specified range
            if (tremor_per_second_x >= 2 && tremor_per_second_x <= 15 && tremor_magnitude_x >= 1000) {
                count_x++;
                tremor_sum_x += tremor_per_second_x;
                tremor_magnitude_sum_x += tremor_magnitude_x;
            }

            // Check if the tremor data is within the specified range
            if (tremor_per_second_z >= 2 && tremor_per_second_z <= 15 && tremor_magnitude_z >= 1000) {
                count_z++;
                tremor_sum_z += tremor_per_second_z;
                tremor_magnitude_sum_z += tremor_magnitude_z;
            }
            
            // If 10 seconds have passed, display the results
            if (seconds == 10) {
                // Calculate the average tremor data
                float32_t tremors_avg_x = (float)tremor_sum_x / count_x;
                float32_t tremors_avg_z = (float)tremor_sum_z / count_z;

                // Calculate the average tremor magnitude
                float32_t tremor_magnitude_avg_x = tremor_magnitude_sum_x / count_x;
                float32_t tremor_magnitude_avg_z = tremor_magnitude_sum_z / count_z;
                
                // if the horizontal tremor is detected
                if (count_z >= threshold) {
                    uint8_t message_tremors[20];
                    uint8_t message_tremor_magnitude[20];

                    led2 = 1; // Turn on the LED2
                    led1 = 0; // Turn off the LED1
                    
                    // Determine the intensity of the tremor
                    if (tremor_magnitude_avg_z < TREMOR_MAGNITUDE_LOW_Z) {
                        Intensity = "Mild";
                    } 
                    else if (TREMOR_MAGNITUDE_LOW_Z < tremor_magnitude_avg_z && tremor_magnitude_avg_z < TREMOR_MAGNITUDE_MEDIUM_Z) {
                        Intensity = "Moderate";
                    } 
                    else if (TREMOR_MAGNITUDE_MEDIUM_Z < tremor_magnitude_avg_z && tremor_magnitude_avg_z < TREMOR_MAGNITUDE_HIGH_Z) {
                        Intensity = "Severe";
                    } 
                    else if (tremor_magnitude_avg_z > TREMOR_MAGNITUDE_HIGH_Z) {
                        Intensity = "Extremely Severe";
                    }
                    
                    // Display the tremor data on the LCD
                    sprintf((char *)message_tremors, "Tremor :%4.0f /s", tremors_avg_z);
                    sprintf((char *)message_tremor_magnitude, "%s", Intensity.c_str());
                    lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"Tremor Detected", CENTER_MODE);
                    lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Average Data in 10s", CENTER_MODE);
                    lcd.DisplayStringAt(0, LINE(8), (uint8_t *)"Horizontal Tremor", CENTER_MODE);
                    lcd.DisplayStringAt(0, LINE(10), (uint8_t *)message_tremors, CENTER_MODE);
                    lcd.DisplayStringAt(0, LINE(12), (uint8_t *)"Intensity:", CENTER_MODE);
                    lcd.DisplayStringAt(0, LINE(14), (uint8_t *)message_tremor_magnitude, CENTER_MODE);
                    
                } 
                // if the vertical tremor is detected
                else if (count_x >= threshold) {
                    uint8_t message_tremors[20];
                    uint8_t message_tremor_magnitude[20];

                    led2 = 1; // Turn on the LED2
                    led1 = 0; // Turn off the LED1
                    
                    // Determine the intensity of the tremor
                    if (tremor_magnitude_avg_x < TREMOR_MAGNITUDE_LOW_X) {
                        Intensity = "Mild";
                    } 
                    else if (TREMOR_MAGNITUDE_LOW_X < tremor_magnitude_avg_x && tremor_magnitude_avg_x < TREMOR_MAGNITUDE_MEDIUM_X) {
                        Intensity = "Moderate";
                    } 
                    else if (TREMOR_MAGNITUDE_MEDIUM_X < tremor_magnitude_avg_x && tremor_magnitude_avg_x < TREMOR_MAGNITUDE_HIGH_X) {
                        Intensity = "Severe";
                    } 
                    else if (tremor_magnitude_avg_x > TREMOR_MAGNITUDE_HIGH_X) {
                        Intensity = "Extremely Severe";
                    }                
                    
                    // Display the tremor data on the LCD
                    sprintf((char *)message_tremors, "Tremor :%4.0f /s", tremors_avg_x);
                    sprintf((char *)message_tremor_magnitude, "%s", Intensity.c_str());
                    lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"Tremor Detected", CENTER_MODE);
                    lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"Average Data in 10s", CENTER_MODE);
                    lcd.DisplayStringAt(0, LINE(8), (uint8_t *)"Vertical Tremor", CENTER_MODE);
                    lcd.DisplayStringAt(0, LINE(10), (uint8_t *)message_tremors, CENTER_MODE);
                    lcd.DisplayStringAt(0, LINE(12), (uint8_t *)"Intensity:", CENTER_MODE);
                    lcd.DisplayStringAt(0, LINE(14), (uint8_t *)message_tremor_magnitude, CENTER_MODE);
                }
                // if no tremor is detected
                else {
                    // If the count is less than the threshold, display the message "No Tremor Detected"
                    lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"No Tremor Detected", CENTER_MODE);
                    led1 = 1; // Turn on the LED1
                    led2 = 0; // Turn off the LED2
                }
                // Wait for 5 seconds
                // Wait for 5 seconds
                ThisThread::sleep_for(5s);

                // Clear the LCD
                lcd.Clear(LCD_COLOR_WHITE);

                // Reset the variables

                // Reset the variables
                seconds = 0;
                count_x = 0;
                count_z = 0;
                tremor_sum_x = 0;
                tremor_sum_z = 0;
                tremor_magnitude_sum_x = 0;
                tremor_magnitude_sum_z = 0;
            }
            // Reset the buffer index
            buffer_index = 0;
        }
        
        // Wait for 20ms
        ThisThread::sleep_for(20ms);

        // Turn off the LEDs
        led1 = 0;  
        led2 = 0;
    }
    return 0;
}