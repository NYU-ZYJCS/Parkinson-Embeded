// Course：ECE6483 – Real Time Embedded Systems
// Tremor Challenge Final Project
// Team Members: Yingjie Zhang, Yichen Zhang, Jiayu Li

// Import the required libraries
#include <mbed.h>
#include <cmath>
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "stm32f429xx.h"
#include <LCD_DISCO_F429ZI.h>

// Define the pins for the SPI communication

// LED2 indicates tremor detected
DigitalOut led2(LED2); 
// LED1 indicates no tremor detected
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

// Register for the X-axis low byte
#define OUT_X_L 0x28 

// Scaling factor for the gyroscope
#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f) 

// Cutoff frequency
#define CUTOFF_FREQUENCY 20 

// FFT size
#define FFT_SIZE 128

// Sampling rate 
#define SAMPLE_RATE 50 

 // Value of PI
#define PI 3.14159265358979323846

// LCD object
LCD_DISCO_F429ZI lcd; 

// Buffer size
const uint32_t BUFFER_SIZE = 50; 

// Time window for tremor detection, set to 10 seconds
const uint32_t time_window = 10; 

// Threshold for tremor detection, set to 5
const uint32_t threshold = 5; 

// Buffer index
uint32_t buffer_index = 0; 

// Time counter
uint32_t seconds = 0; 

// Instance structure for the floating-point RFFT function
arm_rfft_instance_f32 S; 

// Instance structure for the floating-point CFFT function
arm_cfft_radix4_instance_f32 S_CFFT; 

// Event flags for SPI communication
EventFlags flags; 

// Callback function for SPI communication
void spi_cb(int event) {
    flags.set(SPI_FLAG);
}

// Apply the Hanning window to avoid spectral leakage
void applyHanningWindow(float32_t *input, int size) {
    for (int i = 0; i < size; i++) {
        float window = 0.5 - 0.5 * cos(2 * M_PI * i / (size - 1));
        input[i] *= window;
    }
}

// Find the peak in the spectrum
pair<uint32_t, float32_t> findPeak(float32_t *buffer) {
    // Initialize the RFFT and CFFT structures
    arm_rfft_init_f32(&S, &S_CFFT, FFT_SIZE, 0, 1); 

    // Input and output buffers for the FFT
    float32_t fft_input[FFT_SIZE];
    float32_t fft_output[FFT_SIZE * 2];

    // Analyze the spectrum
    // Copy the buffer to the input buffer for FFT, if the buffer length is less than FFT_SIZE, pad zeros
    memcpy(fft_input, buffer, sizeof(float32_t) * BUFFER_SIZE);
    if (BUFFER_SIZE < FFT_SIZE) {
        memset(&fft_input[BUFFER_SIZE], 0, sizeof(float32_t) * (FFT_SIZE - BUFFER_SIZE));
    }

    // Apply the Hanning window
    applyHanningWindow(fft_input, FFT_SIZE);

    // Perform the FFT
    arm_rfft_f32(&S, fft_input, fft_output);

    // Deal with the FFT output
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float frequency = (float)i * SAMPLE_RATE / FFT_SIZE;
        if (frequency > CUTOFF_FREQUENCY) {
            fft_output[2 * i] = 0;     // put the real part to zero
            fft_output[2 * i + 1] = 0; // put the imaginary part to zero
        }
    }

    // Analysis of spectral power
    float32_t power[FFT_SIZE / 2];

    // Calculate the power of the complex numbers
    arm_cmplx_mag_squared_f32(fft_output, power, FFT_SIZE / 2);

    // Calculate the spectral energy
    float32_t spectral_energy = 0;
    for (uint32_t i = 0; i < FFT_SIZE / 2; i++) {
        spectral_energy += power[i];
    }

    // Calculate the magnitude of the complex numbers
    float32_t spectrum[FFT_SIZE / 2];
    arm_cmplx_mag_f32(fft_output, spectrum, FFT_SIZE / 2);

    // Find the peak in the spectrum
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
    float32_t gx_buffer[BUFFER_SIZE], gz_buffer[BUFFER_SIZE];

    // Initialize the buffer index
    uint32_t count = 0;

    // Initialize the tremor sum
    std::string Intensity;

    while (1) {
        // Raw data from the gyroscope
        int16_t raw_gx, raw_gy, raw_gz; 
        // Actual values from the gyroscope
        float gx, gy, gz; 
        // Variables for tremor detection
        int16_t tremor_per_second, tremor_sum; 
        float32_t tremor_magnitude, tremor_magnitude_sum;

        // Prepare to read the gyroscope values starting from OUT_X_L
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        // Perform the SPI transfer to read 6 bytes of data (for x, y, and z axes)
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Convert the received data into 16-bit integers for each axis
        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

        // Convert raw data to actual values using a scaling factor
        gx = ((float)raw_gx) * SCALING_FACTOR;
        gy = ((float)raw_gy) * SCALING_FACTOR;
        gz = ((float)raw_gz) * SCALING_FACTOR;

        // Display the gyroscope data on the LCD
        // Clear the LCD
        lcd.Clear(LCD_COLOR_WHITE);

        // Display the gyroscope data
        uint8_t message_countdown[20];

        // Display the countdown message
        sprintf((char *)message_countdown, "Detecting... %d", 10 - seconds);

        // Display the message on the LCD
        lcd.DisplayStringAt(0, LINE(4), message_countdown, CENTER_MODE);

        // Store the gyroscope data in the buffer
        gz_buffer[buffer_index] = gz; 

        // Increment the buffer index
        buffer_index++;

        // If the buffer is full, perform the FFT and clear the buffer
        if (buffer_index == BUFFER_SIZE) {
            // Increment the time counter
            seconds++;

            // Call findPeak and store the results
            std::pair<uint32_t, float32_t> gz_peak_result = findPeak(gz_buffer);

            // Store the tremor data
            tremor_per_second = round(gz_peak_result.first * ((float)SAMPLE_RATE / FFT_SIZE));
            tremor_magnitude = gz_peak_result.second;

            // Print the tremor data
            printf("Tremors: %d/s, Magnitude: %f\n", tremor_per_second, tremor_magnitude);

            // Check if tremor is detected
            // if the tremor frequency is between 2 and 15 Hz and the tremor magnitude is greater than 1000, it is considered a tremor
            if (tremor_per_second >= 2 && tremor_per_second <= 15 && tremor_magnitude >= 1000) {
                count++;
                tremor_sum += tremor_per_second;
                tremor_magnitude_sum += tremor_magnitude;
            }

            // If the time window is reached, display the results
            if (seconds == 10) {
                // Calculate the average tremor frequency and magnitude
                float32_t tremors_avg = (float)tremor_sum / count;
                float32_t tremor_magnitude_avg = tremor_magnitude_sum / count;

                // If the count is greater than the threshold, display the tremor intensity
                if (count >= threshold) {
                    uint8_t message_tremors[20]; // Message for tremor frequency
                    uint8_t message_tremor_magnitude[20]; // Message for tremor magnitude

                    led2 = 1; // Turn on LED2
                    led1 = 0; // Turn off LED1

                    // Determine the intensity of the tremor
                    // if the tremor magnitude is less than 5000, the intensity is low
                    if (tremor_magnitude_avg < 5000) {
                        Intensity = "LOW";
                    } 
                    // if the tremor magnitude is between 5000 and 15000, the intensity is medium
                    else if (5000 <= tremor_magnitude_avg && tremor_magnitude_avg < 15000) {
                        Intensity = "MEDIUM";
                    } 
                    // if the tremor magnitude is greater than 15000, the intensity is high
                    else if (tremor_magnitude_avg >= 15000) {
                        Intensity = "HIGH";
                    }

                    // Display the tremor result on the LCD
                    sprintf((char *)message_tremors, "Tremors: %4.0fHZ/s", tremors_avg);
                    sprintf((char *)message_tremor_magnitude, "Intensity: %s", Intensity.c_str()); 

                    lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"Tremor Detected", CENTER_MODE); 
                    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Average Data in 10s", CENTER_MODE);
                    lcd.DisplayStringAt(0, LINE(6), (uint8_t *)message_tremors, CENTER_MODE);
                    lcd.DisplayStringAt(0, LINE(7), (uint8_t *)message_tremor_magnitude, CENTER_MODE);
                }
                else {
                    // If the count is less than the threshold, display the message "No Tremor Detected"
                    lcd.DisplayStringAt(0, LINE(4), (uint8_t *)"No Tremor Detected", CENTER_MODE);

                    led1 = 1; // Turn on LED1
                    led2 = 0; // Turn off LED2
                }
                // Wait for 5 seconds
                ThisThread::sleep_for(5s);

                // Clear the LCD
                lcd.Clear(LCD_COLOR_WHITE);

                // Reset the variables
                seconds = 0;
                count = 0;
                tremor_sum = 0;
                tremor_magnitude_sum = 0;
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
}