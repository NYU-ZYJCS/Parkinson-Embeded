#include "mbed.h"
#include "arm_math.h"  // Include CMSIS-DSP header

#define FFT_SIZE 256  // Must be a power of two

float32_t input[FFT_SIZE];  // Input array for FFT
float32_t output[FFT_SIZE];  // Output array for FFT
arm_rfft_fast_instance_f32 S;  // FFT instance

int main() {
    // Initialize the FFT instance
    arm_rfft_fast_init_f32(&S, FFT_SIZE);

    // Prepare input data: a mix of two sinusoids
    for (int i = 0; i < FFT_SIZE; i++) {
        input[i] = 0.125f * arm_sin_f32(2 * PI * 50.0f * i / FFT_SIZE) +
                   0.50f * arm_sin_f32(2 * PI * 120.0f * i / FFT_SIZE);
    }

    // Perform the FFT
    arm_rfft_fast_f32(&S, input, output, 0);

    // Print the FFT output results
    printf("FFT Output:\n");
    for (int i = 0; i < FFT_SIZE; i++) {
        printf("%d: %f\n", i, output[i]);
    }

    while (true) {
        // Loop forever
    }
}
