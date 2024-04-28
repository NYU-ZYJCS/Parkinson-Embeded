#include "arm_math.h"
#include <cmath>
#include <mbed.h>

#define FFT_SIZE 256
#define SAMPLING_FREQ 100.0f // 假设采样频率为100Hz,请根据实际情况调整

void spectral_analysis(float32_t* gx, float32_t* gy, float32_t* gz, uint32_t size) {
    float32_t input[FFT_SIZE];
    float32_t output[FFT_SIZE];
    float32_t psd[FFT_SIZE/2];

    // 执行FFT和计算PSD
    arm_rfft_fast_instance_f32 fft_instance;
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);

    for (uint32_t axis = 0; axis < 3; axis++) {
        float32_t* data = (axis == 0) ? gx : (axis == 1) ? gy : gz;

        // 将数据拷贝到输入缓冲区
        for (uint32_t i = 0; i < size; i++) {
            input[i] = data[i];
        }
        // 如果数据长度小于FFT_SIZE,用0填充剩余部分
        for (uint32_t i = size; i < FFT_SIZE; i++) {
            input[i] = 0;
        }

        arm_rfft_fast_f32(&fft_instance, input, output, 0);
        arm_cmplx_mag_squared_f32(output, psd, FFT_SIZE/2);

        // 找到主导频率
        float32_t max_psd = 0;
        uint32_t max_idx = 0;
        for (uint32_t i = 0; i < FFT_SIZE/2; i++) {
            if (psd[i] > max_psd) {
                max_psd = psd[i];
                max_idx = i;
            }
        }
        float32_t dominant_freq = (float32_t)max_idx / FFT_SIZE * SAMPLING_FREQ;

        // 计算PSD的总和
        float32_t psd_sum = 0;
        for (uint32_t i = 0; i < FFT_SIZE/2; i++) {
            psd_sum += psd[i];
        }

        // 归一化PSD并计算频谱熵
        float32_t spectral_entropy = 0;
        for (uint32_t i = 0; i < FFT_SIZE/2; i++) {
            float32_t normalized_psd = psd[i] / psd_sum;
            if (normalized_psd > 0) {
                spectral_entropy -= normalized_psd * log2f(normalized_psd);
            }
        }

        // 打印结果
        char axis_name = (axis == 0) ? 'X' : (axis == 1) ? 'Y' : 'Z';
        printf("Axis %c:\n", axis_name);
        printf("Dominant frequency: %.2f Hz\n", dominant_freq);
        printf("Spectral entropy: %.4f\n", spectral_entropy);
        printf("\n");
    }
}

void time_domain_features(float32_t* data, uint32_t size, float32_t* mean, float32_t* variance, float32_t* rms, float32_t* peak_to_peak) {
    // 计算均值
    float32_t sum = 0;
    for (uint32_t i = 0; i < size; i++) {
        sum += data[i];
    }
    *mean = sum / size;

    // 计算方差
    float32_t sum_squared_diff = 0;
    for (uint32_t i = 0; i < size; i++) {
        float32_t diff = data[i] - *mean;
        sum_squared_diff += diff * diff;
    }
    *variance = sum_squared_diff / size;

    // 计算均方根值(RMS)
    float32_t sum_squares = 0;
    for (uint32_t i = 0; i < size; i++) {
        sum_squares += data[i] * data[i];
    }
    *rms = sqrtf(sum_squares / size);

    // 计算峰峰值
    float32_t min_val = data[0], max_val = data[0];
    for (uint32_t i = 1; i < size; i++) {
        if (data[i] < min_val) {
            min_val = data[i];
        } else if (data[i] > max_val) {
            max_val = data[i];
        }
    }
    *peak_to_peak = max_val - min_val;
}
/*

时域特征可以为帕金森症的检测提供有价值的信息。让我解释一下每个特征的意义和作用:

均值(Mean):
均值表示数据的中心趋势。对于帕金森症患者,静止性震颤可能导致陀螺仪数据偏离零点。因此,与健康人相比,帕金森症患者的陀螺仪数据的均值可能更远离零。

方差(Variance):
方差衡量数据相对于均值的分散程度。帕金森症患者的震颤可能导致陀螺仪数据的方差增大,因为数据点会在均值周围波动得更剧烈。
均方根值(RMS):
均方根值是信号振幅的度量。对于帕金森症患者,由于静止性震颤,陀螺仪数据的RMS值可能会显著高于健康人。

峰峰值(Peak-to-peak):
峰峰值表示信号的动态范围,即最大值和最小值之间的差值。帕金森症患者的震颤可能导致陀螺仪数据的峰峰值增大,因为数据点在更大的范围内波动。

通过比较这些时域特征,我们可以区分帕金森症患者和健康人的陀螺仪数据。例如,如果一个人的陀螺仪数据具有较大的均值偏移、较高的方差、较大的RMS值和较大的峰峰值,那么他可能患有帕金森症。
*/