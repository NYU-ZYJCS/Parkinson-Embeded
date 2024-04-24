#include <stm32f429xx.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>
#include <stm32f4xx_hal_spi.h>
#include <stm32f4xx_hal_uart.h>
#include <stdio.h>
#include <KalmanFilter.h>
#include "mbed.h"


SPI_HandleTypeDef hspi5;

uint8_t txBuf[2], rxBuf[2];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI5_Init(void);
void L3GD20_Write(uint8_t reg, uint8_t val);
uint8_t L3GD20_Read(uint8_t reg);
void L3GD20_Init(void);

int main(void)
{
  
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI5_Init();
  L3GD20_Init();

  KalmanFilter kalmanX(0.05, 0.001);
  KalmanFilter kalmanY(0.05, 0.001);
  KalmanFilter kalmanZ(0.05, 0.001);

  float gx_raw, gy_raw, gz_raw;
  float gx_filtered = 0, gy_filtered = 0, gz_filtered = 0;
  const float alpha = 0.5;
  float dt = 0.02;  // 假设采样间隔为20ms

  // float gx_bias = 0, gy_bias = 0, gz_bias = 0;
  // for (int i = 0; i < 100; i++) {
  //   float gx = (L3GD20_Read(0x29) << 8) | L3GD20_Read(0x28);
  //   float gy = (L3GD20_Read(0x2B) << 8) | L3GD20_Read(0x2A);
  //   float gz = (L3GD20_Read(0x2D) << 8) | L3GD20_Read(0x2C);
  //   gx_bias = alpha * gx + (1 - alpha) * gx_bias;
  //   gy_bias = alpha * gy + (1 - alpha) * gy_bias;
  //   gz_bias = alpha * gz + (1 - alpha) * gz_bias;
  //   HAL_Delay(10);
  // }

  const int num_samples = 100;
  float gx_samples[num_samples], gy_samples[num_samples], gz_samples[num_samples];

  for (int i = 0; i < num_samples; i++) {
    gx_samples[i] = (float)((L3GD20_Read(0x29) << 8) | L3GD20_Read(0x28));
    gy_samples[i] = (float)((L3GD20_Read(0x2B) << 8) | L3GD20_Read(0x2A));
    gz_samples[i] = (float)((L3GD20_Read(0x2D) << 8) | L3GD20_Read(0x2C));
    HAL_Delay(10);
  }

  std::sort(gx_samples, gx_samples + num_samples);
  std::sort(gy_samples, gy_samples + num_samples);
  std::sort(gz_samples, gz_samples + num_samples);

  float gx_bias = gx_samples[num_samples / 2];
  float gy_bias = gy_samples[num_samples / 2];
  float gz_bias = gz_samples[num_samples / 2];

  
  int loop_count = 0;

  while (1)
  {
    
    if (loop_count % 100 == 0)
        {
            // 每100次循环重新估计一次偏置
            gx_bias = 0;
            gy_bias = 0;
            gz_bias = 0;

            for (int i = 0; i < 100; i++)
            {
                float gx = (float)((L3GD20_Read(0x29) << 8) | L3GD20_Read(0x28));
                float gy = (float)((L3GD20_Read(0x2B) << 8) | L3GD20_Read(0x2A));
                float gz = (float)((L3GD20_Read(0x2D) << 8) | L3GD20_Read(0x2C));

                gx_bias = alpha * gx + (1 - alpha) * gx_bias;
                gy_bias = alpha * gy + (1 - alpha) * gy_bias;
                gz_bias = alpha * gz + (1 - alpha) * gz_bias;

                HAL_Delay(10);
            }
        }
    
    float gx = (float)((L3GD20_Read(0x29) << 8) | L3GD20_Read(0x28)) - gx_bias;
    float gy = (float)((L3GD20_Read(0x2B) << 8) | L3GD20_Read(0x2A)) - gy_bias;
    float gz = (float)((L3GD20_Read(0x2D) << 8) | L3GD20_Read(0x2C)) - gz_bias;
    

    

    gx_raw = gx * 0.07 * 3.14159 / 180;
    gy_raw = gy * 0.07 * 3.14159 / 180;
    gz_raw = gz * 0.07 * 3.14159 / 180;

    gx_filtered = kalmanX.update(gx_filtered, gx_raw, dt);
    gy_filtered = kalmanY.update(gy_filtered, gy_raw, dt);
    gz_filtered = kalmanZ.update(gz_filtered, gz_raw, dt);

    HAL_Delay(200);
    printf("Gyro data: gx = %.2f rad/s, gy = %.2f rad/s, gz = %.2f rad/s\n", gx_filtered, gy_filtered, gz_filtered);
    loop_count++;
    
    
    
  }
    
}




void L3GD20_Write(uint8_t reg, uint8_t val)
{
  txBuf[0] = reg;
  txBuf[1] = val;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi5, txBuf, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
}

uint8_t L3GD20_Read(uint8_t reg)
{
  txBuf[0] = reg | 0x80;
  txBuf[1] = 0x00;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi5, txBuf, rxBuf, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
  return rxBuf[1];
}

void L3GD20_Init(void)
{
  L3GD20_Write(0x20, 0x0F);  // Enable all axes, normal mode, 95Hz
  L3GD20_Write(0x23, 0x20);  // 2000 dps full scale
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  
  /* USER CODE END Error_Handler_Debug */
}

static void MX_SPI5_Init(void)
{
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
    {
        Error_Handler();
    }

}



static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void SystemClock_Config(void)
{
  // ...
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI5)
  {
    __HAL_RCC_SPI5_CLK_ENABLE();

    __HAL_RCC_GPIOF_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  }
}