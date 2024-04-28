#include <mbed.h>
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "stm32f429xx.h"
#include <LCD_DISCO_F429ZI.h>

// TODOs:
// [1] Get started with an SPI object instance and connect to the Gyroscope!
// [2] Read the XYZ axis from the Gyroscope and Visualize on the Teleplot. 
// [3] Fetching Data from the sensor via Polling vs Interrupt ?

// Define control register addresses and their configurations
#define TABLE_SIZE 500

#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define SPI_FLAG 1
#define OUT_X_L 0x28

#define BUFFER_SIZE 100
#define ACCEL_THRESHOLD 0.01f  // 加速度阈值,单位为°/s^2
#define WINDOW_SIZE 500 // 时间窗口大小,单位为采样周期(假设采样频率为100Hz)
#define TREMOR_COUNT 5 // 判定存在震颤所需的快速变化次数

#define INTENSITY_THRESHOLD_LOW 0.5f
#define INTENSITY_THRESHOLD_MEDIUM 1.0f
#define INTENSITY_THRESHOLD_HIGH 1.5f

float gx_buffer[BUFFER_SIZE], gy_buffer[BUFFER_SIZE], gz_buffer[BUFFER_SIZE];
int buffer_index = 0;
int tremor_counter = 0;
LCD_DISCO_F429ZI lcd;


void lcd_x_y_z(int16_t gx, int16_t gy, int16_t gz)
{
  uint8_t message_gx[20]; // 分配足够的数组大小
  uint8_t message_gy[20];
  uint8_t message_gz[20];

  // 分别格式化每个轴的数据
  sprintf((char *)message_gx, "GYRO_X: %6d", gx);
  sprintf((char *)message_gy, "GYRO_Y: %6d", gy);
  sprintf((char *)message_gz, "GYRO_Z: %6d", gz);

  // 分别在不同的行显示每个轴的数据
  lcd.DisplayStringAt(0, LINE(6), (uint8_t *)&message_gx, CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(7), (uint8_t *)&message_gy, CENTER_MODE);
  lcd.DisplayStringAt(0, LINE(8), (uint8_t *)&message_gz, CENTER_MODE);
}



float calculateIntensity() {
  float sum = 0;
  int count = 0;

  for (int i = 0; i < BUFFER_SIZE; i++) {
    int prev_index = (i - 1 + BUFFER_SIZE) % BUFFER_SIZE;
    float ax = gx_buffer[i] - gx_buffer[prev_index];
    float ay = gy_buffer[i] - gy_buffer[prev_index];
    float az = gz_buffer[i] - gz_buffer[prev_index];

    sum += fabs(ax) + fabs(ay) + fabs(az);
    count++;
  }

  return sum / (count * 3);
}


// 假设绿色LED连接到PA_5,黄色LED连接到PA_6,红色LED连接到PA_7
DigitalOut ledGreen(PA_5);
DigitalOut ledYellow(PA_6);
DigitalOut ledRed(PA_7);

void indicateTremor(float intensity) {
  if (intensity < INTENSITY_THRESHOLD_LOW) {
    // 震颤强度低,点亮绿色LED
    ledGreen = 1;
    ledYellow = 0;
    ledRed = 0;
  } else if (intensity < INTENSITY_THRESHOLD_MEDIUM) {
    // 震颤强度中等,点亮黄色LED
    ledGreen = 0;
    ledYellow = 1;
    ledRed = 0;
  } else {
    // 震颤强度高,点亮红色LED
    ledGreen = 0;
    ledYellow = 0;
    ledRed = 1;
  }
}

void detectTremor(float gx, float gy, float gz) {
  // 将新数据添加到缓冲区
  gx_buffer[buffer_index] = gx;
  gy_buffer[buffer_index] = gy;
  gz_buffer[buffer_index] = gz;

  // 计算加速度
  float ax = 0, ay = 0, az = 0;
  if (buffer_index > 0) {
    ax = gx - gx_buffer[(buffer_index - 1 + BUFFER_SIZE) % BUFFER_SIZE];
    ay = gy - gy_buffer[(buffer_index - 1 + BUFFER_SIZE) % BUFFER_SIZE];
    az = gz - gz_buffer[(buffer_index - 1 + BUFFER_SIZE) % BUFFER_SIZE];
  }

  buffer_index = (buffer_index + 1) % BUFFER_SIZE;

  // 检查是否存在快速变化
  if (fabs(ax) > ACCEL_THRESHOLD || fabs(ay) > ACCEL_THRESHOLD || fabs(az) > ACCEL_THRESHOLD) {
    tremor_counter++;
  }

  // 判断是否存在震颤
  if (tremor_counter >= TREMOR_COUNT) {
    // 计算震颤强度
    float intensity = calculateIntensity();
    // 提供视觉指示
    indicateTremor(intensity);
    tremor_counter = 0;
  }

  // 重置计数器
  if (tremor_counter >= WINDOW_SIZE) {
    tremor_counter = 0;
  }
}

EventFlags flags;

void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}

#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)


int main()
{
    // Initialize the SPI object with specific pins.
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);

    // Initialize the LCD
    BSP_LCD_SetFont(&Font20);
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_BLUE);

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

        uint16_t raw_gx, raw_gy, raw_gz;
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
        // printf("RAW -> \t\tgx: %d \t gy: %d \t gz: %d \t\n", raw_gx, raw_gy, raw_gz);

            // printf(">x_axis: %d|g \n", raw_gx);
            // printf(">y_axis: %d|g \n", raw_gy);
            // printf(">z_axis: %d|g \n", raw_gz);

        // Convert raw data to actual values using a scaling factor
        gx = ((float) raw_gx) * SCALING_FACTOR;
        gy = ((float) raw_gy) * SCALING_FACTOR;
        gz = ((float) raw_gz) * SCALING_FACTOR;

        lcd_x_y_z(gx, gy, gz);

        // Print the actual values
        printf("Actual -> \t\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f \t\n", gx, gy, gz);
        
        // Detect tremor
        detectTremor(gx, gy, gz);
    
        // Delay for a short period (e.g., 10ms for 100Hz sampling rate)
        ThisThread::sleep_for(10ms);
        
        

    }
}



