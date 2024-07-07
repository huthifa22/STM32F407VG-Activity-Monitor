#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "LiquidCrystal_I2C.h"
#include "bno055.h"
#include <math.h>

// Thresholds
#define TEMP_CHANGE_THRESHOLD 0.2
#define SPEED_CHANGE_THRESHOLD 0.01
#define ACCEL_CHANGE_THRESHOLD 0.2

void SystemClock_Config(void);
void Error_Handler(char *errorMessage, int lcdNumber);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

I2C_HandleTypeDef i2c;
LiquidCrystal_I2C_HandleTypeDef lcd, lcd2;

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    // Initialize LCD1 at 0x27
    LiquidCrystal_I2C_Init(&lcd, &i2c, 0x27, 20, 4);
    LiquidCrystal_I2C_Begin(&lcd, 20, 4, LCD_5x8DOTS);

    // Initialize LCD2 at 0x20
    LiquidCrystal_I2C_Init(&lcd2, &i2c, 0x20, 20, 4);
    LiquidCrystal_I2C_Begin(&lcd2, 20, 4, LCD_5x8DOTS);

    // Initialize BNO055
    bno055_setup();
    bno055_setOperationModeNDOF();

    char buffer[20];
    bno055_vector_t accel, mag, gyro, euler, quaternion, linear_accel, gravity;
    int8_t temp_raw;
    float temperature_c, temperature_f;
    float prev_temperature_f = 0.0, prev_speed = 0.0, prev_linear_accel_x = 0.0;

    while (1) {

        // Get Sensor Data
        accel = bno055_getVectorAccelerometer();
        mag = bno055_getVectorMagnetometer();
        gyro = bno055_getVectorGyroscope();
        euler = bno055_getVectorEuler();
        quaternion = bno055_getVectorQuaternion();
        linear_accel = bno055_getVectorLinearAccel();
        gravity = bno055_getVectorGravity();
        temp_raw = bno055_getTemp();

        // Convert to fahrenheit
        temperature_c = (float)temp_raw;
        temperature_f = (temperature_c * 9.0 / 5.0) + 32.0;

        // Current Activity
        if (fabs(linear_accel.x - prev_linear_accel_x) < ACCEL_CHANGE_THRESHOLD && fabs(gravity.x - prev_linear_accel_x) < ACCEL_CHANGE_THRESHOLD) {
            snprintf(buffer, sizeof(buffer), "Idle");
        } else if (linear_accel.x >= 0.2 && linear_accel.x < 1.0) {
            snprintf(buffer, sizeof(buffer), "Walking");
        } else if (linear_accel.x >= 1.0 && linear_accel.x < 2.0) {
            snprintf(buffer, sizeof(buffer), "Running");
        } else if (linear_accel.x >= 2.0) {
            snprintf(buffer, sizeof(buffer), "Stairs");
        } else {
            snprintf(buffer, sizeof(buffer), "Getting up");
        }

        //update linear_accel
        prev_linear_accel_x = linear_accel.x;

        // Display activity on LCD1
        LiquidCrystal_I2C_Clear(&lcd);
        LiquidCrystal_I2C_SetCursor(&lcd, 0, 0);
        LiquidCrystal_I2C_Print(&lcd, buffer);

        // Non-negative speed
        float speed = fabs(linear_accel.x);

        // Update if speed has changed a lot or is very low
        if (fabs(speed - prev_speed) > SPEED_CHANGE_THRESHOLD || speed < SPEED_CHANGE_THRESHOLD) {
            if (speed < SPEED_CHANGE_THRESHOLD) {
                speed = 0.0;
            }
            prev_speed = speed;
            LiquidCrystal_I2C_SetCursor(&lcd2, 0, 0);
            snprintf(buffer, sizeof(buffer), "Your Speed: %.1f", speed);
            LiquidCrystal_I2C_Print(&lcd2, buffer);
        }

        // Update temp if change is greater than threshold
        if (fabs(temperature_f - prev_temperature_f) > TEMP_CHANGE_THRESHOLD) {
            prev_temperature_f = temperature_f;
            LiquidCrystal_I2C_SetCursor(&lcd2, 0, 1);
            snprintf(buffer, sizeof(buffer), "Temperature: %.1f F", temperature_f);
            LiquidCrystal_I2C_Print(&lcd2, buffer);
        }
        HAL_Delay(600);
    }
}

void Error_Handler(char *errorMessage, int lcdNumber) {
    if (lcdNumber == 1) {
        LiquidCrystal_I2C_Clear(&lcd);
        LiquidCrystal_I2C_SetCursor(&lcd, 0, 0);
        LiquidCrystal_I2C_Print(&lcd, errorMessage);
    } else if (lcdNumber == 2) {
        LiquidCrystal_I2C_Clear(&lcd2);
        LiquidCrystal_I2C_SetCursor(&lcd2, 0, 0);
        LiquidCrystal_I2C_Print(&lcd2, errorMessage);
    }
    HAL_Delay(2000);
    NVIC_SystemReset();
}

// I2C Configuration
static void MX_I2C1_Init(void) {
    i2c.Instance = I2C1;
    i2c.Init.ClockSpeed = 100000;
    i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    i2c.Init.OwnAddress1 = 0;
    i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c.Init.OwnAddress2 = 0;
    i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&i2c) != HAL_OK) {
        Error_Handler("I2C Initialization Failed", 1);
    }
}

// SCL = PB6 and SDA = PB7 configurations
static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// Sys Clock Configurations
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler("RCC Oscillator Configuration Failed", 2);
    }
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler("RCC Clock Configuration Failed", 2);
    }
}
