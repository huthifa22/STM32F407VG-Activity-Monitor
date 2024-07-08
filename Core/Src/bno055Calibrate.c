#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "bno055.h"

void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
int checkAndDisplayCalibrationStatus();

I2C_HandleTypeDef i2c;

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    // Initialize BNO055
    bno055_setup();
    bno055_setOperationModeNDOF();

    while (1) {
        if (checkAndDisplayCalibrationStatus()) {
            break;
        }
        HAL_Delay(500);
    }
}

void Error_Handler(void) {
    while(1) {
    }
}

int checkAndDisplayCalibrationStatus() {
    bno055_calibration_state_t cal = bno055_getCalibrationState();

    printf("Calibration Status - Sys: %d, Gyro: %d, Accel: %d, Mag: %d\n", cal.sys, cal.gyro, cal.accel, cal.mag);

    if (cal.sys == 3 && cal.gyro == 3 && cal.accel == 3 && cal.mag == 3) {
        printf("Calibration done\n");

        bno055_calibration_data_t calData = bno055_getCalibrationData();

        printf("Calibration Data:\n");
        printf("Accel Offset: x=%d, y=%d, z=%d\n", calData.offset.accel.x, calData.offset.accel.y, calData.offset.accel.z);
        printf("Mag Offset: x=%d, y=%d, z=%d\n", calData.offset.mag.x, calData.offset.mag.y, calData.offset.mag.z);
        printf("Gyro Offset: x=%d, y=%d, z=%d\n", calData.offset.gyro.x, calData.offset.gyro.y, calData.offset.gyro.z);
        printf("Accel Radius: %d\n", calData.radius.accel);
        printf("Mag Radius: %d\n", calData.radius.mag);

        return 1;
    }
    return 0;
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
        Error_Handler();
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
        Error_Handler();
    }
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}
