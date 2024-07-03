#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "LiquidCrystal_I2C.h"
#include "bno055.h"
#include <time.h>

void SystemClock_Config(void);
void Error_Handler(char *errorMessage, int lcdNumber);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

I2C_HandleTypeDef i2c;
LiquidCrystal_I2C_HandleTypeDef lcd;
LiquidCrystal_I2C_HandleTypeDef lcd2;


void bno055_writeData(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};
    if (HAL_I2C_Master_Transmit(&i2c, BNO055_I2C_ADDR << 1, buffer, 2, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler("BNO055 write failed", 1);
    }
}

void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len) {
    if (HAL_I2C_Master_Transmit(&i2c, BNO055_I2C_ADDR << 1, &reg, 1, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler("BNO055 read address failed", 1);
    }
    if (HAL_I2C_Master_Receive(&i2c, BNO055_I2C_ADDR << 1, data, len, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler("BNO055 read data failed", 1);
    }
}

void bno055_delay(int time) {
    HAL_Delay(time);
}

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
    bno055_vector_t euler;

    while (1) {

        // Read Euler
        euler = bno055_getVectorEuler();

        // Display Euler on LCD1
        LiquidCrystal_I2C_Clear(&lcd);
        LiquidCrystal_I2C_SetCursor(&lcd, 0, 0);
        snprintf(buffer, sizeof(buffer), "Yaw: %.2f", euler.z);
        LiquidCrystal_I2C_Print(&lcd, buffer);

        LiquidCrystal_I2C_SetCursor(&lcd, 0, 1);
        snprintf(buffer, sizeof(buffer), "Pitch: %.2f", euler.y);
        LiquidCrystal_I2C_Print(&lcd, buffer);

        LiquidCrystal_I2C_SetCursor(&lcd, 0, 2);
        snprintf(buffer, sizeof(buffer), "Roll: %.2f", euler.x);
        LiquidCrystal_I2C_Print(&lcd, buffer);

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
