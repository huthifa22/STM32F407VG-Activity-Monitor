//#include "stm32f4xx_hal.h"
//#include "stdio.h"
//
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_I2C1_Init(void);
//
//I2C_HandleTypeDef i2c;
//
//int main(void) {
//    HAL_Init();
//    SystemClock_Config();
//    MX_GPIO_Init();
//    MX_I2C1_Init();
//
//    printf("Scanning I2C\n");
//
//    uint16_t addresses[] = {0x20, 0x27, 0x28, 0x29};
//    uint8_t num_addresses = sizeof(addresses) / sizeof(addresses[0]);
//
//    while (1) {
//        for (uint8_t i = 0; i < num_addresses; i++) {
//            uint16_t address = addresses[i];
//            if (HAL_I2C_IsDeviceReady(&i2c, address << 1, 1, HAL_MAX_DELAY) == HAL_OK) {
//                printf("I2C found at 0x%02X\n", address);
//            } else {
//                printf("Failed to detect at 0x%02X\n", address);
//            }
//        }
//        printf("Scan complete.\n");
//        HAL_Delay(2000);
//    }
//}
//
//// I2C Configuration
//static void MX_I2C1_Init(void) {
//    i2c.Instance = I2C1;
//    i2c.Init.ClockSpeed = 100000;
//    i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
//    i2c.Init.OwnAddress1 = 0;
//    i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//    i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//    i2c.Init.OwnAddress2 = 0;
//    i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//    i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//    HAL_I2C_Init(&i2c);
//}
//
//// SCL = PB6 and SDA = PB7 configurations
//static void MX_GPIO_Init(void) {
//    __HAL_RCC_GPIOB_CLK_ENABLE();
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//}
//
//// Sys Clock Configurations
//void SystemClock_Config(void) {
//    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//    __HAL_RCC_PWR_CLK_ENABLE();
//    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//    RCC_OscInitStruct.PLL.PLLM = 8;
//    RCC_OscInitStruct.PLL.PLLN = 336;
//    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//    RCC_OscInitStruct.PLL.PLLQ = 7;
//    HAL_RCC_OscConfig(&RCC_OscInitStruct);
//
//    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
//    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
//    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
//}
