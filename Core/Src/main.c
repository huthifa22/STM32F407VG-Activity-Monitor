#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "LiquidCrystal_I2C.h"
#include "bno055.h"
#include <math.h>
#include <string.h>

#define TEMP_CHANGE_THRESHOLD 0.1
#define DEGREE_CHANGE_THRESHOLD 4.0
#define ACCEL_CHANGE_THRESHOLD 0.06
#define DECAY_FACTOR 0.7

#define IDLE_EULER_CHANGE_THRESHOLD 0.09
#define IDLE_GRAVITY_CHANGE_THRESHOLD 0.09
#define IDLE_DEBOUNCE_THRESHOLD 2

#define SLIGHTLY_MOVING_EULER_CHANGE_THRESHOLD 0.11
#define SLIGHTLY_MOVING_DEBOUNCE_THRESHOLD 1

#define MOVING_EULER_CHANGE_THRESHOLD 0.5
#define MOVING_GRAVITY_CHANGE_THRESHOLD 1.5
#define MOVING_DEBOUNCE_THRESHOLD 3

#define WALKING_ACCEL_CHANGE_THRESHOLD 0.7
#define WALKING_DEBOUNCE_THRESHOLD 2

char buffer[20];
bno055_vector_t accel, mag, gyro, euler, linear_accel, gravity;
bno055_vector_t prev_euler, prev_gravity, prev_linear_accel;
int8_t temp_raw;
float temperature_f, prev_temperature_f = 0.0;
float displayed_acceleration = 0.0;
float heading, previous_heading = -1.0;
float degree_difference;
const char* direction;
const char* previousDirection = "";
int idleDebounceCounter = 0, slightlyMovingDebounceCounter = 0, movingDebounceCounter = 0, walkingDebounceCounter = 0;

void SystemClock_Config(void);
void Error_Handler(char *errorMessage, int lcdNumber);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
float convertToFahrenheit(int8_t temp_c);
void hardcodeCalibrationData();
const char* getCompassDirection(float heading);
float calculateHeading(bno055_vector_t mag);
int isUserIdle(bno055_vector_t euler, bno055_vector_t prev_euler, bno055_vector_t gravity, bno055_vector_t prev_gravity);
int isUserSlightlyMoving(bno055_vector_t euler, bno055_vector_t prev_euler);
int isUserMoving(bno055_vector_t euler, bno055_vector_t prev_euler, bno055_vector_t gravity, bno055_vector_t prev_gravity, bno055_vector_t gyro);
int isUserWalking(bno055_vector_t linear_accel, bno055_vector_t prev_linear_accel);
void userMovementStatus(bno055_vector_t euler, bno055_vector_t prev_euler, bno055_vector_t gravity, bno055_vector_t prev_gravity,
						bno055_vector_t linear_accel, bno055_vector_t prev_linear_accel, bno055_vector_t gyro);

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
    hardcodeCalibrationData();
    bno055_setOperationModeNDOF();

    // Initial Readings
    accel = bno055_getVectorAccelerometer();
    mag = bno055_getVectorMagnetometer();
    gyro = bno055_getVectorGyroscope();
    euler = bno055_getVectorEuler();
    linear_accel = bno055_getVectorLinearAccel();
    gravity = bno055_getVectorGravity();
    temp_raw = bno055_getTemp();

    // Initial Status
    prev_euler = euler;
    prev_gravity = gravity;
    prev_linear_accel = accel;

    LiquidCrystal_I2C_SetCursor(&lcd, 0, 0);
    LiquidCrystal_I2C_Print(&lcd, "Movement: Gauging...");

    while (1) {

        // Get Sensor Data
        accel = bno055_getVectorAccelerometer();
        mag = bno055_getVectorMagnetometer();
        gyro = bno055_getVectorGyroscope();
        euler = bno055_getVectorEuler();
        linear_accel = bno055_getVectorLinearAccel();
        gravity = bno055_getVectorGravity();
        temp_raw = bno055_getTemp();

        // Debug
        printf("Euler: x=%.2f, y=%.2f, z=%.2f\n", euler.x, euler.y, euler.z);
        printf("Gravity: x=%.2f, y=%.2f, z=%.2f\n", gravity.x, gravity.y, gravity.z);

        // Update movement
        userMovementStatus(euler, prev_euler, gravity, prev_gravity, linear_accel, prev_linear_accel, gyro);

        // Update previous values
        prev_euler = euler;
        prev_gravity = gravity;
        prev_linear_accel = linear_accel;

        // If acceleration change is greater than threshold then display it
        if (fabs(linear_accel.y) > ACCEL_CHANGE_THRESHOLD) {
            displayed_acceleration = fabs(linear_accel.y);
        } else {
            // Slow down rate
            displayed_acceleration *= DECAY_FACTOR;
            if (displayed_acceleration < 0.3) {
                displayed_acceleration = 0.0;
            }
        }

        // Display Acceleration
        LiquidCrystal_I2C_SetCursor(&lcd2, 0, 0);
        snprintf(buffer, sizeof(buffer), "Speed: %.1f", displayed_acceleration);
        LiquidCrystal_I2C_Print(&lcd2, buffer);

        // Convert Temperature from C to F
        temperature_f = convertToFahrenheit(temp_raw);

        // Update temperature if change is greater than threshold
        if (fabs(temperature_f - prev_temperature_f) > TEMP_CHANGE_THRESHOLD) {
            prev_temperature_f = temperature_f;
            LiquidCrystal_I2C_SetCursor(&lcd2, 0, 1);
            snprintf(buffer, sizeof(buffer), "Temp: %.1f %cF", temperature_f, 223); //223 is char for degrees
            LiquidCrystal_I2C_Print(&lcd2, buffer);
        }

        // Calculate direction using magnetometer and display it
        heading = calculateHeading(mag);
        if (heading == -0.0) heading = 0.0;
        direction = getCompassDirection(heading);

        // Calculate the degree difference
        degree_difference = fabs(heading - previous_heading);

        // Angle wrap around check
        if (degree_difference > 180.0) {
            degree_difference = 360.0 - degree_difference;
        }

        // Update direction and angle if degree difference is greater than threshold
        if (degree_difference > DEGREE_CHANGE_THRESHOLD) {
            if (strcmp(direction, previousDirection) != 0) {
                previousDirection = direction;
                LiquidCrystal_I2C_SetCursor(&lcd2, 0, 2);
                snprintf(buffer, sizeof(buffer), "Heading: %s          ", direction);
                LiquidCrystal_I2C_Print(&lcd2, buffer);
            }

            previous_heading = heading;
            LiquidCrystal_I2C_SetCursor(&lcd2, 0, 3);
            snprintf(buffer, sizeof(buffer), "Angle: %.f%c    ", heading, 223); //223 is char for degrees
            LiquidCrystal_I2C_Print(&lcd2, buffer);
        }
        HAL_Delay(100);
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

// Data after calibrating the IMU in current environment
// bno055Calibrate.c can be used to retrieve new calibration
void hardcodeCalibrationData() {
    bno055_calibration_data_t calData;

    calData.offset.accel.x = -29;
    calData.offset.accel.y = -16;
    calData.offset.accel.z = -31;
    calData.offset.mag.x = 79;
    calData.offset.mag.y = -150;
    calData.offset.mag.z = 80;
    calData.offset.gyro.x = -2;
    calData.offset.gyro.y = 0;
    calData.offset.gyro.z = 0;
    calData.radius.accel = 1000;
    calData.radius.mag = 814;

    bno055_setCalibrationData(calData);
}

// From Celsius to Fahrenheit
float convertToFahrenheit(int8_t temp_c) {
    return (temp_c * 9.0 / 5.0) + 32.0;
}

// Retrieve Direction
const char* getCompassDirection(float heading) {
    if ((heading >= 337.5) || (heading < 22.5)) {
        return "*North*";
    } else if (heading < 67.5) {
        return "NorthEast";
    } else if (heading < 112.5) {
        return "East";
    } else if (heading < 157.5) {
        return "SouthEast";
    } else if (heading < 202.5) {
        return "South";
    } else if (heading < 247.5) {
        return "SouthWest";
    } else if (heading < 292.5) {
        return "West";
    } else {
        return "NorthWest";
    }
}

// Calculate heading angle in degrees from magnetometer
float calculateHeading(bno055_vector_t mag) {
    // Since IMU is turned 90 deg to the right we adjust mag.x -> -mag.x
    float heading = atan2(-mag.x, mag.y) * 180 / M_PI;
    if (heading < 0) {
        heading += 360;
    }
    return heading;
}

// Movement Status - idle, slightly moving, moving, walking
void userMovementStatus(bno055_vector_t euler, bno055_vector_t prev_euler, bno055_vector_t gravity, bno055_vector_t prev_gravity,
                        bno055_vector_t linear_accel, bno055_vector_t prev_linear_accel, bno055_vector_t gyro) {

    // Check if the user is walking
    if (isUserWalking(linear_accel, prev_linear_accel)) {
        walkingDebounceCounter++;
        idleDebounceCounter = 0;
        slightlyMovingDebounceCounter = 0;
        movingDebounceCounter = 0;
        if (walkingDebounceCounter >= WALKING_DEBOUNCE_THRESHOLD) {
            LiquidCrystal_I2C_SetCursor(&lcd, 10, 0);
            LiquidCrystal_I2C_Print(&lcd, "Walking   ");
        }
    }
    // Check if the user is moving
    else if (isUserMoving(euler, prev_euler, gravity, prev_gravity, gyro)) {
        movingDebounceCounter++;
        idleDebounceCounter = 0;
        slightlyMovingDebounceCounter = 0;
        walkingDebounceCounter = 0;
        if (movingDebounceCounter >= MOVING_DEBOUNCE_THRESHOLD) {
            LiquidCrystal_I2C_SetCursor(&lcd, 10, 0);
            LiquidCrystal_I2C_Print(&lcd, "Moving    ");
        }
    }
    // Check if the user is slightly moving
    else if (isUserSlightlyMoving(euler, prev_euler)) {
        slightlyMovingDebounceCounter++;
        idleDebounceCounter = 0;
        movingDebounceCounter = 0;
        walkingDebounceCounter = 0;
        if (slightlyMovingDebounceCounter >= SLIGHTLY_MOVING_DEBOUNCE_THRESHOLD) {
            LiquidCrystal_I2C_SetCursor(&lcd, 10, 0);
            LiquidCrystal_I2C_Print(&lcd, "Slight    ");
        }
    }
    // Check if the user is idle
    else if (isUserIdle(euler, prev_euler, gravity, prev_gravity)) {
        idleDebounceCounter++;
        slightlyMovingDebounceCounter = 0;
        movingDebounceCounter = 0;
        walkingDebounceCounter = 0;
        if (idleDebounceCounter >= IDLE_DEBOUNCE_THRESHOLD) {
            LiquidCrystal_I2C_SetCursor(&lcd, 10, 0);
            LiquidCrystal_I2C_Print(&lcd, "Idle      ");
        }
    }
}

// Check if user is idle using euler and gravity
int isUserIdle(bno055_vector_t euler, bno055_vector_t prev_euler, bno055_vector_t gravity, bno055_vector_t prev_gravity) {
    float euler_change_x = fabs(euler.x - prev_euler.x);
    float euler_change_y = fabs(euler.y - prev_euler.y);
    float euler_change_z = fabs(euler.z - prev_euler.z);
    float gravity_change_x = fabs(gravity.x - prev_gravity.x);
    float gravity_change_y = fabs(gravity.y - prev_gravity.y);
    float gravity_change_z = fabs(gravity.z - prev_gravity.z);

    return (euler_change_x < IDLE_EULER_CHANGE_THRESHOLD &&
            euler_change_y < IDLE_EULER_CHANGE_THRESHOLD &&
            euler_change_z < IDLE_EULER_CHANGE_THRESHOLD &&
            gravity_change_x < IDLE_GRAVITY_CHANGE_THRESHOLD &&
            gravity_change_y < IDLE_GRAVITY_CHANGE_THRESHOLD &&
            gravity_change_z < IDLE_GRAVITY_CHANGE_THRESHOLD);
}

// Check if user is slightly moving using euler
int isUserSlightlyMoving(bno055_vector_t euler, bno055_vector_t prev_euler) {
    float euler_change_x = fabs(euler.x - prev_euler.x);
    float euler_change_y = fabs(euler.y - prev_euler.y);
    float euler_change_z = fabs(euler.z - prev_euler.z);

    return (euler_change_x >= SLIGHTLY_MOVING_EULER_CHANGE_THRESHOLD ||
            euler_change_y >= SLIGHTLY_MOVING_EULER_CHANGE_THRESHOLD ||
            euler_change_z >= SLIGHTLY_MOVING_EULER_CHANGE_THRESHOLD);
}

// Check if user is moving using euler, gravity, and gyro
int isUserMoving(bno055_vector_t euler, bno055_vector_t prev_euler, bno055_vector_t gravity, bno055_vector_t prev_gravity, bno055_vector_t gyro) {
    float euler_change_x = fabs(euler.x - prev_euler.x);
    float euler_change_y = fabs(euler.y - prev_euler.y);
    float euler_change_z = fabs(euler.z - prev_euler.z);
    float gravity_change_x = fabs(gravity.x - prev_gravity.x);
    float gravity_change_y = fabs(gravity.y - prev_gravity.y);
    float gravity_change_z = fabs(gravity.z - prev_gravity.z);

    // Magnitude of 3D angular velocity vector: |gyro| = √(gyro.x^2 + gyro.y^2 + gyro.z^2)
    float gyro_magnitude = sqrt((gyro.x * gyro.x) + (gyro.y * gyro.y) + (gyro.z * gyro.z));

    return (euler_change_x >= MOVING_EULER_CHANGE_THRESHOLD ||
            euler_change_y >= MOVING_EULER_CHANGE_THRESHOLD ||
            euler_change_z >= MOVING_EULER_CHANGE_THRESHOLD ||
            gravity_change_x >= MOVING_GRAVITY_CHANGE_THRESHOLD ||
            gravity_change_y >= MOVING_GRAVITY_CHANGE_THRESHOLD ||
            gravity_change_z >= MOVING_GRAVITY_CHANGE_THRESHOLD ||
            gyro_magnitude > MOVING_GRAVITY_CHANGE_THRESHOLD);
}

// Check if user is walking using magnitude of 2d vector
int isUserWalking(bno055_vector_t linear_accel, bno055_vector_t prev_linear_accel) {
    float accel_change_x = fabs(linear_accel.x - prev_linear_accel.x);
    float accel_change_y = fabs(linear_accel.y - prev_linear_accel.y);

    // Magnitude of 2d vector : |a| = √(accel_changeX^2 + accel_changeY^2)
    float accel_change_magnitude = sqrt((accel_change_x * accel_change_x) + (accel_change_y * accel_change_y));

    return (accel_change_magnitude >= WALKING_ACCEL_CHANGE_THRESHOLD);
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
