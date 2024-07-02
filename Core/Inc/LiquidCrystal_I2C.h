#ifndef LiquidCrystal_I2C_H
#define LiquidCrystal_I2C_H

#include "stm32f4xx_hal.h"
#include <inttypes.h>

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En 0x04  // Enable bit
#define Rw 0x02  // Read/Write bit
#define Rs 0x01  // Register select bit

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t Address;
    uint8_t Cols;
    uint8_t Rows;
    uint8_t BacklightVal;
    uint8_t DisplayFunction;
    uint8_t DisplayControl;
    uint8_t DisplayMode;
    uint8_t NumLines;
} LiquidCrystal_I2C_HandleTypeDef;

void LiquidCrystal_I2C_Init(LiquidCrystal_I2C_HandleTypeDef *lcd, I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t cols, uint8_t rows);
void LiquidCrystal_I2C_Begin(LiquidCrystal_I2C_HandleTypeDef *lcd, uint8_t cols, uint8_t rows, uint8_t charsize);
void LiquidCrystal_I2C_Clear(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_Home(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_NoDisplay(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_Display(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_NoBlink(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_Blink(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_NoCursor(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_Cursor(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_ScrollDisplayLeft(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_ScrollDisplayRight(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_PrintLeft(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_PrintRight(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_LeftToRight(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_RightToLeft(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_ShiftIncrement(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_ShiftDecrement(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_NoBacklight(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_Backlight(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_Autoscroll(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_NoAutoscroll(LiquidCrystal_I2C_HandleTypeDef *lcd);
void LiquidCrystal_I2C_CreateChar(LiquidCrystal_I2C_HandleTypeDef *lcd, uint8_t location, uint8_t charmap[]);
void LiquidCrystal_I2C_SetCursor(LiquidCrystal_I2C_HandleTypeDef *lcd, uint8_t col, uint8_t row);
void LiquidCrystal_I2C_Print(LiquidCrystal_I2C_HandleTypeDef *lcd, const char *str);
void LiquidCrystal_I2C_Command(LiquidCrystal_I2C_HandleTypeDef *lcd, uint8_t value);
void LiquidCrystal_I2C_Send(LiquidCrystal_I2C_HandleTypeDef *lcd, uint8_t value, uint8_t mode);
void LiquidCrystal_I2C_Write4Bits(LiquidCrystal_I2C_HandleTypeDef *lcd, uint8_t value);
void LiquidCrystal_I2C_ExpanderWrite(LiquidCrystal_I2C_HandleTypeDef *lcd, uint8_t _data);
void LiquidCrystal_I2C_PulseEnable(LiquidCrystal_I2C_HandleTypeDef *lcd, uint8_t _data);

#endif
