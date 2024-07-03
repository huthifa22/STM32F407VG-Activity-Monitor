#include "LiquidCrystal_I2C.h"

void LiquidCrystal_I2C_Init(LiquidCrystal_I2C_HandleTypeDef *lcd, I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t cols, uint8_t rows) {
    lcd->hi2c = hi2c;
    lcd->Address = address << 1;
    lcd->Cols = cols;
    lcd->Rows = rows;
    lcd->BacklightVal = LCD_BACKLIGHT;
    lcd->DisplayFunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
    if (rows > 1) {
        lcd->DisplayFunction |= LCD_2LINE;
    }
}

void LiquidCrystal_I2C_Begin(LiquidCrystal_I2C_HandleTypeDef *lcd, uint8_t cols, uint8_t rows, uint8_t charsize) {
    lcd->NumLines = rows;
    lcd->DisplayFunction |= charsize;

    HAL_Delay(50); // Wait for >40ms

    LiquidCrystal_I2C_ExpanderWrite(lcd, lcd->BacklightVal);
    HAL_Delay(1000);

    // Initialization sequence
    LiquidCrystal_I2C_Write4Bits(lcd, 0x03 << 4);
    HAL_Delay(5); // Wait min 4.1ms

    LiquidCrystal_I2C_Write4Bits(lcd, 0x03 << 4);
    HAL_Delay(5); // Wait min 4.1ms

    LiquidCrystal_I2C_Write4Bits(lcd, 0x03 << 4);
    HAL_Delay(1);

    LiquidCrystal_I2C_Write4Bits(lcd, 0x02 << 4); // Finally, set to 4-bit interface

    LiquidCrystal_I2C_Command(lcd, LCD_FUNCTIONSET | lcd->DisplayFunction);

    lcd->DisplayControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    LiquidCrystal_I2C_Display(lcd);

    LiquidCrystal_I2C_Clear(lcd);

    lcd->DisplayMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    LiquidCrystal_I2C_Command(lcd, LCD_ENTRYMODESET | lcd->DisplayMode);
}

void LiquidCrystal_I2C_Clear(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    LiquidCrystal_I2C_Command(lcd, LCD_CLEARDISPLAY);
    HAL_Delay(2);
}

void LiquidCrystal_I2C_Home(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    LiquidCrystal_I2C_Command(lcd, LCD_RETURNHOME);
    HAL_Delay(2);
}

void LiquidCrystal_I2C_NoDisplay(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    lcd->DisplayControl &= ~LCD_DISPLAYON;
    LiquidCrystal_I2C_Command(lcd, LCD_DISPLAYCONTROL | lcd->DisplayControl);
}

void LiquidCrystal_I2C_Display(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    lcd->DisplayControl |= LCD_DISPLAYON;
    LiquidCrystal_I2C_Command(lcd, LCD_DISPLAYCONTROL | lcd->DisplayControl);
}

void LiquidCrystal_I2C_NoBlink(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    lcd->DisplayControl &= ~LCD_BLINKON;
    LiquidCrystal_I2C_Command(lcd, LCD_DISPLAYCONTROL | lcd->DisplayControl);
}

void LiquidCrystal_I2C_Blink(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    lcd->DisplayControl |= LCD_BLINKON;
    LiquidCrystal_I2C_Command(lcd, LCD_DISPLAYCONTROL | lcd->DisplayControl);
}

void LiquidCrystal_I2C_NoCursor(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    lcd->DisplayControl &= ~LCD_CURSORON;
    LiquidCrystal_I2C_Command(lcd, LCD_DISPLAYCONTROL | lcd->DisplayControl);
}

void LiquidCrystal_I2C_Cursor(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    lcd->DisplayControl |= LCD_CURSORON;
    LiquidCrystal_I2C_Command(lcd, LCD_DISPLAYCONTROL | lcd->DisplayControl);
}

void LiquidCrystal_I2C_ScrollDisplayLeft(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    LiquidCrystal_I2C_Command(lcd, LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void LiquidCrystal_I2C_ScrollDisplayRight(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    LiquidCrystal_I2C_Command(lcd, LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void LiquidCrystal_I2C_PrintLeft(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    LiquidCrystal_I2C_Command(lcd, LCD_ENTRYMODESET | LCD_ENTRYLEFT);
}

void LiquidCrystal_I2C_PrintRight(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    LiquidCrystal_I2C_Command(lcd, LCD_ENTRYMODESET | LCD_ENTRYRIGHT);
}

void LiquidCrystal_I2C_LeftToRight(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    lcd->DisplayMode |= LCD_ENTRYLEFT;
    LiquidCrystal_I2C_Command(lcd, LCD_ENTRYMODESET | lcd->DisplayMode);
}

void LiquidCrystal_I2C_RightToLeft(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    lcd->DisplayMode &= ~LCD_ENTRYLEFT;
    LiquidCrystal_I2C_Command(lcd, LCD_ENTRYMODESET | lcd->DisplayMode);
}

void LiquidCrystal_I2C_ShiftIncrement(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    lcd->DisplayMode |= LCD_ENTRYSHIFTINCREMENT;
    LiquidCrystal_I2C_Command(lcd, LCD_ENTRYMODESET | lcd->DisplayMode);
}

void LiquidCrystal_I2C_ShiftDecrement(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    lcd->DisplayMode &= ~LCD_ENTRYSHIFTINCREMENT;
    LiquidCrystal_I2C_Command(lcd, LCD_ENTRYMODESET | lcd->DisplayMode);
}

void LiquidCrystal_I2C_NoBacklight(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    lcd->BacklightVal = LCD_NOBACKLIGHT;
    LiquidCrystal_I2C_ExpanderWrite(lcd, 0);
}

void LiquidCrystal_I2C_Backlight(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    lcd->BacklightVal = LCD_BACKLIGHT;
    LiquidCrystal_I2C_ExpanderWrite(lcd, 0);
}

void LiquidCrystal_I2C_Autoscroll(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    lcd->DisplayMode |= LCD_ENTRYSHIFTINCREMENT;
    LiquidCrystal_I2C_Command(lcd, LCD_ENTRYMODESET | lcd->DisplayMode);
}

void LiquidCrystal_I2C_NoAutoscroll(LiquidCrystal_I2C_HandleTypeDef *lcd) {
    lcd->DisplayMode &= ~LCD_ENTRYSHIFTINCREMENT;
    LiquidCrystal_I2C_Command(lcd, LCD_ENTRYMODESET | lcd->DisplayMode);
}

void LiquidCrystal_I2C_CreateChar(LiquidCrystal_I2C_HandleTypeDef *lcd, uint8_t location, uint8_t charmap[]) {
    location &= 0x7;
    LiquidCrystal_I2C_Command(lcd, LCD_SETCGRAMADDR | (location << 3));
    for (int i = 0; i < 8; i++) {
        LiquidCrystal_I2C_Send(lcd, charmap[i], Rs);
    }
}

void LiquidCrystal_I2C_SetCursor(LiquidCrystal_I2C_HandleTypeDef *lcd, uint8_t col, uint8_t row) {
    const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row > lcd->NumLines) {
        row = lcd->NumLines - 1;
    }
    LiquidCrystal_I2C_Command(lcd, LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void LiquidCrystal_I2C_Print(LiquidCrystal_I2C_HandleTypeDef *lcd, const char *str) {
    while (*str) {
        LiquidCrystal_I2C_Send(lcd, *str++, Rs);
    }
}

void LiquidCrystal_I2C_Command(LiquidCrystal_I2C_HandleTypeDef *lcd, uint8_t value) {
    LiquidCrystal_I2C_Send(lcd, value, 0);
}

void LiquidCrystal_I2C_Send(LiquidCrystal_I2C_HandleTypeDef *lcd, uint8_t value, uint8_t mode) {
    uint8_t highnib = value & 0xf0;
    uint8_t lownib = (value << 4) & 0xf0;
    LiquidCrystal_I2C_Write4Bits(lcd, (highnib) | mode);
    LiquidCrystal_I2C_Write4Bits(lcd, (lownib) | mode);
}

void LiquidCrystal_I2C_Write4Bits(LiquidCrystal_I2C_HandleTypeDef *lcd, uint8_t value) {
    LiquidCrystal_I2C_ExpanderWrite(lcd, value);
    LiquidCrystal_I2C_PulseEnable(lcd, value);
}

void LiquidCrystal_I2C_ExpanderWrite(LiquidCrystal_I2C_HandleTypeDef *lcd, uint8_t _data) {
    uint8_t data_t[1];
    data_t[0] = _data | lcd->BacklightVal;
    HAL_I2C_Master_Transmit(lcd->hi2c, lcd->Address, data_t, 1, 1000);
}

void LiquidCrystal_I2C_PulseEnable(LiquidCrystal_I2C_HandleTypeDef *lcd, uint8_t _data) {
    LiquidCrystal_I2C_ExpanderWrite(lcd, _data | En);
    HAL_Delay(1);
    LiquidCrystal_I2C_ExpanderWrite(lcd, _data & ~En);
    HAL_Delay(1);
}
