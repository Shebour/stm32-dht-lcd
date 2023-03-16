#include "hd44780.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdint.h>

static void pulse_enable() {
  HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
}

static void write4bits(uint8_t value, struct LCD *screen) {
  for (int i = 0; i < 4; i++) {
    uint8_t val = (value >> i) & 0x01;
    if (val == 1) {
      HAL_GPIO_WritePin(screen->ports[i], screen->data_pin[i], GPIO_PIN_SET);
    } else if (val == 0) {
      HAL_GPIO_WritePin(screen->ports[i], screen->data_pin[i], GPIO_PIN_RESET);
    }
  }
  pulse_enable();
}

static void write8bits(uint8_t value, struct LCD *screen) {
  for (int i = 0; i < 8; i++) {
    uint8_t val = (value >> i) & 0x01;
    if (val == 1) {
      HAL_GPIO_WritePin(screen->ports[i], screen->data_pin[i], GPIO_PIN_SET);
    } else if (val == 0) {
      HAL_GPIO_WritePin(screen->ports[i], screen->data_pin[i], GPIO_PIN_RESET);
    }
  }
  pulse_enable();
}

static void send(uint8_t value, uint8_t mode, struct LCD *screen) {
  if (mode == 1)
    HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
  if (screen->bits == 8)
    write8bits(value, screen);
  else {
    write4bits(value >> 4, screen);
    write4bits(value, screen);
  }
}

static void LCD_Init(int rows, int fourbit, uint16_t data_pin[],
                     GPIO_TypeDef *ports[], struct LCD *self) {
  for (int i = 0; i < 8; i++) {
    self->data_pin[i] = data_pin[i];
    self->ports[i] = ports[i];
  }
  if (fourbit) {
    self->disp_function = LCD_4BITMODE | LCD_5x8DOTS;
    self->bits = 4;
  } else {
    self->disp_function = LCD_8BITMODE | LCD_5x8DOTS;
    self->bits = 8;
  }
  if (rows == 1) {
    self->disp_function |= LCD_1LINE;
  } else {
    self->disp_function |= LCD_2LINE;
  }
  self->rows = rows;
  self->rows_offset[0] = 0x00;
  self->rows_offset[1] = 0x40;
  self->rows_offset[2] = 0x00 + COLS;
  self->rows_offset[3] = 0x40 + COLS;

  HAL_Delay(50);
  if (self->bits == 4) {
    write4bits(0x03, self);
    HAL_Delay(50);
    write4bits(0x03, self);
    HAL_Delay(50);
    write4bits(0x03, self);
    HAL_Delay(150);
    write4bits(0x02, self);
  } else {
    self->command(LCD_FUNCTIONSET | self->disp_function, self);
    HAL_Delay(50);
    self->command(LCD_FUNCTIONSET | self->disp_function, self);
    HAL_Delay(50);
    self->command(LCD_FUNCTIONSET | self->disp_function, self);
  }
  self->command(LCD_FUNCTIONSET | self->disp_function, self);
  self->disp_control = LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF;
  self->display(self);

  self->clear(self);
  self->disp_mode = LCD_ENTRY_L | LCD_ENTRY_SD;
  self->command(LCD_ENTRYMODESET | self->disp_mode, self);
}

static void LCD_Clear(struct LCD *self) {
  self->command(LCD_CLEARDISPLAY, self);
  HAL_Delay(1000);
}

static void LCD_Home(struct LCD *self) {
  self->command(LCD_RETURNHOME, self);
  HAL_Delay(1000);
}

static void LCD_Display(struct LCD *self) {
  self->disp_control |= LCD_DISPLAY_ON;
  self->command(LCD_DISPLAYCONTROL | self->disp_control, self);
}

static void LCD_No_Display(struct LCD *self) {
  self->disp_control &= ~LCD_DISPLAY_ON;
  self->command(LCD_DISPLAYCONTROL | self->disp_control, self);
}

static void LCD_No_Blink(struct LCD *self) {
  self->disp_control &= ~LCD_BLINK_ON;
  self->command(LCD_DISPLAYCONTROL | self->disp_control, self);
}

static void LCD_Blink(struct LCD *self) {
  self->disp_control |= LCD_BLINK_ON;
  self->command(LCD_DISPLAYCONTROL | self->disp_control, self);
}

static void LCD_No_Cursor(struct LCD *self) {
  self->disp_control &= ~LCD_CURSOR_ON;
  self->command(LCD_DISPLAYCONTROL | self->disp_control, self);
}

static void LCD_Cursor(struct LCD *self) {
  self->disp_control |= LCD_CURSOR_ON;
  self->command(LCD_DISPLAYCONTROL | self->disp_control, self);
}

static void LCD_Scroll_Left(struct LCD *self) {
  self->command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT, self);
}

static void LCD_Scroll_Right(struct LCD *self) {
  self->command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT, self);
}
static void LCD_L2R(struct LCD *self) {
  self->disp_mode |= LCD_ENTRY_L;
  self->command(LCD_ENTRYMODESET | self->disp_mode, self);
}

static void LCD_R2L(struct LCD *self) {
  self->disp_mode &= ~LCD_ENTRY_L;
  self->command(LCD_ENTRYMODESET | self->disp_mode, self);
}
static void LCD_Auto_Scroll(struct LCD *self) {
  self->disp_mode |= LCD_ENTRY_SI;
  self->command(LCD_ENTRYMODESET | self->disp_mode, self);
}

static void LCD_No_Auto_Scroll(struct LCD *self) {
  self->disp_mode &= ~LCD_ENTRY_SI;
  self->command(LCD_ENTRYMODESET | self->disp_mode, self);
}

static void LCD_Create_Char(uint8_t location, uint8_t charmap[],
                            struct LCD *self) {
  location &= 0x7;
  self->command(LCD_SETCGRAMADDR | (location << 3), self);
  for (int i = 0; i < 8; i++) {
    send(charmap[i], 1, self);
  }
}
static void LCD_Set_Cursor(uint8_t col, uint8_t row, struct LCD *self) {
  if (row >= self->rows) {
    row = self->rows - 1;
  }
  if (col >= COLS) {
    col = COLS - 1;
  }
  self->command(LCD_SETDDRAMADDR | (col + self->rows_offset[row]), self);
}
static void LCD_Print(char *str, struct LCD *self) {
  for (unsigned i = 0; str[i] != '\0'; i++) {
    send(str[i], 1, self);
  }
}
static void LCD_Command(uint8_t cmd, struct LCD *self) { send(cmd, 0, self); }

static void LCD_Write(uint8_t value, struct LCD *self) { send(value, 1, self); }

static int mypow(int x, int y) {
  int res = x;
  while (y > 1) {
    res *= x;
    y--;
  }
  return res;
}

static int my_convert(int value, int base, char *str) {
  int size = 0;
  if (value < 0) {
    str[size] = '-';
    value = -value;
    size++;
  }
  if (value < base) {
    char c = value + '0';
    str[size] = c;
    return 1;
  }
  size += my_convert(value / base, base, str);
  str[size] = ('0' + value % base);
  return 1 + size;
}

static void floattostr(float value, int decimal, char *str) {
  int integer_part = (int)value;
  value = value - (float)integer_part;
  value = value * mypow(10, decimal);
  int dec_part = (int)value;
  char str_int[100] = {0};
  char str_dec[100] = {0};
  my_convert(integer_part, 10, str_int);
  my_convert(dec_part, 10, str_dec);
  int i = 0;
  while (str_int[i] != '\0') {
    str[i] = str_int[i];
    i++;
  }
  str[i] = '.';
  i++;
  int j = 0;
  while (str_dec[j] != '\0') {
    str[i] = str_dec[j];
    i++;
    j++;
  }
}

static void LCD_Print_Float(float value, int decimal, struct LCD *self) {
  char str[16] = {0};
  floattostr(value, decimal, str);
  for (unsigned i = 0; str[i] != '\0'; i++) {
    send(str[i], 1, self);
  }
}

void init_screen(struct LCD *screen) {
  screen->init = LCD_Init;
  screen->clear = LCD_Clear;
  screen->home = LCD_Home;
  screen->display = LCD_Display;
  screen->no_display = LCD_No_Display;
  screen->no_blink = LCD_No_Blink;
  screen->blink = LCD_Blink;
  screen->no_cursor = LCD_No_Cursor;
  screen->cursor = LCD_Cursor;
  screen->scroll_left = LCD_Scroll_Left;
  screen->scroll_right = LCD_Scroll_Right;
  screen->left_to_right = LCD_L2R;
  screen->right_to_left = LCD_R2L;
  screen->auto_scroll = LCD_Auto_Scroll;
  screen->no_auto_scroll = LCD_No_Auto_Scroll;
  screen->create_char = LCD_Create_Char;
  screen->set_cursor = LCD_Set_Cursor;
  screen->print = LCD_Print;
  screen->command = LCD_Command;
  screen->write = LCD_Write;
  screen->print_float = LCD_Print_Float;
}
