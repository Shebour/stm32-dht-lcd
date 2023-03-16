#ifndef __HD44780_H
#define __HD44780_H
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

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
#define LCD_ENTRY_R 0x00
#define LCD_ENTRY_L 0x02
#define LCD_ENTRY_SI 0x01
#define LCD_ENTRY_SD 0x00

// flags for display on/off control
#define LCD_BLINK_ON 0x01
#define LCD_BLINK_OFF 0x00
#define LCD_CURSOR_ON 0x02
#define LCD_CURSOR_OFF 0x00
#define LCD_DISPLAY_ON 0x04
#define LCD_DISPLAY_OFF 0x00

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

#define COLS 16

struct LCD {
  int rows;
  int bits;
  uint16_t data_pin[8];
  GPIO_TypeDef *ports[8];
  uint8_t disp_function;
  uint8_t disp_control;
  uint8_t disp_mode;
  uint8_t rows_offset[4];
  void (*init)(int rows, int fourbit, uint16_t data_pin[],
               GPIO_TypeDef *ports[], struct LCD *self);
  void (*clear)(struct LCD *self);
  void (*home)(struct LCD *self);
  void (*display)(struct LCD *self);
  void (*no_display)(struct LCD *self);
  void (*no_blink)(struct LCD *self);
  void (*blink)(struct LCD *self);
  void (*no_cursor)(struct LCD *self);
  void (*cursor)(struct LCD *self);
  void (*scroll_left)(struct LCD *self);
  void (*scroll_right)(struct LCD *self);
  void (*left_to_right)(struct LCD *self);
  void (*right_to_left)(struct LCD *self);
  void (*auto_scroll)(struct LCD *self);
  void (*no_auto_scroll)(struct LCD *self);
  void (*create_char)(uint8_t location, uint8_t charmap[], struct LCD *self);
  void (*set_cursor)(uint8_t col, uint8_t row, struct LCD *self);
  void (*print)(char *str, struct LCD *self);
  void (*command)(uint8_t cmd, struct LCD *self);
  void (*write)(uint8_t value, struct LCD *self);
  void (*print_float)(float value, int decimal, struct LCD *self);
};

void init_screen(struct LCD *screen);
#endif /* __HD44780_H */
