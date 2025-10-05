#include "sys/sys.h"
#include "delay/delay.h"
#include "fmc.h"
#include "lcd.h"
#include "lcd_ex.h"
#include "lcd_font.h"

uint32_t g_point_color = BLACK;
uint32_t g_back_color  = WHITE;

_lcd_dev lcddev;

static uint16_t lcd_rd_data(void);

void LCD_Init(SRAM_HandleTypeDef         hsram,
              FMC_NORSRAM_TimingTypeDef *p_ext_timeing) {
  GPIO_InitTypeDef gpio_init_struct;

  LCD_CS_GPIO_CLK_ENABLE();
  LCD_WR_GPIO_CLK_ENABLE();
  LCD_RD_GPIO_CLK_ENABLE();
  LCD_RS_GPIO_CLK_ENABLE();
  LCD_BL_GPIO_CLK_ENABLE();

  gpio_init_struct.Pin       = LCD_CS_GPIO_PIN;
  gpio_init_struct.Mode      = GPIO_MODE_AF_PP;
  gpio_init_struct.Pull      = GPIO_PULLUP;
  gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;
  gpio_init_struct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(LCD_CS_GPIO_PORT, &gpio_init_struct);

  gpio_init_struct.Pin = LCD_WR_GPIO_PIN;
  HAL_GPIO_Init(LCD_WR_GPIO_PORT, &gpio_init_struct);

  gpio_init_struct.Pin = LCD_RD_GPIO_PIN;
  HAL_GPIO_Init(LCD_RD_GPIO_PORT, &gpio_init_struct);

  gpio_init_struct.Pin = LCD_RS_GPIO_PIN;
  HAL_GPIO_Init(LCD_RS_GPIO_PORT, &gpio_init_struct);

  gpio_init_struct.Pin  = LCD_BL_GPIO_PIN;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(LCD_BL_GPIO_PORT, &gpio_init_struct);

  lcd_mpu_config();
  lcd_read_id();

  if (lcddev.id == 0x7789) {
    lcd_ex_st7789_reginit();
  } else if (lcddev.id == 0x9341) {
    lcd_ex_ili9341_reginit();
  } else if (lcddev.id == 0x5310) {
    lcd_ex_nt35310_reginit();
  } else if (lcddev.id == 0x7796) {
    lcd_ex_st7796_reginit();
  } else if (lcddev.id == 0x5510) {
    lcd_ex_nt35510_reginit();
  } else if (lcddev.id == 0x9806) {
    lcd_ex_ili9806_reginit();
  } else if (lcddev.id == 0x1963) {
    lcd_ex_ssd1963_reginit();
    lcd_ssd_backlight_set(100);
  }

  if (lcddev.id == 0x9341) {
    p_ext_timeing->AddressSetupTime = 4; // 4 fmc_ker_ck = 18 ns
    p_ext_timeing->DataSetupTime    = 4; // 4 fmc_ker_ck = 18 ns
    FMC_NORSRAM_Extended_Timing_Init(hsram.Extended, p_ext_timeing,
                                     hsram.Init.NSBank,
                                     hsram.Init.ExtendedMode);
  } else if (lcddev.id == 0x7789) {
    p_ext_timeing->AddressSetupTime = 5; // 5 fmc_ker_ck = 22.5 ns
    p_ext_timeing->DataSetupTime    = 5; // 5 fmc_ker_ck = 22.5 ns
    FMC_NORSRAM_Extended_Timing_Init(hsram.Extended, p_ext_timeing,
                                     hsram.Init.NSBank,
                                     hsram.Init.ExtendedMode);
  } else if (lcddev.id == 0x1963) {
    p_ext_timeing->AddressSetupTime = 7; // 7 fmc_ker_ck = 31.5 ns
    p_ext_timeing->DataSetupTime    = 7; // 7 fmc_ker_ck = 31.5 ns
    FMC_NORSRAM_Extended_Timing_Init(hsram.Extended, p_ext_timeing,
                                     hsram.Init.NSBank,
                                     hsram.Init.ExtendedMode);
  } else if (lcddev.id == 0x5310 || lcddev.id == 0x5510 ||
             lcddev.id == 0x7796 || lcddev.id == 0x9806) {
    p_ext_timeing->AddressSetupTime = 3; // 3 fmc_ker_ck = 13.5 ns
    p_ext_timeing->DataSetupTime    = 3; // 3 fmc_ker_ck = 13.5 ns
    FMC_NORSRAM_Extended_Timing_Init(hsram.Extended, p_ext_timeing,
                                     hsram.Init.NSBank,
                                     hsram.Init.ExtendedMode);
  }

  lcd_display_dir(LCD_PORTRAIT);
  LCD_BL(1);
  lcd_clear(g_back_color);
}

void lcd_read_id(void) {
  // try 9341
  lcd_wr_regno(0xD3);
  lcddev.id = lcd_rd_data(); // dummy read
  lcddev.id = lcd_rd_data(); // 0x00
  lcddev.id = lcd_rd_data(); // 93
  lcddev.id <<= 8;
  lcddev.id |= lcd_rd_data(); // 41
  if (lcddev.id == 0x9341) {
    return;
  }

  // try ST7789
  lcd_wr_regno(0x04);
  lcddev.id = lcd_rd_data(); // dummy read
  lcddev.id = lcd_rd_data(); // 0x85
  lcddev.id = lcd_rd_data(); // 0x85
  lcddev.id <<= 8;
  lcddev.id |= lcd_rd_data(); // 0x52

  // replace 8552 ID to 7789
  if (lcddev.id == 0x8552) {
    lcddev.id = 0x7789;
  }
  if (lcddev.id == 0x7789) {
    return;
  }

  // try NT35310
  lcd_wr_regno(0xD4);
  lcddev.id = lcd_rd_data(); // dummy read
  lcddev.id = lcd_rd_data(); // 0x01
  lcddev.id = lcd_rd_data(); // 0x53
  lcddev.id <<= 8;
  lcddev.id |= lcd_rd_data(); // 0x10
  if (lcddev.id == 0x5310) {
    return;
  }

  // try ST7796
  lcd_wr_regno(0XD3);
  lcddev.id = lcd_rd_data(); // dummy read
  lcddev.id = lcd_rd_data(); // 0X00
  lcddev.id = lcd_rd_data(); // 0X77
  lcddev.id <<= 8;
  lcddev.id |= lcd_rd_data(); // 0X96
  if (lcddev.id == 0x7796) {
    return;
  }

  // try NT35510
  // send secret key (supplied by the manufacturer)
  lcd_write_reg(0xF000, 0x0055);
  lcd_write_reg(0xF001, 0x00AA);
  lcd_write_reg(0xF002, 0x0052);
  lcd_write_reg(0xF003, 0x0008);
  lcd_write_reg(0xF004, 0x0001);

  lcd_wr_regno(0xC500);      // low 8 bit
  lcddev.id = lcd_rd_data(); // 0x80
  lcddev.id <<= 8;

  lcd_wr_regno(0xC501);       // high 8 bit
  lcddev.id |= lcd_rd_data(); // 0x00
  delay_ms(5);                // wait 0XC501 reset (1963 reset cose 5ms)
  if (lcddev.id == 0x5510) {
    return;
  }

  // try ILI9806
  lcd_wr_regno(0XD3);
  lcddev.id = lcd_rd_data(); // dummy read
  lcddev.id = lcd_rd_data(); // 0X00
  lcddev.id = lcd_rd_data(); // 0X98
  lcddev.id <<= 8;
  lcddev.id |= lcd_rd_data(); // 0X06
  if (lcddev.id == 0x9806) {
    return;
  }

  // try SSD1963
  lcd_wr_regno(0xA1);
  lcddev.id = lcd_rd_data();
  lcddev.id = lcd_rd_data(); // 0x57
  lcddev.id <<= 8;
  lcddev.id |= lcd_rd_data(); // 0x61

  if (lcddev.id == 0x5761) {
    lcddev.id = 0x1963; // SSD1963 ID is 5761H
  }
}

void lcd_wr_data(volatile uint16_t data) {
  data = data; // Delay that must be inserted when using -O2 optimization
  LCD->LCD_RAM = data;
}

/**
 * @brief       LCD write register NO/address
 * @param       regno: register NO/address
 * @retval      void
 */
void lcd_wr_regno(volatile uint16_t regno) {
  regno = regno; // Delay that must be inserted when using -O2 optimization
  LCD->LCD_REG = regno;
}

/**
 * @brief       LCD write register
 * @param       regno: register NO/address
 * @param       data
 * @retval      void
 */
void lcd_write_reg(uint16_t regno, uint16_t data) {
  LCD->LCD_REG = regno;
  LCD->LCD_RAM = data;
}

/**
 * @brief       LCD read data
 * @retval      data
 */
static uint16_t lcd_rd_data(void) {
  volatile uint16_t ram;
  ram = LCD->LCD_RAM;
  return ram;
}

/**
 * @brief       LCD延时函数,仅用于部分在mdk -O1时间优化时需要设置的地方
 * @param       i:延时的数值
 * @retval      void
 */
static void lcd_opt_delay(uint32_t i) {
  while (i--); // use while(1) {__asm volatile("");} for AC6
}

/**
 * @brief       prepare write GRAM
 * @retval      void
 */
void lcd_write_ram_prepare(void) {
  LCD->LCD_REG = lcddev.wramcmd;
}

/**
 * @brief       read by coordinate
 * @param       x, y: coordinate
 * @retval      32 bit color
 */
uint32_t lcd_read_point(uint16_t x, uint16_t y) {
  uint16_t r = 0, g = 0, b = 0;

  if (x >= lcddev.width || y >= lcddev.height) {
    return 0; // overflow
  }

  lcd_set_cursor(x, y);

  if (lcddev.id == 0x5510) {
    lcd_wr_regno(0x2E00); // 5510 send GRAM
  } else {
    lcd_wr_regno(0x2E); // 9341/5310/1963/7789/7796/9806 send GRAM
  }

  r = lcd_rd_data(); // dummy read

  if (lcddev.id == 0x1963) {
    return r;
  }

  lcd_opt_delay(2);
  r = lcd_rd_data(); // real color

  if (lcddev.id == 0x7796) {
    return r;
  }
  // 9341/5310/5510/7789/9806 need read twice
  lcd_opt_delay(2);
  b = lcd_rd_data();
  g = r & 0xFF; // 9341/5310/5510/7789/9806, R and G, each 8 bit

  g <<= 8;
  return (((r >> 11) << 11) | ((g >> 10) << 5) | (b >> 11));
}

/**
 * @brief       LCD open display
 * @retval      void
 */
void lcd_display_on(void) {
  if (lcddev.id == 0X5510) {
    lcd_wr_regno(0X2900);
  } else {
    // 9341/5310/1963/7789/7796/9806
    lcd_wr_regno(0X29);
  }
}

/**
 * @brief       LCD close display
 * @retval      void
 */
void lcd_display_off(void) {
  if (lcddev.id == 0x5510) {
    lcd_wr_regno(0x2800);
  } else {
    // 9341/5310/1963/7789/7796/980
    lcd_wr_regno(0x28);
  }
}

/**
 * @brief       set cursor coordinate (not available for RGB LCD)
 * @param       x, y: coordinate
 * @retval      void
 */
void lcd_set_cursor(uint16_t x, uint16_t y) {
  if (lcddev.id == 0x1963) {
    if (lcddev.dir == LCD_PORTRAIT) {
      x = lcddev.width - 1 - x;
      lcd_wr_regno(lcddev.setxcmd);
      lcd_wr_data(0);
      lcd_wr_data(0);
      lcd_wr_data(x >> 8);
      lcd_wr_data(x & 0xFF);
    } else {
      lcd_wr_regno(lcddev.setxcmd);
      lcd_wr_data(x >> 8);
      lcd_wr_data(x & 0xFF);
      lcd_wr_data((lcddev.width - 1) >> 8);
      lcd_wr_data((lcddev.width - 1) & 0xFF);
    }

    lcd_wr_regno(lcddev.setycmd);
    lcd_wr_data(y >> 8);
    lcd_wr_data(y & 0xFF);
    lcd_wr_data((lcddev.height - 1) >> 8);
    lcd_wr_data((lcddev.height - 1) & 0xFF);
  } else if (lcddev.id == 0x5510) {
    lcd_wr_regno(lcddev.setxcmd);
    lcd_wr_data(x >> 8);
    lcd_wr_regno(lcddev.setxcmd + 1);
    lcd_wr_data(x & 0xFF);
    lcd_wr_regno(lcddev.setycmd);
    lcd_wr_data(y >> 8);
    lcd_wr_regno(lcddev.setycmd + 1);
    lcd_wr_data(y & 0xFF);
  } else {
    // 9341/5310/7789
    lcd_wr_regno(lcddev.setxcmd);
    lcd_wr_data(x >> 8);
    lcd_wr_data(x & 0xFF);
    lcd_wr_regno(lcddev.setycmd);
    lcd_wr_data(y >> 8);
    lcd_wr_data(y & 0xFF);
  }
}

/**
 * @brief       set LCD scan direction (not available for RGB LCD)
 * @note        9341/5310/5510/1963/7789 tested
 *              Other functions may be affected by this function's settings
 * (especially 9341). Therefore, it is generally recommended to set it to
 * L2R_U2D. Setting it to other scan modes may cause abnormal display.
 *
 * @param       dir: 0~7
 * @see         lcd.h
 * @retval      void
 */
void lcd_scan_dir(uint8_t dir) {
  uint16_t regval = 0;
  uint16_t dirreg = 0;
  uint16_t temp;

  // When in landscape mode, do not alter the scanning direction for 1963!
  // When in portrait mode, 1963 changes direction
  // (this applies only to the special handling of 1963; it has no effect on
  // other driver ICs).
  if ((lcddev.dir == 1 && lcddev.id != 0x1963) ||
      (lcddev.dir == 0 && lcddev.id == 0x1963)) {
    switch (dir) {
    case 0:
      dir = 6;
      break;

    case 1:
      dir = 7;
      break;

    case 2:
      dir = 4;
      break;

    case 3:
      dir = 5;
      break;

    case 4:
      dir = 1;
      break;

    case 5:
      dir = 0;
      break;

    case 6:
      dir = 3;
      break;

    case 7:
      dir = 2;
      break;
    }
  }

  // Set the values of bits 5, 6, and 7 in the 0x36/0x3600 register according to
  // the scanning method.
  switch (dir) {
  case L2R_U2D:
    regval |= (0 << 7) | (0 << 6) | (0 << 5);
    break;

  case L2R_D2U:
    regval |= (1 << 7) | (0 << 6) | (0 << 5);
    break;

  case R2L_U2D:
    regval |= (0 << 7) | (1 << 6) | (0 << 5);
    break;

  case R2L_D2U:
    regval |= (1 << 7) | (1 << 6) | (0 << 5);
    break;

  case U2D_L2R:
    regval |= (0 << 7) | (0 << 6) | (1 << 5);
    break;

  case U2D_R2L:
    regval |= (0 << 7) | (1 << 6) | (1 << 5);
    break;

  case D2U_L2R:
    regval |= (1 << 7) | (0 << 6) | (1 << 5);
    break;

  case D2U_R2L:
    regval |= (1 << 7) | (1 << 6) | (1 << 5);
    break;
  }

  // For the vast majority of driver ICs, controlled by the 0x36 register
  dirreg = 0x36;

  // For the 5510, the registers differ from those of other driver ICs.
  if (lcddev.id == 0x5510) {
    dirreg = 0x3600;
  }

  // 9341 & 7789 & 7796 must set the BGR bit
  if (lcddev.id == 0x9341 || lcddev.id == 0x7789 || lcddev.id == 0x7796) {
    regval |= 0x08;
  }

  lcd_write_reg(dirreg, regval);

  // 1963 No coordinate processing
  if (lcddev.id != 0x1963) {
    if (regval & 0x20) {
      // Swap X and Y
      if (lcddev.width < lcddev.height) {
        temp          = lcddev.width;
        lcddev.width  = lcddev.height;
        lcddev.height = temp;
      }
    } else {
      // Swap X and Y
      if (lcddev.width > lcddev.height) {
        temp          = lcddev.width;
        lcddev.width  = lcddev.height;
        lcddev.height = temp;
      }
    }
  }

  // Set the display area (window) size
  if (lcddev.id == 0x5510) {
    lcd_wr_regno(lcddev.setxcmd);
    lcd_wr_data(0);
    lcd_wr_regno(lcddev.setxcmd + 1);
    lcd_wr_data(0);
    lcd_wr_regno(lcddev.setxcmd + 2);
    lcd_wr_data((lcddev.width - 1) >> 8);
    lcd_wr_regno(lcddev.setxcmd + 3);
    lcd_wr_data((lcddev.width - 1) & 0xFF);
    lcd_wr_regno(lcddev.setycmd);
    lcd_wr_data(0);
    lcd_wr_regno(lcddev.setycmd + 1);
    lcd_wr_data(0);
    lcd_wr_regno(lcddev.setycmd + 2);
    lcd_wr_data((lcddev.height - 1) >> 8);
    lcd_wr_regno(lcddev.setycmd + 3);
    lcd_wr_data((lcddev.height - 1) & 0xFF);
  } else {
    lcd_wr_regno(lcddev.setxcmd);
    lcd_wr_data(0);
    lcd_wr_data(0);
    lcd_wr_data((lcddev.width - 1) >> 8);
    lcd_wr_data((lcddev.width - 1) & 0xFF);
    lcd_wr_regno(lcddev.setycmd);
    lcd_wr_data(0);
    lcd_wr_data(0);
    lcd_wr_data((lcddev.height - 1) >> 8);
    lcd_wr_data((lcddev.height - 1) & 0xFF);
  }
}

/**
 * @brief       draw point
 * @param       x, y: coordinate
 * @param       color: point color(32 bit)
 * @retval      void
 */
void lcd_draw_point(uint16_t x, uint16_t y, uint32_t color) {
  lcd_set_cursor(x, y);
  lcd_write_ram_prepare();
  LCD->LCD_RAM = color;
}

/**
 * @brief       SSD1963 backlight setting
 * @param       pwm: backlight level, 0~100
 * @retval      void
 */
void lcd_ssd_backlight_set(uint8_t pwm) {
  lcd_wr_regno(0xBE);      // Configure PWM Output
  lcd_wr_data(0x05);       // 1. Set PWM Frequency
  lcd_wr_data(pwm * 2.55); // 2. Set PWM Duty Cycle
  lcd_wr_data(0x01);       // 3. Set C
  lcd_wr_data(0xFF);       // 4. Set D
  lcd_wr_data(0x00);       // 5. Set E
  lcd_wr_data(0x00);       // 6. Set F
}

/**
 * @brief       set LCD display direction
 * @param       dir: 0 = portrait; 1 = landscape
 * @retval      void
 */
void lcd_display_dir(uint8_t dir) {
  lcddev.dir = dir;

  lcddev.width = 240;
  if (dir == LCD_PORTRAIT) {
    lcddev.height = 320;

    if (lcddev.id == 0x5510) {
      lcddev.wramcmd = 0x2C00;
      lcddev.setxcmd = 0x2A00;
      lcddev.setycmd = 0x2B00;
      lcddev.width   = 480;
      lcddev.height  = 800;
    } else if (lcddev.id == 0x1963) {
      lcddev.wramcmd = 0x2C;
      lcddev.setxcmd = 0x2B;
      lcddev.setycmd = 0x2A;
      lcddev.width   = 480;
      lcddev.height  = 800;
    } else {
      // 9341/5310/7789/7796/9806
      lcddev.wramcmd = 0x2C;
      lcddev.setxcmd = 0x2A;
      lcddev.setycmd = 0x2B;
    }

    if (lcddev.id == 0x5310 || lcddev.id == 0x7796) {
      lcddev.width  = 320;
      lcddev.height = 480;
    }
    if (lcddev.id == 0X9806) {
      lcddev.width  = 480;
      lcddev.height = 800;
    }
  } else {
    // LCD_LANDSCAPE
    lcddev.width  = 320;
    lcddev.height = 240;

    if (lcddev.id == 0x5510) {
      lcddev.wramcmd = 0x2C00;
      lcddev.setxcmd = 0x2A00;
      lcddev.setycmd = 0x2B00;
      lcddev.width   = 800;
      lcddev.height  = 480;
    } else if (lcddev.id == 0x1963 || lcddev.id == 0x9806) {
      lcddev.wramcmd = 0x2C;
      lcddev.setxcmd = 0x2A;
      lcddev.setycmd = 0x2B;
      lcddev.width   = 800;
      lcddev.height  = 480;
    } else {
      // 9341/5310/7789/7796
      lcddev.wramcmd = 0x2C;
      lcddev.setxcmd = 0x2A;
      lcddev.setycmd = 0x2B;
    }

    if (lcddev.id == 0x5310 || lcddev.id == 0x7796) {
      lcddev.width  = 480;
      lcddev.height = 320;
    }
  }

  lcd_scan_dir(DFT_SCAN_DIR);
}

/**
 * @brief       set window(not available for RGB LCS), and set point
 * coordinate(sx,sy).
 * @param       sx, sy: window coordinate
 * @param       width, height: window width and height
 * @retval      void
 */
void lcd_set_window(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height) {
  uint16_t twidth, theight;
  twidth  = sx + width - 1;
  theight = sy + height - 1;

  if (lcddev.id == 0x1963 && lcddev.dir != 1) {
    sx     = lcddev.width - width - sx;
    height = sy + height - 1;
    lcd_wr_regno(lcddev.setxcmd);
    lcd_wr_data(sx >> 8);
    lcd_wr_data(sx & 0xFF);
    lcd_wr_data((sx + width - 1) >> 8);
    lcd_wr_data((sx + width - 1) & 0xFF);
    lcd_wr_regno(lcddev.setycmd);
    lcd_wr_data(sy >> 8);
    lcd_wr_data(sy & 0xFF);
    lcd_wr_data(height >> 8);
    lcd_wr_data(height & 0xFF);
  } else if (lcddev.id == 0x5510) {
    lcd_wr_regno(lcddev.setxcmd);
    lcd_wr_data(sx >> 8);
    lcd_wr_regno(lcddev.setxcmd + 1);
    lcd_wr_data(sx & 0xFF);
    lcd_wr_regno(lcddev.setxcmd + 2);
    lcd_wr_data(twidth >> 8);
    lcd_wr_regno(lcddev.setxcmd + 3);
    lcd_wr_data(twidth & 0xFF);
    lcd_wr_regno(lcddev.setycmd);
    lcd_wr_data(sy >> 8);
    lcd_wr_regno(lcddev.setycmd + 1);
    lcd_wr_data(sy & 0xFF);
    lcd_wr_regno(lcddev.setycmd + 2);
    lcd_wr_data(theight >> 8);
    lcd_wr_regno(lcddev.setycmd + 3);
    lcd_wr_data(theight & 0xFF);
  } else {
    // 9341/5310/7789/1963/7796/9806
    lcd_wr_regno(lcddev.setxcmd);
    lcd_wr_data(sx >> 8);
    lcd_wr_data(sx & 0xFF);
    lcd_wr_data(twidth >> 8);
    lcd_wr_data(twidth & 0xFF);
    lcd_wr_regno(lcddev.setycmd);
    lcd_wr_data(sy >> 8);
    lcd_wr_data(sy & 0xFF);
    lcd_wr_data(theight >> 8);
    lcd_wr_data(theight & 0xFF);
  }
}

/**
 * @brief       set MPU region
 * @param       void
 * @retval      void
 */
void lcd_mpu_config(void) {
  MPU_Region_InitTypeDef mpu_init_struct;

  // Disable the MPU before configuring it, then enable the MPU after
  // configuration is complete.
  HAL_MPU_Disable();
  // The external SRAM is region 0, with a size of 2MB. This region is
  // read-write.
  mpu_init_struct.Enable           = MPU_REGION_ENABLE;
  mpu_init_struct.Number           = LCD_REGION_NUMBER;
  mpu_init_struct.BaseAddress      = LCD_ADDRESS_START;
  mpu_init_struct.Size             = LCD_REGION_SIZE;
  mpu_init_struct.SubRegionDisable = 0X00;
  mpu_init_struct.TypeExtField     = MPU_TEX_LEVEL0;
  mpu_init_struct.AccessPermission = MPU_REGION_FULL_ACCESS;
  mpu_init_struct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  mpu_init_struct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  mpu_init_struct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  mpu_init_struct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
  HAL_MPU_ConfigRegion(&mpu_init_struct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief       clear screen
 * @param       color
 * @retval      void
 */
void lcd_clear(uint16_t color) {
  uint32_t index      = 0;
  uint32_t totalpoint = lcddev.width;

  totalpoint *= lcddev.height;
  lcd_set_cursor(0x00, 0x0000);
  lcd_write_ram_prepare();

  for (index = 0; index < totalpoint; index++) {
    LCD->LCD_RAM = color;
  }
}

/**
 * @brief       Fill a single color within a specified rectangle
 * @param       (sx,sy),(ex,ey): rectangle diagonal coordinates
 * @param       color: 32 bit
 * @retval      void
 */
void lcd_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey,
              uint32_t color) {
  uint16_t i, j;
  uint16_t xlen = 0;
  xlen          = ex - sx + 1;

  for (i = sy; i <= ey; i++) {
    lcd_set_cursor(sx, i);
    lcd_write_ram_prepare();

    for (j = 0; j < xlen; j++) {
      LCD->LCD_RAM = color; // display color
    }
  }
}

/**
 * @brief       Fill the designated area with the specified color block.
 * @param       (sx,sy),(ex,ey): rectangle diagonal coordinates
 * @param       color: The starting address of the color array to be filled
 * @retval      void
 */
void lcd_color_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey,
                    uint16_t *color) {
  uint16_t height, width;
  uint16_t i, j;

  width  = ex - sx + 1;
  height = ey - sy + 1;

  for (i = 0; i < height; i++) {
    lcd_set_cursor(sx, sy + i);
    lcd_write_ram_prepare();

    for (j = 0; j < width; j++) {
      LCD->LCD_RAM = color[i * width + j];
    }
  }
}

/**
 * @brief       draw line
 * @param       x1,y1: start coordinate
 * @param       x2,y2: end coordinate
 * @param       color: line color
 * @retval      void
 */
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                   uint16_t color) {
  uint16_t t;
  int      xerr = 0, yerr = 0, delta_x, delta_y, distance;
  int      incx, incy, row, col;
  delta_x = x2 - x1;
  delta_y = y2 - y1;
  row     = x1;
  col     = y1;

  if (delta_x > 0) {
    incx = 1; // Set single-step direction
  } else if (delta_x == 0) {
    incx = 0; // Vertical line
  } else {
    incx    = -1;
    delta_x = -delta_x;
  }

  if (delta_y > 0) {
    incy = 1;
  } else if (delta_y == 0) {
    incy = 0; // Horizon line
  } else {
    incy    = -1;
    delta_y = -delta_y;
  }

  if (delta_x > delta_y) {
    distance = delta_x;
  } else {
    distance = delta_y;
  }

  for (t = 0; t <= distance + 1; t++) {
    lcd_draw_point(row, col, color);
    xerr += delta_x;
    yerr += delta_y;

    if (xerr > distance) {
      xerr -= distance;
      row += incx;
    }

    if (yerr > distance) {
      yerr -= distance;
      col += incy;
    }
  }
}

/**
 * @brief       Draw a horizontal line
 * @param       x,y   : start coordinate
 * @param       len   : line length
 * @param       color : line color
 * @retval      void
 */
void lcd_draw_hline(uint16_t x, uint16_t y, uint16_t len, uint16_t color) {
  if ((len == 0) || (x > lcddev.width) || (y > lcddev.height)) {
    return;
  }

  lcd_fill(x, y, x + len - 1, y, color);
}

/**
 * @brief       draw rectangle
 * @param       x1,y1: start coordinate
 * @param       x2,y2: end coordinate
 * @param       color: rectangle color
 * @retval      void
 */
void lcd_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                        uint16_t color) {
  lcd_draw_line(x1, y1, x2, y1, color);
  lcd_draw_line(x1, y1, x1, y2, color);
  lcd_draw_line(x1, y2, x2, y2, color);
  lcd_draw_line(x2, y1, x2, y2, color);
}

/**
 * @brief       draw circle
 * @param       x0,y0 : Center coordinate of a circle
 * @param       r     : radius
 * @param       color : circle color
 * @retval      void
 */
void lcd_draw_circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color) {
  int a, b;
  int di;

  a  = 0;
  b  = r;
  di = 3 - (r << 1);

  while (a <= b) {
    lcd_draw_point(x0 + a, y0 - b, color); /* 5 */
    lcd_draw_point(x0 + b, y0 - a, color); /* 0 */
    lcd_draw_point(x0 + b, y0 + a, color); /* 4 */
    lcd_draw_point(x0 + a, y0 + b, color); /* 6 */
    lcd_draw_point(x0 - a, y0 + b, color); /* 1 */
    lcd_draw_point(x0 - b, y0 + a, color);
    lcd_draw_point(x0 - a, y0 - b, color); /* 2 */
    lcd_draw_point(x0 - b, y0 - a, color); /* 7 */
    a++;

    // Bresenham
    if (di < 0) {
      di += 4 * a + 6;
    } else {
      di += 10 + 4 * (a - b);
      b--;
    }
  }
}

/**
 * @brief       file circle
 * @param       x,y  : Center coordinate of a circle
 * @param       r    : radius
 * @param       color: circle color
 * @retval      void
 */
void lcd_fill_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color) {
  uint32_t i;
  uint32_t imax  = ((uint32_t)r * 707) / 1000 + 1;
  uint32_t sqmax = (uint32_t)r * (uint32_t)r + (uint32_t)r / 2;
  uint32_t xr    = r;

  lcd_draw_hline(x - r, y, 2 * r, color);

  for (i = 1; i <= imax; i++) {
    if ((i * i + xr * xr) > sqmax) {
      // draw lines from outside
      if (xr > imax) {
        lcd_draw_hline(x - i + 1, y + xr, 2 * (i - 1), color);
        lcd_draw_hline(x - i + 1, y - xr, 2 * (i - 1), color);
      }

      xr--;
    }

    // draw lines from inside (center)
    lcd_draw_hline(x - xr, y + i, 2 * xr, color);
    lcd_draw_hline(x - xr, y - i, 2 * xr, color);
  }
}

/**
 * @brief       Draw a character at the specified position
 * @param       x,y  : coordinate
 * @param       chr  : char to display
 * @param       size : font size: 12/16/24/32
 * @param       mode : Overlay mode (1); Non-overlay mode (0);
 * @param       color: font color
 * @retval      void
 */
void lcd_show_char(uint16_t x, uint16_t y, unsigned char chr, uint8_t size, uint8_t mode,
                   uint16_t color) {
  uint8_t  temp, t1, t;
  uint16_t y0    = y;
  uint8_t  csize = 0;
  uint8_t *pfont = 0;

  // The number of bytes occupied by the bitmap set corresponding to a single
  // character in a font.
  csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2);
  // Obtain the offset value (the ASCII character set starts with space when
  // taking the modulus, so -' ' corresponds to the character in the character
  // set)
  chr = chr - ' ';

  switch (size) {
  case 12:
    pfont = (uint8_t *)asc2_1206[chr];
    break;

  case 16:
    pfont = (uint8_t *)asc2_1608[chr];
    break;

  case 24:
    pfont = (uint8_t *)asc2_2412[chr];
    break;

  case 32:
    pfont = (uint8_t *)asc2_3216[chr];
    break;

  default:
    return;
  }

  for (t = 0; t < csize; t++) {
    temp = pfont[t]; // Retrieve the bitmap data of a character

    for (t1 = 0; t1 < 8; t1++) // One byte equals eight points
    {
      if (temp & 0x80) // Effective point, needs to be displayed
      {
        lcd_draw_point(x, y, color); // Draw the point to display it.
      } else if (mode == 0) {
        // Invalid point, not displayed
        // Painting the background color effectively hides this point (note that
        // the background color is controlled by a global variable).
        lcd_draw_point(x, y, g_back_color);
      }

      temp <<= 1; // Shift to obtain the state of the next bit
      y++;

      if (y >= lcddev.height)
        return; // overflow

      if ((y - y0) == size) {
        y = y0;
        x++;

        if (x >= lcddev.width) {
          return; // x overflow
        }

        break;
      }
    }
  }
}

/**
 * @brief       Exponential function
 * @param       m: base number
 * @param       n: exponent
 * @retval      m^n
 */
static uint32_t lcd_pow(uint8_t m, uint8_t n) {
  uint32_t result = 1;

  while (n--) {
    result *= m;
  }

  return result;
}

/**
 * @brief       Display numbers
 * @param       x,y : 起始坐标
 * @param       num : 数值(0 ~ 2^32)
 * @param       len : 显示数字的位数
 * @param       size: 选择字体 12/16/24/32
 * @retval      void
 */
void lcd_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len,
                  uint8_t size, uint16_t color) {
  uint8_t t, temp;
  uint8_t enshow = 0;

  for (t = 0; t < len; t++) /* 按总显示位数循环 */
  {
    temp = (num / lcd_pow(10, len - t - 1)) % 10; /* 获取对应位的数字 */

    if (enshow == 0 && t < (len - 1)) /* 没有使能显示,且还有位要显示 */
    {
      if (temp == 0) {
        lcd_show_char(x + (size / 2) * t, y, ' ', size, 0,
                      color); /* 显示空格,占位 */
        continue;             /* 继续下个一位 */
      } else {
        enshow = 1; /* 使能显示 */
      }
    }

    lcd_show_char(x + (size / 2) * t, y, temp + '0', size, 0,
                  color); /* 显示字符 */
  }
}

/**
 * @brief       扩展显示len个数字(高位是0也显示)
 * @param       x,y : start coordinate
 * @param       num : 0 ~ 2^32
 * @param       len : length
 * @param       size: font size: 12/16/24/32
 * @param       mode: display mode
 *              [7]:0, do not fill; 1, fill with 0.
 *              [6:1]: preserve
 *              [0]: 0, do not display overlay; 1, display overlay.
 * @retval      void
 */
void lcd_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len,
                   uint8_t size, uint8_t mode, uint16_t color) {
  uint8_t t, temp;
  uint8_t enshow = 0;

  for (t = 0; t < len; t++) {
    temp = (num / lcd_pow(10, len - t - 1)) % 10;

    if (enshow == 0 && t < (len - 1)) {
      if (temp == 0) {
        if (mode & 0x80) {
          // High-order bits require padding with zeros.
          lcd_show_char(x + (size / 2) * t, y, '0', size, mode & 0x01, color);
        } else {
          lcd_show_char(x + (size / 2) * t, y, ' ', size, mode & 0x01, color);
        }

        continue;
      } else {
        enshow = 1;
      }
    }

    lcd_show_char(x + (size / 2) * t, y, temp + '0', size, mode & 0x01, color);
  }
}

/**
 * @brief       display string
 * @param       x,y         : start coordinate
 * @param       width,height: area size
 * @param       size        : font size: 12/16/24/32
 * @param       p           : Start address of the string
 * @retval      void
 */
void lcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                     uint8_t size, unsigned char *p, uint16_t color) {
  uint8_t x0 = x;

  width += x;
  height += y;

  while ((*p <= '~') && (*p >= ' ')) {
    if (x >= width) {
      x = x0;
      y += size;
    }

    if (y >= height) {
      break;
    }

    lcd_show_char(x, y, *p, size, 0, color);
    x += size / 2;
    p++;
  }
}

void lcd_show_multiline_string(uint16_t x, uint16_t y, uint16_t width,
                               uint16_t height, uint8_t size, const unsigned char *p,
                               uint16_t color) {
  uint16_t x0    = x;
  uint16_t x_end = x + width;
  uint16_t y_end = y + height;

  while (*p) {
    char c = *p++;
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      x = x0;
      y += size;
      if (y >= y_end) {
        break;
      }
      continue;
    }

    if (c < ' ' || c > '~') {
      continue;
    }

    if (x >= x_end) {
      x = x0;
      y += size;
      if (y >= y_end) {
        break;
      }
    }

    lcd_show_char(x, y, c, size, 0, color);
    x += size / 2;
  }
}

void lcd_draw_bitmap(uint16_t x, uint16_t y, const unsigned char *bitmap,
                     uint16_t width, uint16_t height, uint8_t scale,
                     uint32_t color) {
  uint32_t byte_width = (width + 7) / 8;
  for (uint16_t j = 0; j < height; j++) {
    for (uint16_t i = 0; i < width; i++) {
      uint32_t byte_index = j * byte_width + (i / 8);
      uint8_t  bit_mask   = 0x80 >> (i % 8);
      if (bitmap[byte_index] & bit_mask) {
        uint16_t x0 = x + i * scale;
        uint16_t y0 = y + j * scale;
        lcd_fill(x0, y0, x0 + scale, y0 + scale, color);
      }
    }
  }
}
