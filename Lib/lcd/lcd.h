#ifndef STM32_LIB_LCD_H
#define STM32_LIB_LCD_H

#include "sys/sys.h"

// LCD_D0~D15 in LCD_Init()

// LCD write, PD5 -> FMC_NWE
#define LCD_WR_GPIO_PORT GPIOD
#define LCD_WR_GPIO_PIN GPIO_PIN_5
#define LCD_WR_GPIO_CLK_ENABLE()                                               \
  do {                                                                         \
    __HAL_RCC_GPIOD_CLK_ENABLE();                                              \
  } while (0)

// LCD read, PD4 -> FMC_NOE
#define LCD_RD_GPIO_PORT GPIOD
#define LCD_RD_GPIO_PIN GPIO_PIN_4
#define LCD_RD_GPIO_CLK_ENABLE()                                               \
  do {                                                                         \
    __HAL_RCC_GPIOD_CLK_ENABLE();                                              \
  } while (0)

// LCD backlight, PB5 -> FMC_NWE
#define LCD_BL_GPIO_PORT GPIOB
#define LCD_BL_GPIO_PIN GPIO_PIN_5
#define LCD_BL_GPIO_CLK_ENABLE()                                               \
  do {                                                                         \
    __HAL_RCC_GPIOB_CLK_ENABLE();                                              \
  } while (0)

// LCD Chip Select, PD7 -> FMC_NE1
#define LCD_CS_GPIO_PORT GPIOD
#define LCD_CS_GPIO_PIN GPIO_PIN_7
#define LCD_CS_GPIO_CLK_ENABLE()                                               \
  do {                                                                         \
    __HAL_RCC_GPIOD_CLK_ENABLE();                                              \
  } while (0)

// LCD Register Select, PD13 -> FMC_A18
#define LCD_RS_GPIO_PORT GPIOD
#define LCD_RS_GPIO_PIN GPIO_PIN_13
#define LCD_RS_GPIO_CLK_ENABLE()                                               \
  do {                                                                         \
    __HAL_RCC_GPIOD_CLK_ENABLE();                                              \
  } while (0)

// FMC Chip Select, 1-4
#define LCD_FMC_NEX 1
// FMC LCD Register Select, 0-25
#define LCD_FMC_AX 18

/******************************************************************************/

typedef struct {
  uint16_t width;
  uint16_t height;
  uint16_t id;      // LCD ID
  uint8_t  dir;     // Orientation control: 0 = portrait; 1 = landscape
  uint16_t wramcmd; // write gram cmd
  uint16_t setxcmd; // set x coordinate cmd
  uint16_t setycmd; // set y coordinate cmd
} _lcd_dev;

extern _lcd_dev lcddev;

// LCD pen color, default red
extern uint32_t g_point_color;
// LCD background color, default white
extern uint32_t g_back_color;

// LCD backlight
#define LCD_BL(x)                                                              \
  do {                                                                         \
    x ? HAL_GPIO_WritePin(LCD_BL_GPIO_PORT, LCD_BL_GPIO_PIN, GPIO_PIN_SET)     \
      : HAL_GPIO_WritePin(LCD_BL_GPIO_PORT, LCD_BL_GPIO_PIN, GPIO_PIN_RESET);  \
  } while (0)

// LCD address
typedef struct {
  volatile uint16_t LCD_REG;
  volatile uint16_t LCD_RAM;
} LCD_TypeDef;

// LCD MPU protected
#define LCD_REGION_NUMBER MPU_REGION_NUMBER0  // LCD region, use MPU region 0
#define LCD_ADDRESS_START (0X60000000)        // LCD region start address
#define LCD_REGION_SIZE MPU_REGION_SIZE_256MB // LCD region size

/* Detailed calculation method of LCD_BASE:
 * We usually use FMC Bank1 to drive TFT LCD (MCU interface type).
 * The total address range of Bank1 is 256MB, divided equally into 4 parts:
 * Bank1 (FMC_NE1) address range: 0x6000 0000 ~ 0x63FF FFFF
 * Bank2 (FMC_NE2) address range: 0x6400 0000 ~ 0x67FF FFFF
 * Bank3 (FMC_NE3) address range: 0x6800 0000 ~ 0x6BFF FFFF
 * Bank4 (FMC_NE4) address range: 0x6C00 0000 ~ 0x6FFF FFFF
 *
 * We need to choose the proper chip select (connected to LCD_CS)
 * and address line (connected to LCD_RS) according to the hardware wiring.
 * The Apollo H743 development board uses FMC_NE1 connected to LCD_CS,
 * FMC_A18 connected to LCD_RS, with 16-bit data bus.
 * The calculation is as follows:
 * The base address of FMC_NE1 is: 0x6000 0000;
 * Base address of NEX (x=1/2/3/4): 0x6000 0000 + (0x400 0000 * (x - 1))
 * FMC_A18 corresponds to the address value: 2^18 * 2 = 0x80000;
 * FMC_Ay corresponds to address (y = 0 ~ 25): 2^y * 2
 *
 * LCD->LCD_REG corresponds to LCD_RS = 0 (LCD register)
 * LCD->LCD_RAM corresponds to LCD_RS = 1 (LCD data)
 * So the address of LCD->LCD_RAM is:  0x6000 0000 + 2^18 * 2 = 0x6008 0000
 * The address of LCD->LCD_REG can be any address before LCD->LCD_RAM.
 * Since we use a structure to manage LCD_REG and LCD_RAM (REG first, RAM next,
 * both 16-bit wide) The base address of the structure (LCD_BASE) = LCD_RAM - 2
 * = 0x6008 0000 -2
 *
 * A more general calculation formula ((Chip select FMC_NEX) X=1/2/3/4, (RS
 * address line FMC_Ay) y=0~25): LCD_BASE = (0x6000 0000 + (0x400 0000 * (x -
 * 1))) | (2^y * 2 -2) Equivalent (using shift operation) LCD_BASE = (0x6000
 * 0000 + (0x400 0000 * (x - 1))) | ((1 << y) * 2 -2)
 */
#define LCD_BASE                                                               \
  (uint32_t)((0X60000000 + (0X4000000 * (LCD_FMC_NEX - 1))) |                  \
             (((1 << LCD_FMC_AX) * 2) - 2))
#define LCD ((LCD_TypeDef *)LCD_BASE)

/******************************************************************************/

#define LCD_PORTRAIT 0
#define LCD_LANDSCAPE 1

// Scan direction definition
#define L2R_U2D 0 // left->right, up->down
#define L2R_D2U 1 // left->right, down->up
#define R2L_U2D 2 // left->right, up->down
#define R2L_D2U 3 // right->left, down->up

#define U2D_L2R 4 // up->down, left->right
#define U2D_R2L 5 // up->down, right->left
#define D2U_L2R 6 // down->up, left->right
#define D2U_R2L 7 // down->up, right->left

#define DFT_SCAN_DIR L2R_U2D // default scan direction

#define WHITE 0xFFFF
#define BLACK 0x0000
#define RED 0xF800
#define GREEN 0x07E0
#define BLUE 0x001F
#define MAGENTA 0xF81F // BLUE + RED
#define YELLOW 0xFFE0  // GREEN + RED
#define CYAN 0x07FF    // GREEN + BLUE

#define BROWN 0xBC40      // Brown
#define BRRED 0xFC07      // Reddish brown
#define GRAY 0x8430       // Gray
#define DARKBLUE 0x01CF   // Dark blue
#define LIGHTBLUE 0x7D7C  // Light blue
#define GRAYBLUE 0x5458   // Grayish blue
#define LIGHTGREEN 0x841F // Light green
#define LGRAY 0xC618      // Light gray, window background color
#define LGRAYBLUE 0xA651  // Light grayish blue (middle layer color)
#define LBBLUE 0x2B12 // Light brownish blue (inverted color for selected item)

/******************************************************************************/

// SSD1963

#define SSD_HOR_RESOLUTION 800
#define SSD_VER_RESOLUTION 480

#define SSD_HOR_PULSE_WIDTH 1   // Horizontal pulse width
#define SSD_HOR_BACK_PORCH 46   // Horizontal back porch
#define SSD_HOR_FRONT_PORCH 210 // Horizontal front porch

#define SSD_VER_PULSE_WIDTH 1  // Vertical pulse width
#define SSD_VER_BACK_PORCH 23  // Vertical back porch
#define SSD_VER_FRONT_PORCH 22 // Vertical front porch

#define SSD_HT (SSD_HOR_RESOLUTION + SSD_HOR_BACK_PORCH + SSD_HOR_FRONT_PORCH)
#define SSD_HPS (SSD_HOR_BACK_PORCH)
#define SSD_VT (SSD_VER_RESOLUTION + SSD_VER_BACK_PORCH + SSD_VER_FRONT_PORCH)
#define SSD_VPS (SSD_VER_BACK_PORCH)

/******************************************************************************/

void LCD_Init(SRAM_HandleTypeDef hsram, FMC_NORSRAM_TimingTypeDef *p_ext_timeing);

void lcd_mpu_config(void);
void lcd_read_id(void);

void lcd_wr_data(volatile uint16_t data);   // LCD write data
void lcd_wr_regno(volatile uint16_t regno); // LCD write register NO/address
void lcd_write_reg(uint16_t regno, uint16_t data); // LCD write register value

void lcd_display_on(void);
void lcd_display_off(void);
void lcd_scan_dir(uint8_t dir);          // set scan direction
void lcd_display_dir(uint8_t dir);       // set display direction
void lcd_ssd_backlight_set(uint8_t pwm); // SSD1963 backlight

void     lcd_write_ram_prepare(void);
void     lcd_set_cursor(uint16_t x, uint16_t y);
uint32_t lcd_read_point(uint16_t x, uint16_t y);
void     lcd_draw_point(uint16_t x, uint16_t y, uint32_t color);

void lcd_clear(uint16_t color);
void lcd_fill_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color);
void lcd_draw_circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);
void lcd_draw_hline(uint16_t x, uint16_t y, uint16_t len,
                    uint16_t color); // Draw horizontal line
void lcd_set_window(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height);
void lcd_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey,
              uint32_t color); // fill rectangle with solid color
void lcd_color_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey,
                    uint16_t *color); // fill rectangle with gradient color
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                   uint16_t color);
void lcd_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                        uint16_t color);

void lcd_show_char(uint16_t x, uint16_t y, unsigned char chr, uint8_t size, uint8_t mode,
                   uint16_t color);
void lcd_show_num(uint16_t x, uint16_t y, uint32_t num, uint8_t len,
                  uint8_t size, uint16_t color);
void lcd_show_xnum(uint16_t x, uint16_t y, uint32_t num, uint8_t len,
                   uint8_t size, uint8_t mode, uint16_t color);
void lcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                     uint8_t size, unsigned char *p, uint16_t color);

void lcd_show_multiline_string(uint16_t x, uint16_t y, uint16_t width,
                          uint16_t height, uint8_t size, const unsigned char *p,
                          uint16_t color);

void lcd_draw_bitmap(uint16_t x, uint16_t y, const unsigned char *bitmap,
                           uint16_t width, uint16_t height, uint8_t scale, uint32_t color);

#endif // STM32_LIB_LCD_H