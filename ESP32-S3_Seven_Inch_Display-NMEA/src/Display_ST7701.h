// Display driver for the Waveshare ESP32-S3-Touch-LCD-7.
//
// The file keeps its original (ST7701) name because it's included everywhere, but
// this board has an ST7262 800x480 RGB panel that needs no command/SPI init:
// it's driven purely by the RGB timing signals. Reset and backlight are on the
// CH422G IO expander (see TCA9554PWR.h).
//
// The UI is the 480x480 layout from the original 4" (ESP32-S3-Touch-LCD-4) firmware. LVGL renders at
// 480x480 and
// the flush copies it into the middle of the 800x480 frame buffer; the bars
// either side stay black. Touch coordinates are shifted back the same way in
// LVGL_Driver.cpp.
#pragma once
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_rgb.h"
#include <esp_err.h>

#include "TCA9554PWR.h"
#include "LVGL_Driver.h"
#include "Touch_GT911.h"

#define Backlight_MAX   100

// ── Physical panel ──────────────────────────────────────────────────────────
#define PANEL_H_RES                               (800)
#define PANEL_V_RES                               (480)

// ── Logical UI (what LVGL and the rest of the firmware see) ─────────────────
#define ESP_PANEL_LCD_WIDTH                       (480)
#define ESP_PANEL_LCD_HEIGHT                      (480)
#define UI_X_OFFSET                               ((PANEL_H_RES - ESP_PANEL_LCD_WIDTH) / 2)   // 160
#define UI_Y_OFFSET                               ((PANEL_V_RES - ESP_PANEL_LCD_HEIGHT) / 2)  // 0

// ── RGB timing (Waveshare BOARD_WAVESHARE_ESP32_S3_TOUCH_LCD_7) ─────────────
// Waveshare uses 16 MHz, but with IDF 5 bounce buffers. On this framework
// (IDF 4.4) the DMA reads the frame buffer straight from PSRAM, and 16 MHz
// (32 MB/s) underruns: the picture flickers and tears. 12 MHz -> (800+20) x
// (480+20) = ~29 Hz refresh.
#define ESP_PANEL_LCD_RGB_TIMING_FREQ_HZ          (12 * 1000 * 1000)
#define ESP_PANEL_LCD_RGB_TIMING_HPW              (4)
#define ESP_PANEL_LCD_RGB_TIMING_HBP              (8)
#define ESP_PANEL_LCD_RGB_TIMING_HFP              (8)
#define ESP_PANEL_LCD_RGB_TIMING_VPW              (4)
#define ESP_PANEL_LCD_RGB_TIMING_VBP              (8)
#define ESP_PANEL_LCD_RGB_TIMING_VFP              (8)
#define ESP_PANEL_LCD_RGB_PCLK_ACTIVE_NEG         (1)     // falling edge
#define ESP_PANEL_LCD_RGB_DATA_WIDTH              (16)
#define ESP_PANEL_LCD_RGB_PIXEL_BITS              (16)    // RGB565

// ── RGB pins ────────────────────────────────────────────────────────────────
#define ESP_PANEL_LCD_PIN_NUM_RGB_HSYNC           (46)
#define ESP_PANEL_LCD_PIN_NUM_RGB_VSYNC           (3)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DE              (5)
#define ESP_PANEL_LCD_PIN_NUM_RGB_PCLK            (7)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DISP            (-1)
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA0           (14)  // B0
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA1           (38)  // B1
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA2           (18)  // B2
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA3           (17)  // B3
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA4           (10)  // B4
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA5           (39)  // G0
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA6           (0)   // G1
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA7           (45)  // G2
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA8           (48)  // G3
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA9           (47)  // G4
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA10          (21)  // G5
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA11          (1)   // R0
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA12          (2)   // R1
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA13          (42)  // R2
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA14          (41)  // R3
#define ESP_PANEL_LCD_PIN_NUM_RGB_DATA15          (40)  // R4

extern uint8_t LCD_Backlight;
extern esp_lcd_panel_handle_t panel_handle;
uint32_t get_vsync_count();
uint32_t get_vsync_max_gap_us();
void reset_vsync_stats();

void LCD_Init();
// Draws an area given in UI (480x480) coordinates; offsets it onto the panel.
void LCD_addWindow(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend, uint8_t* color);

// Backlight: this board only switches it on/off (CH422G EXIO2); no PWM.
// Any Light > 0 is "on".
void Backlight_Init();
void Set_Backlight(uint8_t Light);
