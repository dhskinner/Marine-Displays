// Display driver for the Waveshare ESP32-S3-Touch-LCD-7 (ST7262, 800x480 RGB).
// See Display_ST7701.h for why the file keeps its original name.
#include "Display_ST7701.h"
#include "TCA9554PWR.h"
#include <Arduino.h>
#include "esp_timer.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us

esp_lcd_panel_handle_t panel_handle = NULL;

// ── Frame-done statistics (kept for the diagnostics that print them) ────────
static volatile uint32_t g_vsync_count = 0;
static volatile uint32_t g_vsync_max_gap_us = 0;
static volatile uint32_t g_vsync_prev_us = 0;

static bool IRAM_ATTR on_frame_done(esp_lcd_panel_handle_t panel, esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx)
{
  uint32_t now = (uint32_t)esp_timer_get_time();
  if (g_vsync_prev_us != 0) {
    uint32_t delta = now - g_vsync_prev_us;
    if (delta > g_vsync_max_gap_us) g_vsync_max_gap_us = delta;
  }
  g_vsync_prev_us = now;
  g_vsync_count = g_vsync_count + 1;
  return false;
}

uint32_t get_vsync_count()      { return g_vsync_count; }
uint32_t get_vsync_max_gap_us() { return g_vsync_max_gap_us; }
void reset_vsync_stats()        { g_vsync_count = 0; g_vsync_max_gap_us = 0; g_vsync_prev_us = 0; }

// ── Panel ───────────────────────────────────────────────────────────────────
static void LCD_Reset()
{
  Set_EXIO(pin_lcd_rst(), Low);
  vTaskDelay(pdMS_TO_TICKS(10));
  Set_EXIO(pin_lcd_rst(), High);
  vTaskDelay(pdMS_TO_TICKS(100));
}

// Fill the whole 800x480 panel (including the bars either side of the UI).
static void LCD_FillPanel(uint16_t color565)
{
  if (!panel_handle) return;
  const int rows = 16;
  size_t px = (size_t)PANEL_H_RES * rows;
  uint16_t *buf = (uint16_t *)heap_caps_malloc(px * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
  if (!buf) buf = (uint16_t *)heap_caps_malloc(px * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
  if (!buf) {
    Serial.println("[DISPLAY] LCD_FillPanel: no memory for fill buffer");
    return;
  }
  for (size_t i = 0; i < px; ++i) buf[i] = color565;
  for (int y = 0; y < PANEL_V_RES; y += rows) {
    int y_end = y + rows > PANEL_V_RES ? PANEL_V_RES : y + rows;
    esp_lcd_panel_draw_bitmap(panel_handle, 0, y, PANEL_H_RES, y_end, buf);
  }
  heap_caps_free(buf);
}

// Fill the bars either side of the 480x480 UI (the panel is wider than the UI).
// LVGL never draws there, so they keep whatever was last put here.
void LCD_FillSideBars(uint16_t color565)
{
  if (!panel_handle || UI_X_OFFSET <= 0) return;
  const int w = UI_X_OFFSET;                       // 160 px each side
  const int rows = 16;
  size_t px = (size_t)w * rows;
  uint16_t *buf = (uint16_t *)heap_caps_malloc(px * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
  if (!buf) buf = (uint16_t *)heap_caps_malloc(px * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
  if (!buf) {
    Serial.println("[DISPLAY] LCD_FillSideBars: no memory for fill buffer");
    return;
  }
  for (size_t i = 0; i < px; ++i) buf[i] = color565;
  for (int y = 0; y < PANEL_V_RES; y += rows) {
    int y_end = y + rows > PANEL_V_RES ? PANEL_V_RES : y + rows;
    esp_lcd_panel_draw_bitmap(panel_handle, 0, y, w, y_end, buf);
    esp_lcd_panel_draw_bitmap(panel_handle, PANEL_H_RES - w, y, PANEL_H_RES, y_end, buf);
  }
  heap_caps_free(buf);
}

static void LCD_CreatePanel()
{
  esp_lcd_rgb_panel_config_t cfg = {};
  cfg.clk_src = LCD_CLK_SRC_PLL160M;
  cfg.timings.pclk_hz = ESP_PANEL_LCD_RGB_TIMING_FREQ_HZ;
  cfg.timings.h_res = PANEL_H_RES;
  cfg.timings.v_res = PANEL_V_RES;
  cfg.timings.hsync_pulse_width = ESP_PANEL_LCD_RGB_TIMING_HPW;
  cfg.timings.hsync_back_porch = ESP_PANEL_LCD_RGB_TIMING_HBP;
  cfg.timings.hsync_front_porch = ESP_PANEL_LCD_RGB_TIMING_HFP;
  cfg.timings.vsync_pulse_width = ESP_PANEL_LCD_RGB_TIMING_VPW;
  cfg.timings.vsync_back_porch = ESP_PANEL_LCD_RGB_TIMING_VBP;
  cfg.timings.vsync_front_porch = ESP_PANEL_LCD_RGB_TIMING_VFP;
  cfg.timings.flags.pclk_active_neg = ESP_PANEL_LCD_RGB_PCLK_ACTIVE_NEG;
  cfg.data_width = ESP_PANEL_LCD_RGB_DATA_WIDTH;
  cfg.psram_trans_align = 64;
  cfg.hsync_gpio_num = ESP_PANEL_LCD_PIN_NUM_RGB_HSYNC;
  cfg.vsync_gpio_num = ESP_PANEL_LCD_PIN_NUM_RGB_VSYNC;
  cfg.de_gpio_num = ESP_PANEL_LCD_PIN_NUM_RGB_DE;
  cfg.pclk_gpio_num = ESP_PANEL_LCD_PIN_NUM_RGB_PCLK;
  cfg.disp_gpio_num = ESP_PANEL_LCD_PIN_NUM_RGB_DISP;
  const int data_pins[16] = {
    ESP_PANEL_LCD_PIN_NUM_RGB_DATA0,  ESP_PANEL_LCD_PIN_NUM_RGB_DATA1,
    ESP_PANEL_LCD_PIN_NUM_RGB_DATA2,  ESP_PANEL_LCD_PIN_NUM_RGB_DATA3,
    ESP_PANEL_LCD_PIN_NUM_RGB_DATA4,  ESP_PANEL_LCD_PIN_NUM_RGB_DATA5,
    ESP_PANEL_LCD_PIN_NUM_RGB_DATA6,  ESP_PANEL_LCD_PIN_NUM_RGB_DATA7,
    ESP_PANEL_LCD_PIN_NUM_RGB_DATA8,  ESP_PANEL_LCD_PIN_NUM_RGB_DATA9,
    ESP_PANEL_LCD_PIN_NUM_RGB_DATA10, ESP_PANEL_LCD_PIN_NUM_RGB_DATA11,
    ESP_PANEL_LCD_PIN_NUM_RGB_DATA12, ESP_PANEL_LCD_PIN_NUM_RGB_DATA13,
    ESP_PANEL_LCD_PIN_NUM_RGB_DATA14, ESP_PANEL_LCD_PIN_NUM_RGB_DATA15,
  };
  for (int i = 0; i < 16; ++i) cfg.data_gpio_nums[i] = data_pins[i];
  cfg.on_frame_trans_done = on_frame_done;
  cfg.user_ctx = NULL;
  cfg.flags.fb_in_psram = 1;

  // One 800x480 RGB565 frame buffer = 750 KB of PSRAM.
  size_t fb_bytes = (size_t)PANEL_H_RES * PANEL_V_RES * (ESP_PANEL_LCD_RGB_PIXEL_BITS / 8);
  size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  if (psram_free < fb_bytes) {
    Serial.printf("[DISPLAY] Not enough PSRAM for the frame buffer (%u needed, %u free) - no display\n",
                  (unsigned)fb_bytes, (unsigned)psram_free);
    return;
  }

  esp_err_t rc = esp_lcd_new_rgb_panel(&cfg, &panel_handle);
  if (rc != ESP_OK || !panel_handle) {
    Serial.printf("[DISPLAY] esp_lcd_new_rgb_panel failed: %s\n", esp_err_to_name(rc));
    panel_handle = NULL;
    return;
  }
  esp_lcd_panel_reset(panel_handle);
  esp_lcd_panel_init(panel_handle);
  Serial.printf("[DISPLAY] RGB panel %dx%d @ %u MHz, UI %dx%d at x=%d y=%d, PSRAM free %u\n",
                PANEL_H_RES, PANEL_V_RES, (unsigned)(ESP_PANEL_LCD_RGB_TIMING_FREQ_HZ / 1000000),
                ESP_PANEL_LCD_WIDTH, ESP_PANEL_LCD_HEIGHT, UI_X_OFFSET, UI_Y_OFFSET,
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

void LCD_Init()
{
  // Backlight off while the panel comes up, so no garbage is shown.
  Set_EXIO(pin_backlight(), Low);
  LCD_Reset();
  LCD_CreatePanel();
  LCD_FillPanel(0x0000);   // black bars either side of the 480x480 UI
  Backlight_Init();
}

void LCD_addWindow(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend, uint8_t* color)
{
  if (!panel_handle) return;
  // LVGL gives inclusive UI coordinates; draw_bitmap wants exclusive ends.
  int x0 = Xstart, y0 = Ystart;
  int x1 = Xend + 1, y1 = Yend + 1;
  if (x1 > ESP_PANEL_LCD_WIDTH)  x1 = ESP_PANEL_LCD_WIDTH;
  if (y1 > ESP_PANEL_LCD_HEIGHT) y1 = ESP_PANEL_LCD_HEIGHT;
  if (x0 >= x1 || y0 >= y1) return;
  // Copy in short bands with a brief pause between them. The panel's DMA
  // streams the frame buffer from PSRAM continuously; draw_bitmap copies the
  // area and then writes the cache back for all those rows in one burst, and
  // a long burst (a full-width band of values is ~30 rows) can hold the bus
  // long enough for the DMA to run dry. That showed as the background
  // flickering on solid-colour screens whenever values changed. Short bands
  // keep each burst small, and the pause (no PSRAM access) lets it catch up.
  const int BAND_ROWS = 8;
  const size_t row_bytes = (size_t)(x1 - x0) * 2;   // RGB565
  for (int y = y0; y < y1; y += BAND_ROWS) {
    int y_end = y + BAND_ROWS < y1 ? y + BAND_ROWS : y1;
    esp_lcd_panel_draw_bitmap(panel_handle,
                              x0 + UI_X_OFFSET, y + UI_Y_OFFSET,
                              x1 + UI_X_OFFSET, y_end + UI_Y_OFFSET,
                              color + (size_t)(y - y0) * row_bytes);
    if (y_end < y1) esp_rom_delay_us(20);
  }
}

// ── Backlight (on/off only on this board) ───────────────────────────────────
uint8_t LCD_Backlight = 50;

void Backlight_Init()
{
  Set_Backlight(LCD_Backlight);
}

void Set_Backlight(uint8_t Light)
{
  if (Light > Backlight_MAX) {
    printf("Set Backlight parameters in the range of 0 to 100 \r\n");
    return;
  }
  // Set_EXIO skips the I2C write when the level is unchanged, so the
  // brightness slider calling this repeatedly costs nothing.
  Set_EXIO(pin_backlight(), Light > 0 ? High : Low);
}
