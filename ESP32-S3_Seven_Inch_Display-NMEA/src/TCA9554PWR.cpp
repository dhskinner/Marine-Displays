// CH422G IO expander driver for the Waveshare ESP32-S3-Touch-LCD-7.
// Keeps the original TCA9554PWR API; see TCA9554PWR.h.
#include "TCA9554PWR.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

extern "C" int ets_printf(const char *fmt, ...);

uint8_t g_tca9554_address = CH422G_ADDR_WR_SET;

// The CH422G can't read back its output latch (RD_IO returns pin levels), so
// the output byte is kept here. CAN_SEL is always set.
static uint8_t s_out = CH422G_OUT_DEFAULT;
static bool s_outputs_enabled = false;
static SemaphoreHandle_t s_lock = NULL;

static void lock()   { if (!s_lock) s_lock = xSemaphoreCreateRecursiveMutex(); xSemaphoreTakeRecursive(s_lock, portMAX_DELAY); }
static void unlock() { xSemaphoreGiveRecursive(s_lock); }

static bool ch422g_write(uint8_t addr, uint8_t data)
{
  Wire.beginTransmission(addr);
  Wire.write(data);
  return Wire.endTransmission() == 0;
}

// Writes the output levels, then (once) switches the IO pins to outputs.
// Levels first, so no pin glitches when the outputs turn on.
static bool ch422g_write_outputs(uint8_t value)
{
  value |= CH422G_BIT_CAN_SEL;
  bool ok = ch422g_write(CH422G_ADDR_WR_IO, value);
  if (ok && !s_outputs_enabled) {
    ok = ch422g_write(CH422G_ADDR_WR_SET, CH422G_SET_IO_OE);
    if (ok) {
      s_outputs_enabled = true;
      ok = ch422g_write(CH422G_ADDR_WR_IO, value);
    }
  }
  if (ok) s_out = value;
  else {
    static unsigned long last_log = 0;
    if (millis() - last_log > 5000) {
      last_log = millis();
      printf("[EXIO] CH422G write failed (out=0x%02X)\r\n", value);
    }
  }
  return ok;
}

bool detect_expander_address()
{
  Wire.beginTransmission(CH422G_ADDR_WR_SET);
  bool ack = Wire.endTransmission() == 0;
  ets_printf("[BOARD] Waveshare ESP32-S3-Touch-LCD-7, CH422G %s\r\n", ack ? "found" : "NOT responding");
  return ack;
}

bool is_board_v4() { return false; }

uint8_t ch32v003_get_dir_shadow() { return 0xFF; }
uint8_t ch32v003_get_out_shadow() { return s_out; }

uint8_t exio_output_reg() { return CH422G_ADDR_WR_IO; }
uint8_t exio_dir_reg()    { return CH422G_ADDR_WR_SET; }
uint8_t exio_input_reg()  { return CH422G_ADDR_RD_IO; }

uint8_t pin_tp_rst()    { return 2; }
uint8_t pin_backlight() { return 3; }
uint8_t pin_lcd_rst()   { return 4; }
uint8_t pin_sdcs()      { return 5; }
uint8_t pin_can_sel()   { return 6; }

static bool ch422g_read_inputs(uint8_t *out)
{
  if (Wire.requestFrom((uint8_t)CH422G_ADDR_RD_IO, (uint8_t)1) != 1) return false;
  *out = Wire.read();
  return true;
}

bool I2C_Read_EXIO_safe(uint8_t REG, uint8_t *out)
{
  if (REG == CH422G_ADDR_WR_IO) { *out = s_out; return true; }
  if (REG == CH422G_ADDR_WR_SET) { *out = 0x00; return true; }   // "all outputs" (TCA convention)
  lock();
  bool ok = ch422g_read_inputs(out);
  unlock();
  return ok;
}

uint8_t I2C_Read_EXIO(uint8_t REG)
{
  uint8_t val = 0xFF;
  I2C_Read_EXIO_safe(REG, &val);
  return val;
}

uint8_t I2C_Write_EXIO(uint8_t REG, uint8_t Data)
{
  if (REG != CH422G_ADDR_WR_IO) return 0;   // direction/PWM registers don't exist here
  lock();
  bool ok = ch422g_write_outputs(Data);
  unlock();
  return ok ? 0 : (uint8_t)-1;
}

void Mode_EXIO(uint8_t Pin, uint8_t State) { (void)Pin; (void)State; }

void Mode_EXIOS(uint8_t PinState)
{
  (void)PinState;
  if (s_outputs_enabled) return;
  lock();
  ch422g_write_outputs(s_out);
  unlock();
}

uint8_t Read_EXIO(uint8_t Pin)
{
  if (Pin < 1 || Pin > 8) return 0;
  return (I2C_Read_EXIO(CH422G_ADDR_RD_IO) >> (Pin - 1)) & 0x01;
}

uint8_t Read_EXIOS(uint8_t REG)
{
  if (REG == 0xFF) REG = CH422G_ADDR_RD_IO;
  return I2C_Read_EXIO(REG);
}

void Set_EXIO(uint8_t Pin, uint8_t State)
{
  if (State > 1 || Pin < 1 || Pin > 8) {
    printf("Parameter error, please enter the correct parameter!\r\n");
    return;
  }
  lock();
  uint8_t bit = (uint8_t)(1u << (Pin - 1));
  uint8_t value = State ? (s_out | bit) : (s_out & (uint8_t)~bit);
  value |= CH422G_BIT_CAN_SEL;   // never let CAN_SEL go low (see header)
  if (value != s_out || !s_outputs_enabled) ch422g_write_outputs(value);
  unlock();
}

void Set_EXIOS(uint8_t PinState)
{
  lock();
  if (((PinState | CH422G_BIT_CAN_SEL) != s_out) || !s_outputs_enabled) ch422g_write_outputs(PinState);
  unlock();
}

void Set_Toggle(uint8_t Pin)
{
  if (Pin < 1 || Pin > 8) return;
  Set_EXIO(Pin, ((s_out >> (Pin - 1)) & 0x01) ? Low : High);
}

void TCA9554PWR_Init(uint8_t PinState)
{
  (void)PinState;
  lock();
  s_outputs_enabled = false;              // force WR_SET to be sent again
  ch422g_write_outputs(s_out);
  unlock();
}

void backlight_set_pwm(uint8_t duty) { (void)duty; }
