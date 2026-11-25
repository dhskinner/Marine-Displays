// IO expander API — Waveshare ESP32-S3-Touch-LCD-7 (CH422G).
//
// The name and API are kept from the original 4" (ESP32-S3-Touch-LCD-4) firmware
// (TCA9554 / CH32V003) so
// callers don't change, but the 7" board has a CH422G, which is not a
// register-based I2C device: each "register" is its own I2C address and takes
// a single data byte. Its IO pins are all inputs or all outputs; this driver
// keeps them all outputs, so Mode_EXIO/Mode_EXIOS do nothing.
//
// Pins use the existing 1-indexed convention: Pin N = EXIO(N-1) = bit N-1.
//   EXIO1 (pin 2)  TP_RST   GT911 reset
//   EXIO2 (pin 3)  DISP     backlight on/off
//   EXIO3 (pin 4)  LCD_RST
//   EXIO4 (pin 5)  SD_CS    TF card chip select (active low)
//   EXIO5 (pin 6)  CAN_SEL  HIGH = GPIO19/20 to CAN transceiver, LOW = native USB
//
// CAN_SEL is held HIGH by the driver no matter what a caller writes: older
// 4" board code clears bit 5 ("V3 buzzer off"), which on this board would
// silently switch NMEA 2000 off.
#pragma once

#include <stdio.h>
#include "I2C_Driver.h"

#define TCA9554_EXIO1 0x01
#define TCA9554_EXIO2 0x02
#define TCA9554_EXIO3 0x03
#define TCA9554_EXIO4 0x04
#define TCA9554_EXIO5 0x05
#define TCA9554_EXIO6 0x06
#define TCA9554_EXIO7 0x07

// ── CH422G I2C "addresses" (7-bit) ──────────────────────────────────────────
#define CH422G_ADDR_WR_SET      0x24      // system parameters (IO_OE, OD_EN, ...)
#define CH422G_ADDR_WR_OC       0x23      // OC0..OC3 open-drain outputs
#define CH422G_ADDR_WR_IO       0x38      // IO0..IO7 output levels
#define CH422G_ADDR_RD_IO       0x26      // IO0..IO7 input levels
#define CH422G_SET_IO_OE        0x01      // WR_SET bit0: IO0..7 are push-pull outputs

#define CH422G_BIT_CAN_SEL      (1u << 5)
// Power-on output levels: TP_RST, LCD_RST, SD_CS, CAN_SEL and unused pins high;
// backlight (bit2) off until the panel has a picture.
#define CH422G_OUT_DEFAULT      0xFB

// Kept so the 4" board code that names these still compiles. The 7" board has
// no TCA9554 or CH32V003; is_board_v4() is always false, so the V4 paths that
// use the CH32V003_* values never run.
#define TCA9554_ADDR_V3         CH422G_ADDR_WR_SET
#define TCA9554_ADDR_V4         CH422G_ADDR_WR_SET
extern uint8_t g_tca9554_address;
#define TCA9554_ADDRESS         g_tca9554_address

#define TCA9554_INPUT_REG       CH422G_ADDR_RD_IO
#define TCA9554_OUTPUT_REG      CH422G_ADDR_WR_IO
#define TCA9554_Polarity_REG    0x00
#define TCA9554_CONFIG_REG      CH422G_ADDR_WR_SET

#define CH32V003_DIR_REG        CH422G_ADDR_WR_SET
#define CH32V003_OUTPUT_REG     CH422G_ADDR_WR_IO
#define CH32V003_INPUT_REG      CH422G_ADDR_RD_IO
#define CH32V003_PWM_REG        0x00
#define CH32V003_ADC_REG        0x00
#define CH32V003_DIR_ALL_OUT    0xFF
#define CH32V003_OUT_NORMAL     CH422G_OUT_DEFAULT
#define CH32V003_OUT_BUZZER     CH422G_OUT_DEFAULT
#define CH32V003_DIR_SAFE       0xFF
#define CH32V003_DIR_FULL       0xFF
#define V4_DIR_TCA_CONVENTION   0x00

#define Low   0
#define High  1
#define EXIO_PIN1   1
#define EXIO_PIN2   2
#define EXIO_PIN3   3
#define EXIO_PIN4   4
#define EXIO_PIN5   5
#define EXIO_PIN6   6
#define EXIO_PIN7   7
#define EXIO_PIN8   8

/*****************************************************  Board detection  ****************************************************/
bool detect_expander_address();                             // Checks the CH422G answers; logs if not
bool is_board_v4();                                         // Always false on the 7" board

uint8_t ch32v003_get_dir_shadow();
uint8_t ch32v003_get_out_shadow();

uint8_t exio_output_reg();                                  // CH422G_ADDR_WR_IO
uint8_t exio_dir_reg();                                     // CH422G_ADDR_WR_SET
uint8_t exio_input_reg();                                   // CH422G_ADDR_RD_IO

// Functional pin numbers (1-indexed, see table above)
uint8_t pin_tp_rst();                                       // 2 (EXIO1)
uint8_t pin_backlight();                                    // 3 (EXIO2)
uint8_t pin_lcd_rst();                                      // 4 (EXIO3)
uint8_t pin_sdcs();                                         // 5 (EXIO4)
uint8_t pin_can_sel();                                      // 6 (EXIO5)

// 4" board V4-only pins. No buzzer or SYS_EN on this board; the code that uses
// them is behind is_board_v4(). Writes to bit 5 are ignored (CAN_SEL).
#define PIN_BEE_EN   7
#define PIN_SYS_EN   6

/*****************************************************  Operation register REG   ****************************************************/
bool    I2C_Read_EXIO_safe(uint8_t REG, uint8_t *out);      // Output reg -> shadow; input reg -> RD_IO
uint8_t I2C_Read_EXIO(uint8_t REG);
uint8_t I2C_Write_EXIO(uint8_t REG,uint8_t Data);           // Output reg -> all outputs; others ignored
/********************************************************** Set EXIO mode **********************************************************/
void Mode_EXIO(uint8_t Pin,uint8_t State);                  // No-op: CH422G IO pins are all outputs
void Mode_EXIOS(uint8_t PinState);                          // No-op (re-asserts output mode)
/********************************************************** Read EXIO status **********************************************************/
uint8_t Read_EXIO(uint8_t Pin);
uint8_t Read_EXIOS(uint8_t REG = 0xFF);                     // Default: input levels. exio_output_reg(): shadow.
/********************************************************** Set the EXIO output status **********************************************************/
void Set_EXIO(uint8_t Pin,uint8_t State);                   // Sets one pin; skips the write if unchanged
void Set_EXIOS(uint8_t PinState);                           // Sets all pins (CAN_SEL forced high)
/********************************************************** Flip EXIO state **********************************************************/
void Set_Toggle(uint8_t Pin);
/********************************************************* Init ***********************************************************/
void TCA9554PWR_Init(uint8_t PinState = 0x00);              // Writes the default levels and enables outputs

void backlight_set_pwm(uint8_t duty);                       // No-op: no PWM backlight on this board
