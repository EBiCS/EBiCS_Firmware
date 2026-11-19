/*
 * Display protocols of the KT firmware, chosen at runtime (kt_settings.ui8_display):
 *   - Kunteng KT-LCD3 / LCD5: controller sends a 12 byte status every 150 ms, the display a 13 byte packet
 *     (casainho's code, moved here from ebike_app.c)
 *   - KingMeter 901U / KM5S: request / answer, 0x52 operation and 0x53 settings messages with the 901U
 *     handshake (ported from EBiCS display_kingmeter.c, Michael Fabry's library)
 *   - Bafang LCD (1200 baud) and 850C / 860C (9600 baud): request / answer (ported from EBiCS display_bafang.c)
 * Receiving and sending are interrupt driven (ring buffers), so the main loop never waits for the UART.
 *
 * This file is part of the KT firmware, GPL v3.
 */

#include <stdint.h>
#include "stm8s.h"
#include "stm8s_uart2.h"
#include "main.h"
#include "interrupts.h"
#include "display.h"
#include "settings.h"
#include "ebike_app.h"
#include "motor.h"
#include "brake.h"
#include "eeprom.h"

extern volatile struc_lcd_configuration_variables lcd_configuration_variables;
extern float f_wheel_speed;
extern float f_wheel_perimeter;
uint8_t ebike_app_cruise_control_is_set (void);
uint8_t throttle_is_set (void);

uint16_t ui16_display_wheel_perimeter_mm = 0;

// ------------------------------------------------------------------------------------------- ring buffers
#define RX_SIZE 64      // power of 2
#define TX_SIZE 32      // power of 2
// not static: the simulator test (kunteng/sim/run_sim.py) writes display messages straight into the buffer
volatile uint8_t ui8_rx_buf[RX_SIZE];
volatile uint8_t ui8_rx_head = 0;          // written by the interrupt
uint8_t ui8_rx_tail = 0;                   // read by display_poll
static volatile uint8_t ui8_tx_buf[TX_SIZE];
static uint8_t ui8_tx_head = 0;
static volatile uint8_t ui8_tx_tail = 0;

void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER)
{
  uint8_t ui8_next;
  uint8_t ui8_byte;
  if (UART2->SR & (UART2_SR_RXNE | UART2_SR_OR))
  {
    ui8_byte = UART2->DR; // reading DR also clears an overrun
    ui8_next = (ui8_rx_head + 1) & (RX_SIZE - 1);
    if (ui8_next != ui8_rx_tail) // full: drop the byte
    {
      ui8_rx_buf[ui8_rx_head] = ui8_byte;
      ui8_rx_head = ui8_next;
    }
  }
}

// "transmission complete" interrupt: one byte at a time (robust, the gap between bytes is negligible)
static volatile uint8_t ui8_tx_busy = 0;

void UART2_TX_IRQHandler(void) __interrupt(UART2_TX_IRQHANDLER)
{
  (void) UART2->SR; // TC is cleared by reading SR and then writing DR
  if (ui8_tx_tail != ui8_tx_head)
  {
    UART2->DR = ui8_tx_buf[ui8_tx_tail];
    ui8_tx_tail = (ui8_tx_tail + 1) & (TX_SIZE - 1);
  }
  else
  {
    UART2->CR2 &= (uint8_t) ~UART2_CR2_TCIEN;
    UART2->SR &= (uint8_t) ~UART2_SR_TC;
    ui8_tx_busy = 0;
  }
}

static void tx (uint8_t ui8_byte)
{
  uint8_t ui8_next = (ui8_tx_head + 1) & (TX_SIZE - 1);
  if (ui8_next == ui8_tx_tail) { return; } // full: a message is never longer than the buffer, so drop
  disableInterrupts ();
  if (!ui8_tx_busy)
  {
    ui8_tx_busy = 1;
    (void) UART2->SR;
    UART2->DR = ui8_byte;
    UART2->CR2 |= UART2_CR2_TCIEN;
  }
  else
  {
    ui8_tx_buf[ui8_tx_head] = ui8_byte;
    ui8_tx_head = ui8_next;
  }
  enableInterrupts ();
}

static uint8_t rx_count (void)
{
  return (ui8_rx_head - ui8_rx_tail) & (RX_SIZE - 1);
}

static uint8_t rx_peek (uint8_t ui8_offset)
{
  return ui8_rx_buf[(ui8_rx_tail + ui8_offset) & (RX_SIZE - 1)];
}

static void rx_drop (uint8_t ui8_n)
{
  ui8_rx_tail = (ui8_rx_tail + ui8_n) & (RX_SIZE - 1);
}

// ------------------------------------------------------------------------------------------- common values
static uint16_t wheel_period_ms (void)
{
  // standstill: a very long period (shows 0 km/h). 36000 x circumference overflowed 16 bits for wheels over
  // 1.82 m (26": 74250 -> 8714 ms = 0.9 km/h on the display)
  if (f_wheel_speed < 1) { return 65000; }
  return (uint16_t) ((3600.0 * f_wheel_perimeter) / f_wheel_speed);
}

static uint8_t battery_bars_x4 (void) // 16 full, 12, 8, 4, 3 empty (KT display scale)
{
  uint16_t ui16_volts = motor_get_ADC_battery_voltage_filtered () * ADC_BATTERY_VOLTAGE_K;
  if (ui16_volts > kt_settings.ui16_battery_pack_volts_x256[0]) { return 16; }
  if (ui16_volts > kt_settings.ui16_battery_pack_volts_x256[1]) { return 12; }
  if (ui16_volts > kt_settings.ui16_battery_pack_volts_x256[2]) { return 8; }
  if (ui16_volts > kt_settings.ui16_battery_pack_volts_x256[3]) { return 4; }
  return 3;
}

static uint16_t battery_current_x10 (void) // A x10, motoring only
{
  int16_t i16_current = motor_get_current_filtered_10b (); // 0.125 A
  if (i16_current < 0) { return 0; }
  return (uint16_t) ((i16_current * 10) >> 3);
}

static void use_default_lcd_values (void)
{
  // displays without the KT parameters: values from the settings block
  lcd_configuration_variables.ui8_motor_characteristic = kt_settings.ui8_motor_characteristic;
  lcd_configuration_variables.ui8_power_assist_control_mode = 1; // speed limit only, power from throttle / PAS
  lcd_configuration_variables.ui8_controller_max_current = 10;   // 100 %
}

// ------------------------------------------------------------------------------------------- Kunteng
static void kt_send_status (void)
{
  uint8_t ui8_buf[12];
  uint8_t ui8_i;
  uint8_t ui8_crc = 0;
  uint8_t ui8_moving = 0;
  uint8_t ui8_error = motor_controller_get_error ();
  uint8_t ui8_soc = battery_bars_x4 ();
  uint16_t ui16_period = wheel_period_ms ();
  int16_t i16_current = motor_get_current_filtered_10b ();

  if (ui8_error == MOTOR_CONTROLLER_ERROR_91_BATTERY_UNDER_VOLTAGE) { ui8_soc = 1; ui8_error = 0; } // empty, flashing
  if (brake_is_set ()) { ui8_moving |= (1 << 5); }
  if (ebike_app_cruise_control_is_set ()) { ui8_moving |= (1 << 3); }
  if (throttle_is_set ()) { ui8_moving |= (1 << 1); }
  if (i16_current < 0) { i16_current = 0; }
  i16_current <<= 1; // 0.125 A -> 0.25 A
  if (i16_current > 255) { i16_current = 255; }

  ui8_buf[0] = 65;
  ui8_buf[1] = ui8_soc;
  ui8_buf[2] = kt_settings.ui8_communications_battery_voltage;
  ui8_buf[3] = (uint8_t) (ui16_period >> 8);
  ui8_buf[4] = (uint8_t) ui16_period;
  ui8_buf[5] = ui8_error;
  ui8_buf[6] = 0;
  ui8_buf[7] = ui8_moving;
  ui8_buf[8] = (uint8_t) i16_current;
  ui8_buf[9] = 0;
  ui8_buf[10] = 0;
  ui8_buf[11] = 0;
  for (ui8_i = 1; ui8_i <= 11; ui8_i++) { ui8_crc ^= ui8_buf[ui8_i]; }
  ui8_buf[6] = ui8_crc;
  for (ui8_i = 0; ui8_i <= 11; ui8_i++) { tx (ui8_buf[ui8_i]); }
}

// casainho's framing: the packet is taken from the bytes 50, 14 (the end of the previous display packet) on,
// 12 bytes, B10 of the display packet is not used and counts as 0 in the CRC
static void kt_poll (void)
{
  uint8_t ui8_p[13];
  uint8_t ui8_i;
  uint8_t ui8_crc = 0;

  while (rx_count () >= 2 && !((rx_peek (0) == 50) && (rx_peek (1) == 14))) { rx_drop (1); }
  if (rx_count () < 12) { return; }
  for (ui8_i = 0; ui8_i < 12; ui8_i++) { ui8_p[ui8_i] = rx_peek (ui8_i); }
  ui8_p[12] = 0;
  rx_drop (12);
  for (ui8_i = 0; ui8_i <= 12; ui8_i++) { if (ui8_i != 7) { ui8_crc ^= ui8_p[ui8_i]; } }
  if (((ui8_crc ^ 10) == ui8_p[7]) || ((ui8_crc ^ 9) == ui8_p[7]) || ((ui8_crc ^ 2) == ui8_p[7]))
  {
    lcd_configuration_variables.ui8_assist_level = ui8_p[3] & 7;
    lcd_configuration_variables.ui8_motor_characteristic = ui8_p[5];
    lcd_configuration_variables.ui8_wheel_size = ((ui8_p[6] & 192) >> 6) | ((ui8_p[4] & 7) << 2);
    lcd_configuration_variables.ui8_max_speed = 10 + (((ui8_p[4] & 248) >> 3) | (ui8_p[6] & 32));
    lcd_configuration_variables.ui8_power_assist_control_mode = ui8_p[6] & 8;
    lcd_configuration_variables.ui8_controller_max_current = (ui8_p[9] & 15);
    eeprom_write_if_values_changed ();
  }
}

// ------------------------------------------------------------------------------------------- KingMeter 901U
static const uint8_t ui8_km_handshake[64] = {
  137, 159, 134, 249, 88, 11, 250, 61, 33, 150, 3, 193, 118, 141, 209, 94, 226, 68, 146, 158, 145, 127,
  216, 62, 116, 230, 101, 211, 251, 54, 229, 247, 20, 222, 59, 63, 35, 252, 142, 238, 23, 197, 84, 77,
  147, 173, 210, 57, 142, 223, 157, 97, 36, 160, 229, 237, 75, 80, 37, 113, 154, 88, 23, 120 };

static void km_answer (uint8_t ui8_cmd, uint8_t ui8_handshake_index)
{
  uint8_t ui8_buf[9];
  uint8_t ui8_i;
  uint16_t ui16_sum = 0;
  uint16_t ui16_period = wheel_period_ms ();
  uint16_t ui16_current = battery_current_x10 ();

  if (ui16_period > 0x0DAC) { ui16_period = 0x0DAC; } // KM_MAX_WHEELTIME: wheel stopped
  ui8_buf[0] = 0x3A;
  ui8_buf[1] = 0x1A;
  ui8_buf[2] = ui8_cmd;
  ui8_buf[3] = 0x05;
  if (ui8_cmd == 0x52)
  {
    ui8_buf[4] = (battery_bars_x4 () <= 3) ? 0x40 : 0x00;          // battery low symbol
    ui8_buf[5] = (uint8_t) ((ui16_current * 3) / 10);              // 1/3 A
    ui8_buf[6] = (uint8_t) (ui16_period >> 8);
    ui8_buf[7] = (uint8_t) ui16_period;
    ui8_buf[8] = motor_controller_get_error ();
  }
  else // 0x53 settings: handshake
  {
    ui8_buf[4] = 0x00;
    ui8_buf[5] = 0x00;
    ui8_buf[6] = 0x0D;
    ui8_buf[7] = ui8_km_handshake[ui8_handshake_index & 0x3F];
    ui8_buf[8] = 0x00;
  }
  for (ui8_i = 1; ui8_i < 9; ui8_i++) { ui16_sum += ui8_buf[ui8_i]; }
  for (ui8_i = 0; ui8_i < 9; ui8_i++) { tx (ui8_buf[ui8_i]); }
  tx ((uint8_t) ui16_sum);
  tx ((uint8_t) (ui16_sum >> 8));
  tx (0x0D);
  tx (0x0A);
}

// message: 0x3A, address, command, length n, n data bytes, checksum (sum of bytes 1 ... 3+n, low byte first), 0x0D 0x0A
static void km_poll (void)
{
  uint8_t ui8_n;
  uint8_t ui8_i;
  uint8_t ui8_cmd;
  uint16_t ui16_sum = 0;
  uint16_t ui16_got;
  uint8_t ui8_level;
  uint16_t ui16_mm;

  while (rx_count () && (rx_peek (0) != 0x3A)) { rx_drop (1); }
  if (rx_count () < 4) { return; }
  ui8_n = rx_peek (3);
  if (ui8_n > 20) { rx_drop (1); return; } // not a real length: resync
  if (rx_count () < (uint8_t) (ui8_n + 6)) { return; }
  for (ui8_i = 1; ui8_i < (uint8_t) (4 + ui8_n); ui8_i++) { ui16_sum += rx_peek (ui8_i); }
  ui16_got = rx_peek (4 + ui8_n) | (((uint16_t) rx_peek (5 + ui8_n)) << 8);
  ui8_cmd = rx_peek (2);
  if (ui16_sum == ui16_got)
  {
    if ((ui8_cmd == 0x52) && (ui8_n >= 2))
    {
      // assist level 0 ... 255 of the KM display -> KT levels 0 ... 5
      ui8_level = rx_peek (4);
      lcd_configuration_variables.ui8_assist_level = (uint8_t) ((((uint16_t) ui8_level) * 5 + 127) / 255);
      km_answer (0x52, 0);
    }
    else if ((ui8_cmd == 0x53) && (ui8_n >= 10))
    {
      ui16_mm = (((uint16_t) rx_peek (12)) << 8) | rx_peek (13);
      if ((ui16_mm >= 500) && (ui16_mm <= 3000)) { ui16_display_wheel_perimeter_mm = ui16_mm; }
      ui8_level = rx_peek (11);                       // speed limit (km/h)
      if ((ui8_level >= 5) && (ui8_level <= 70)) { lcd_configuration_variables.ui8_max_speed = ui8_level; }
      km_answer (0x53, rx_peek (9));
    }
  }
  rx_drop (ui8_n + 6);
}

// ------------------------------------------------------------------------------------------- Bafang
// requests 0x11 <cmd> are answered at once; infos 0x16 <cmd> ... set values
static void bf_poll (void)
{
  uint8_t ui8_cmd;
  uint8_t ui8_v;
  uint16_t ui16_v;
  uint16_t ui16_period;

  while (rx_count () && (rx_peek (0) != 0x11) && (rx_peek (0) != 0x16)) { rx_drop (1); }
  if (rx_count () < 2) { return; }
  ui8_cmd = rx_peek (1);
  if (rx_peek (0) == 0x11)
  {
    rx_drop (2);
    switch (ui8_cmd)
    {
      case 0x20: // speed: wheel rpm
      ui16_period = wheel_period_ms ();
      ui16_v = (ui16_period >= 36000U) ? 0 : (uint16_t) (60000UL / ui16_period);
      tx ((uint8_t) (ui16_v >> 8));
      tx ((uint8_t) ui16_v);
      tx ((uint8_t) ((uint8_t) (ui16_v >> 8) + (uint8_t) ui16_v + 32));
      break;
      case 0x08: // status
      tx (brake_is_set () ? 0x03 : 0x01);
      break;
      case 0x11: // battery %
      ui8_v = battery_bars_x4 ();
      ui8_v = (ui8_v <= 3) ? 5 : (uint8_t) (ui8_v * 6); // 4 -> 24 %, 8 -> 48, 12 -> 72, 16 -> 96
      tx (ui8_v);
      tx (ui8_v);
      break;
      case 0x0A: // current, 0.5 A
      ui16_v = battery_current_x10 () / 5;
      ui8_v = (ui16_v > 255) ? 255 : (uint8_t) ui16_v;
      tx (ui8_v);
      tx (ui8_v);
      break;
      case 0x31: // moving
      ui8_v = (ui16_motor_get_motor_speed_erps () > 0) ? '1' : '0';
      tx (ui8_v);
      tx (ui8_v);
      break;
      case 0x24: // battery voltage x100
      ui16_v = (uint16_t) ((((uint32_t) motor_get_ADC_battery_voltage_filtered ()) * 272) / 10);
      tx ((uint8_t) (ui16_v >> 8));
      tx ((uint8_t) ui16_v);
      tx ((uint8_t) ((uint8_t) (ui16_v >> 8) + (uint8_t) ui16_v));
      break;
      default:
      break;
    }
    return;
  }
  // info
  if (ui8_cmd == 0x0B) // assist level: 0x16 0x0B level checksum
  {
    if (rx_count () < 4) { return; }
    ui8_v = rx_peek (2);
    if (rx_peek (3) == (uint8_t) (0x16 + 0x0B + ui8_v))
    {
      // Bafang codes for the 9 level display, 3 and 5 level displays use a subset
      switch (ui8_v)
      {
        case 0: lcd_configuration_variables.ui8_assist_level = 0; break;
        case 1: case 11: lcd_configuration_variables.ui8_assist_level = 1; break;
        case 13: case 21: lcd_configuration_variables.ui8_assist_level = 2; break;
        case 2: case 23: case 3: lcd_configuration_variables.ui8_assist_level = 3; break;
        case 12: case 27: case 28: lcd_configuration_variables.ui8_assist_level = 4; break;
        case 29: case 30: lcd_configuration_variables.ui8_assist_level = 5; break;
        default: break; // 6 = walk assist: not supported
      }
    }
    rx_drop (4);
  }
  else if (ui8_cmd == 0x1A) { if (rx_count () < 3) { return; } rx_drop (3); } // light
  else if (ui8_cmd == 0x1F) { if (rx_count () < 5) { return; } rx_drop (5); } // wheel diameter
  else { rx_drop (2); }
}

// ------------------------------------------------------------------------------------------- dispatch
uint32_t display_baudrate (void)
{
  return (kt_settings.ui8_display == DISPLAY_BAFANG_LCD) ? 1200 : 9600;
}

void display_init (void)
{
  if (kt_settings.ui8_display != DISPLAY_KT) { use_default_lcd_values (); }
  UART2->CR2 |= UART2_CR2_RIEN;
}

void display_poll (void)
{
  switch (kt_settings.ui8_display)
  {
    case DISPLAY_KINGMETER: km_poll (); break;
    case DISPLAY_BAFANG_LCD:
    case DISPLAY_BAFANG_TFT: bf_poll (); break;
    default: kt_poll (); break;
  }
}

void display_controller (void)
{
  if (kt_settings.ui8_display == DISPLAY_KT) { kt_send_status (); } // the KT display expects it every 150 ms
}
