/*
 * Display protocols of the KT firmware: Kunteng (KT-LCD3/LCD5), KingMeter 901U / KM5S, Bafang LCD and
 * Bafang 850C/860C. Chosen at runtime with the settings block (kt_settings.ui8_display).
 *
 * This file is part of the KT firmware, GPL v3.
 */

#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <stdint.h>

#define DISPLAY_KT          0   // Kunteng KT-LCD3 / LCD5 (9600 baud)
#define DISPLAY_KINGMETER   1   // KingMeter 901U / KM5S (9600 baud)
#define DISPLAY_BAFANG_LCD  2   // Bafang LCD (1200 baud)
#define DISPLAY_BAFANG_TFT  3   // Bafang 850C / 860C (9600 baud)
#define DISPLAY_LAST        3

extern uint16_t ui16_display_wheel_perimeter_mm; // set by displays that send the wheel size in mm (0 = KT code)

uint32_t display_baudrate (void);
void display_init (void);        // after uart_init
void display_poll (void);        // as often as possible from the main loop: answers requests of the display
void display_controller (void);  // every 150 ms from ebike_app_controller

#endif /* _DISPLAY_H_ */
