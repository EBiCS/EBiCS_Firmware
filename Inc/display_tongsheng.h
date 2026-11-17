/*
 * display_tongsheng.h
 *
 * Tongsheng (TSDZ2/TSDZ8) factory display protocol, used by the cheap
 * VLCD5, VLCD6, XH18 (and some 850C "Tongsheng" variants) displays.
 * 9600 baud 8N1. Display -> controller: 7 byte frames starting with 0x59,
 * controller -> display: 9 byte frames starting with 0x43, both end with an
 * 8 bit additive checksum of the preceding bytes.
 *
 * This file is part of EBiCS, GPL v3 or later.
 */

#ifndef DISPLAY_TONGSHENG_H_
#define DISPLAY_TONGSHENG_H_

#include <stdint.h>

#define TS_RX_BUFF          64      // DMA ring buffer, must be a power of 2
#define TS_RX_FRAME_LEN     7
#define TS_TX_FRAME_LEN     9
#define TS_RX_HEADER        0x59
#define TS_TX_HEADER        0x43
#define TS_WHEEL_STOPPED    0x0700  // wheel period value for "standing still" (2 ms ticks)

// display -> controller, byte 1 bitfield
#define TS_LIGHT            0x01
#define TS_LEVEL_3          0x02
#define TS_LEVEL_4          0x04
#define TS_LEVEL_5          0x08
#define TS_LEVEL_0          0x10
#define TS_WALK             0x20
#define TS_LEVEL_2          0x40
#define TS_LEVEL_1          0x80

typedef struct
{
	// received from the display
	uint8_t  assist_level;      // 0..5
	uint8_t  light;             // 0/1
	uint8_t  walk;              // 0/1 walk assist button held
	uint8_t  wheel_inch;        // wheel diameter setting of the display
	uint8_t  speed_limit;       // km/h setting of the display
	uint32_t rx_frames;         // number of valid frames received (diagnostics)
} TONGSHENG_t;

void Tongsheng_Init(TONGSHENG_t* ts);
void Tongsheng_Service(TONGSHENG_t* ts);

#endif /* DISPLAY_TONGSHENG_H_ */
