/*
 * display_vesc.h
 *
 * Minimal VESC UART protocol on the display port (115200 baud).
 * Lets VESC compatible displays, apps (via a UART-BLE bridge) and dashboards
 * read live values (COMM_GET_VALUES) and request motor current
 * (COMM_SET_CURRENT, released automatically after 1 s without commands).
 * This is not a full VESC: no motor configuration from VESC Tool.
 *
 * Packet: 0x02, len, payload[len], crc16 hi, crc16 lo, 0x03  (CRC16-CCITT/XMODEM)
 *
 * This file is part of EBiCS, GPL v3 or later.
 */

#ifndef DISPLAY_VESC_H_
#define DISPLAY_VESC_H_

#include <stdint.h>

#define VESC_RX_BUFF        64      // DMA ring buffer, power of 2
#define VESC_MAX_PAYLOAD    (VESC_RX_BUFF - 6)
#define VESC_CMD_TIMEOUT_MS 1000

// subset of the VESC COMM_PACKET_ID enum
#define COMM_FW_VERSION         0
#define COMM_GET_VALUES         4
#define COMM_SET_DUTY           5
#define COMM_SET_CURRENT        6
#define COMM_SET_CURRENT_BRAKE  7
#define COMM_ALIVE              30

typedef struct
{
	int32_t  current_target;    // requested phase current in internal units (like PH_CURRENT_MAX)
	uint8_t  cmd_active;        // 1 while a COMM_SET_CURRENT is valid
	uint32_t last_cmd_tick;     // HAL_GetTick() of the last current/alive command
	uint32_t rx_packets;        // valid packets received (diagnostics)
} VESC_t;

void Vesc_Init(VESC_t* v);
void Vesc_Service(VESC_t* v);

#endif /* DISPLAY_VESC_H_ */
