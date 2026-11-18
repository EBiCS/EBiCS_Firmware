/*
 * display_vesc.c
 *
 * Minimal VESC UART protocol, see display_vesc.h.
 *
 * This file is part of EBiCS, GPL v3 or later.
 */

#include "config.h"
#include "main.h"
#include "stm32f1xx_hal.h"

#if (DISPLAY_TYPE == DISPLAY_TYPE_VESC)

#include <string.h>
#include "FOC.h"
#include "display_vesc.h"

extern UART_HandleTypeDef huart1;
extern MotorState_t MS;
extern uint16_t ui16_erps;
extern int8_t i8_direction;
extern int8_t i8_reverse_flag;

static uint8_t rx_ring[VESC_RX_BUFF];
static uint8_t rx_tail = 0;
static uint8_t tx_buf[96];

static uint16_t crc16(const uint8_t *buf, uint16_t len)
{
	uint16_t crc = 0;
	while (len--) {
		crc ^= (uint16_t)(*buf++) << 8;
		for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
	}
	return crc;
}

static void put16(uint8_t *b, int32_t *i, int16_t v) { b[(*i)++] = v >> 8; b[(*i)++] = v; }
static void put32(uint8_t *b, int32_t *i, int32_t v) { b[(*i)++] = v >> 24; b[(*i)++] = v >> 16; b[(*i)++] = v >> 8; b[(*i)++] = v; }
static int32_t get32(const uint8_t *b) { return ((int32_t)b[0] << 24) | ((int32_t)b[1] << 16) | ((int32_t)b[2] << 8) | b[3]; }

static void send_payload(const uint8_t *payload, uint8_t len)
{
	uint16_t crc = crc16(payload, len);
	tx_buf[0] = 0x02;
	tx_buf[1] = len;
	memcpy(tx_buf + 2, payload, len);
	tx_buf[2 + len] = crc >> 8;
	tx_buf[3 + len] = crc & 0xFF;
	tx_buf[4 + len] = 0x03;
	HAL_UART_Transmit_DMA(&huart1, tx_buf, len + 5);
}

static void reply_fw_version(void)
{
	static uint8_t p[40];
	int32_t i = 0;
	p[i++] = COMM_FW_VERSION;
	p[i++] = 6;                                  // major
	p[i++] = 0;                                  // minor
	memcpy(p + i, "EBiCS", 6); i += 6;           // hardware name, 0 terminated
	memcpy(p + i, (const void*)0x1FFFF7E8, 12);  // STM32 unique ID as UUID
	i += 12;
	p[i++] = 0;                                  // pairing done
	p[i++] = 0;                                  // test version
	p[i++] = 0;                                  // hw type: VESC
	p[i++] = 0;                                  // custom config count
	send_payload(p, (uint8_t)i);
}

static void reply_values(void)
{
	static uint8_t p[80];
	int32_t i = 0;
	int32_t ma_per_unit = CAL_I >> 8;                           // mA per internal current unit
	int32_t dir = i8_direction * i8_reverse_flag;
	int32_t erpm = motor_is_turning() ? (int32_t)ui16_erps * 60 * dir : 0;

	p[i++] = COMM_GET_VALUES;
	put16(p, &i, MS.int_Temperature * 10);                     // temp fet, 0.1 C
	put16(p, &i, MS.Temperature * 10);                         // temp motor, 0.1 C
	put32(p, &i, (MS.i_q * ma_per_unit) / 10);                 // avg motor current, 0.01 A
	put32(p, &i, MS.Battery_Current / 10);                     // avg input current, 0.01 A
	put32(p, &i, (MS.i_d * ma_per_unit) / 10);                 // avg id
	put32(p, &i, (MS.i_q * ma_per_unit) / 10);                 // avg iq
	put16(p, &i, (int16_t)((MS.u_abs * 1000) / _U_MAX) * dir); // duty, 0.1 %
	put32(p, &i, erpm);                                        // electrical rpm
	put16(p, &i, (int16_t)((MS.Voltage * CAL_V) / 100));       // input voltage, 0.1 V
	put32(p, &i, 0);                                           // amp hours
	put32(p, &i, 0);                                           // amp hours charged
	put32(p, &i, 0);                                           // watt hours
	put32(p, &i, 0);                                           // watt hours charged
	put32(p, &i, 0);                                           // tachometer
	put32(p, &i, 0);                                           // tachometer abs
	p[i++] = 0;                                                // fault code
	put32(p, &i, 0);                                           // pid pos
	p[i++] = 0;                                                // controller id
	put16(p, &i, MS.int_Temperature * 10);                     // temp mos 1..3
	put16(p, &i, MS.int_Temperature * 10);
	put16(p, &i, MS.int_Temperature * 10);
	put32(p, &i, 0);                                           // avg vd
	put32(p, &i, 0);                                           // avg vq
	p[i++] = 0;                                                // status
	send_payload(p, (uint8_t)i);
}

static void process(VESC_t* v, const uint8_t *p, uint8_t len)
{
	v->rx_packets++;
	switch (p[0])
	{
	case COMM_FW_VERSION:
		reply_fw_version();
		break;
	case COMM_GET_VALUES:
		reply_values();
		break;
	case COMM_SET_CURRENT:
		if (len >= 5) {
			int32_t ma = get32(p + 1);
			int32_t target = ma / (CAL_I >> 8);
			if (target < 0) target = 0;                        // no regen via VESC commands
			if (target > RS.ph_current_max) target = RS.ph_current_max;
			v->current_target = target;
			v->cmd_active = 1;
			v->last_cmd_tick = HAL_GetTick();
		}
		break;
	case COMM_SET_CURRENT_BRAKE:
	case COMM_SET_DUTY:                                        // not supported: treat as "no drive"
		v->current_target = 0;
		v->cmd_active = 1;
		v->last_cmd_tick = HAL_GetTick();
		break;
	case COMM_ALIVE:
		v->last_cmd_tick = HAL_GetTick();
		break;
	default:
		break;
	}
}

void Vesc_Init(VESC_t* v)
{
	v->current_target = 0;
	v->cmd_active = 0;
	v->last_cmd_tick = 0;
	v->rx_packets = 0;
	rx_tail = 0;
	if (HAL_UART_Receive_DMA(&huart1, rx_ring, VESC_RX_BUFF) != HAL_OK)
	{
		Error_Handler();
	}
}

void Vesc_Service(VESC_t* v)
{
	uint8_t head = (uint8_t)(VESC_RX_BUFF - DMA1_Channel5->CNDTR) & (VESC_RX_BUFF - 1);
	uint8_t avail = (uint8_t)(head - rx_tail) & (VESC_RX_BUFF - 1);
	uint8_t payload[VESC_MAX_PAYLOAD];

	while (avail >= 6)
	{
		uint8_t len = rx_ring[(rx_tail + 1) & (VESC_RX_BUFF - 1)];
		if (rx_ring[rx_tail] != 0x02 || len == 0 || len > VESC_MAX_PAYLOAD) {
			rx_tail = (rx_tail + 1) & (VESC_RX_BUFF - 1);
			avail--;
			continue;
		}
		if (avail < len + 5) break;   // wait for the rest of the packet

		for (uint8_t n = 0; n < len; n++) payload[n] = rx_ring[(rx_tail + 2 + n) & (VESC_RX_BUFF - 1)];
		uint16_t crc = ((uint16_t)rx_ring[(rx_tail + 2 + len) & (VESC_RX_BUFF - 1)] << 8) | rx_ring[(rx_tail + 3 + len) & (VESC_RX_BUFF - 1)];
		uint8_t end = rx_ring[(rx_tail + 4 + len) & (VESC_RX_BUFF - 1)];

		if (end == 0x03 && crc == crc16(payload, len)) {
			process(v, payload, len);
			rx_tail = (rx_tail + len + 5) & (VESC_RX_BUFF - 1);
			avail -= len + 5;
		} else {
			rx_tail = (rx_tail + 1) & (VESC_RX_BUFF - 1);
			avail--;
		}
	}

	if (v->cmd_active && (HAL_GetTick() - v->last_cmd_tick) > VESC_CMD_TIMEOUT_MS) {
		v->cmd_active = 0;   // safety: release the motor if the commanding device goes quiet
		v->current_target = 0;
	}
}

#endif // DISPLAY_TYPE_VESC
