/*
 * display_tongsheng.c
 *
 * Tongsheng factory display protocol (VLCD5 / VLCD6 / XH18).
 * Frames are found by scanning the DMA ring buffer for the header byte and
 * verifying the checksum, so partial or merged UART chunks don't matter.
 *
 * This file is part of EBiCS, GPL v3 or later.
 */

#include "config.h"
#include "main.h"
#include "stm32f1xx_hal.h"

#if (DISPLAY_TYPE == DISPLAY_TYPE_TONGSHENG)

#include "display_tongsheng.h"

extern UART_HandleTypeDef huart1;
extern MotorState_t MS;
extern uint8_t brake_flag;
extern volatile uint8_t ui8_Push_Assist_flag;
extern uint32_t get_wheeltime_ms(void);

static uint8_t rx_ring[TS_RX_BUFF];
static uint8_t rx_tail = 0;
static uint8_t tx_frame[TS_TX_FRAME_LEN];

static uint8_t ts_checksum(const uint8_t *b, uint8_t n)
{
	uint8_t s = 0;
	while (n--) s += *b++;
	return s;
}

static uint8_t ts_level_from_bits(uint8_t bits)
{
	// only one gear bit should be set, if several are set the highest wins
	if (bits & TS_LEVEL_5) return 5;
	if (bits & TS_LEVEL_4) return 4;
	if (bits & TS_LEVEL_3) return 3;
	if (bits & TS_LEVEL_2) return 2;
	if (bits & TS_LEVEL_1) return 1;
	return 0;
}

static uint8_t ts_battery_level(void)
{
	// 0..12 from the battery voltage, using the battery bar thresholds of config.h (mV*10)
	int32_t v = MS.Voltage * CAL_BAT_V;
	if (v <= BATTERY_LEVEL_1) return 0;
	if (v >= BATTERY_LEVEL_5) return 12;
	return (uint8_t)(((v - BATTERY_LEVEL_1) * 12) / (BATTERY_LEVEL_5 - BATTERY_LEVEL_1));
}

static void ts_apply_rx(TONGSHENG_t* ts, const uint8_t *f)
{
	ts->light        = (f[1] & TS_LIGHT) ? 1 : 0;
	ts->walk         = (f[1] & TS_WALK) ? 1 : 0;
	ts->assist_level = ts_level_from_bits(f[1]);
	ts->wheel_inch   = f[3];
	ts->speed_limit  = f[5];
	ts->rx_frames++;

	// assist level 0..5 -> 0..255 scale used by the KM5S/No2 code path in main.c
	if (ts->walk) {
		ui8_Push_Assist_flag = 1;
		MS.assist_level = 255;      // push assist current = PUSHASSIST_CURRENT
	} else {
		ui8_Push_Assist_flag = 0;
		MS.assist_level = ts->assist_level * 51;
	}

	HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, ts->light ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void ts_send_status(void)
{
	uint32_t period = get_wheeltime_ms() >> 1;  // 2 ms ticks
	if (period > TS_WHEEL_STOPPED) period = TS_WHEEL_STOPPED;

	int32_t current_x10 = MS.Battery_Current / 100;   // mA -> 0.1 A
	if (current_x10 < 0) current_x10 = 0;
	if (current_x10 > 255) current_x10 = 255;

	uint8_t status = 0;
	if (MS.Voltage < VOLTAGE_MIN) status |= 0x01;           // undervoltage
	if (READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)) status |= 0x04; // motor working

	uint8_t error = 0;
	if (MS.Temperature > MOTOR_TEMPERATURE_MAX || MS.int_Temperature > CONTROLLER_TEMPERATURE_MAX) error = 1; // temperature protection

	tx_frame[0] = TS_TX_HEADER;
	tx_frame[1] = ts_battery_level();
	tx_frame[2] = status;
	tx_frame[3] = 0;                          // torque signal (not used by the display)
	tx_frame[4] = (uint8_t) current_x10;
	tx_frame[5] = error;
	tx_frame[6] = period & 0xFF;              // wheel period, little endian
	tx_frame[7] = (period >> 8) & 0xFF;
	tx_frame[8] = ts_checksum(tx_frame, 8);

	HAL_UART_Transmit_DMA(&huart1, tx_frame, TS_TX_FRAME_LEN);
}

void Tongsheng_Init(TONGSHENG_t* ts)
{
	ts->assist_level = 0;
	ts->light = 0;
	ts->walk = 0;
	ts->wheel_inch = 0;
	ts->speed_limit = 0;
	ts->rx_frames = 0;
	rx_tail = 0;
	if (HAL_UART_Receive_DMA(&huart1, rx_ring, TS_RX_BUFF) != HAL_OK)
	{
		Error_Handler();
	}
}

void Tongsheng_Service(TONGSHENG_t* ts)
{
	uint8_t head = (uint8_t)(TS_RX_BUFF - DMA1_Channel5->CNDTR) & (TS_RX_BUFF - 1);
	uint8_t avail = (uint8_t)(head - rx_tail) & (TS_RX_BUFF - 1);
	uint8_t frame[TS_RX_FRAME_LEN];
	uint8_t got_frame = 0;

	while (avail >= TS_RX_FRAME_LEN)
	{
		if (rx_ring[rx_tail] != TS_RX_HEADER) {
			rx_tail = (rx_tail + 1) & (TS_RX_BUFF - 1);
			avail--;
			continue;
		}
		for (uint8_t n = 0; n < TS_RX_FRAME_LEN; n++) frame[n] = rx_ring[(rx_tail + n) & (TS_RX_BUFF - 1)];

		if (ts_checksum(frame, TS_RX_FRAME_LEN - 1) == frame[TS_RX_FRAME_LEN - 1]) {
			ts_apply_rx(ts, frame);
			got_frame = 1;
			rx_tail = (rx_tail + TS_RX_FRAME_LEN) & (TS_RX_BUFF - 1);
			avail -= TS_RX_FRAME_LEN;
		} else {
			rx_tail = (rx_tail + 1) & (TS_RX_BUFF - 1);  // false header, resync
			avail--;
		}
	}

	if (got_frame) ts_send_status();
}

#endif // DISPLAY_TYPE_TONGSHENG
