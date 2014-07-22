/*
 * fsm_ecu_functions.c
 *
 * Created: 22.05.2014 17:26:07
 *  Author: oyvinaak
 */ 
#include <asf.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "INVERTER_defines.h"
#include "queue_handles.h"
#include "fsm_ecu.h"
#include "fsm_ecu_functions.h"

#define BMS_PRECHARGE_BIT			3
#define BRAKE_PLAUSIBILITY_TIME_LIMIT 25
#define BRAKE_KERS_TRESHOLD			5000 // ???
#define BRAKE_TRESHOLD				5000
#define MAX_KERS					(int16_t)-3277


bool brake_over_travel_check (fsm_ecu_data_t* ecu_data) {
	return true;
}

bool brake_plausibility_check(fsm_ecu_data_t *ecu_data) {
	static uint8_t plausibility_timer = 0;
	int16_t trq_sens = max(ecu_data->trq_sens0, ecu_data->trq_sens1); //Changed to max 10/6-14
	
	if ((trq_sens > 250) && (ecu_data->brake_front > BRAKE_TRESHOLD)) {
		plausibility_timer++;
		if (plausibility_timer == BRAKE_PLAUSIBILITY_TIME_LIMIT) {
			plausibility_timer = 0;
			return false;
		}
	} else {
		plausibility_timer = 0; //Added 10/6-14
	}
	return true;
	
// 	if ((trq_sens > 250) && (ecu_data->brake_front > BRAKE_TRESHOLD)) {
// 		return false;
// 	}
// 	return true;
}

bool torque_plausibility_check(fsm_ecu_data_t *ecu_data) {
	/* Torque sensors = <0, 1000> */
	if ( ecu_data->trq_sens0_err || ecu_data->trq_sens1_err) {
		asm("nop");
		return false;
	} else {
		int16_t deviation = max(ecu_data->trq_sens0, ecu_data->trq_sens1) - min(ecu_data->trq_sens0,ecu_data->trq_sens1);
		if (deviation > 100) {
			asm("nop");
			return false;
		}
		return true;
	}
}

uint16_t calc_bms_power(fsm_ecu_data_t *ecu_data) {
	int16_t current = ecu_data->bms_current;
	int16_t voltage = ecu_data->vdc_battery;
	return (uint16_t)(voltage*current);
}

uint16_t calc_inverter_power(fsm_ecu_data_t *ecu_data) {
	uint16_t rpm = ecu_data->rpm;
	uint16_t trq = (uint16_t)ecu_data->trq_cmd*180/MAX_TORQUE;//180Nm
	uint32_t power = trq*rpm*628/(100*60*1000); //In kW
	return (uint16_t)power; //Such number
}

int16_t calc_kers(fsm_ecu_data_t *ecu_data) {
	float speed = ecu_data->WRR_sens & 0xFF;
	speed = speed*2.574;
	static bool allow_kers = false;
	
	if (speed > 10) {
		allow_kers = true;
	}
	
	if (allow_kers) {
		if ((ecu_data->trq_sens0 < 20) && (ecu_data->trq_sens1 < 20)) {
			if (speed > 5.5) { //km/h
				if ((ecu_data->max_cell_temp > 0) && (ecu_data->max_cell_temp < 45)) {
					return (MAX_KERS*ecu_data->kers_factor)/100; //TODO Return a value from ecu_data that is received from dash
				}
			} else {
				allow_kers = false;
			}
		}
	}
	return 0;
}

uint16_t calc_max_current_allowed(fsm_ecu_data_t* ecu_data) {
	if (!(ecu_data->vdc_battery == 0)) {
		return (82500 / ecu_data->vdc_battery);	
	}
	return 0;
}

uint8_t check_bspd(void) {
	uint8_t temp=0;
	xQueueReceive(queue_bspd, &temp, 0);
	return temp;
}

uint8_t check_inverter_error(fsm_ecu_data_t *ecu_data) {
	uint16_t temp = ecu_data->inverter_error;
	return (uint8_t)(temp & 1 << PWR_FAULT) | (temp & 1 << RFE_FAULT) | (temp & 1 << RESOLVER_FAULT);
}

uint8_t check_inverter_timeout(fsm_ecu_data_t *ecu_data) {
	if (ecu_data->inverter_timeout == TIMER_10_HZ) {
		return 1;
	}
	return 0;
}

uint8_t get_brake_sens(fsm_ecu_data_t *ecu_data) {
	uint8_t status = 0;
	if (xQueueReceive( queue_brake_front, &ecu_data->brake_front, 0 ) == pdFALSE) {
		status |= 1 << 0;
	}
	if (xQueueReceive( queue_brake_rear, &ecu_data->brake_rear, 0 ) == pdFALSE) {
		status |= 1 << 1;
	}
	return status;
}

uint8_t get_speed_sens(fsm_ecu_data_t *ecu_data) {
	uint8_t status=0;
	if (xQueueReceive( queue_wheel_fl, &ecu_data->WFL_sens, 0 ) == pdFALSE) {
		status++;
	}
	if (xQueueReceive( queue_wheel_fr, &ecu_data->WFR_sens, 0 ) == pdFALSE) {
		status++;
	}
	if (xQueueReceive( queue_wheel_rl, &ecu_data->WRL_sens, 0 ) == pdFALSE) {
		status++;
	}
	if (xQueueReceive( queue_wheel_rr, &ecu_data->WRR_sens, 0 ) == pdFALSE) {
		status++;
	}
	asm("nop");
	return status;
}

uint8_t get_trq_sens(fsm_ecu_data_t *ecu_data) {
	uint8_t status = 0;
	if (xQueueReceive( queue_trq_sens0, &ecu_data->trq_sens0, 0 ) == pdFALSE ) {
		status++;
	}
	if (xQueueReceive( queue_trq_sens1, &ecu_data->trq_sens1, 0 ) == pdFALSE ) {
		status++;
	}
	if (xQueueReceive( queue_trq_sens0_err, &ecu_data->trq_sens0_err, 0 ) == pdFALSE ) {
		status++;
	}
	if (xQueueReceive( queue_trq_sens1_err, &ecu_data->trq_sens1_err, 0 ) == pdFALSE ) {
		status++;
	}
	return status;
}

uint16_t convert_num_to_vdc(uint32_t num) {
	/* num = 33.2*vdc - 827 */
	uint32_t num_be = convert_to_big_endian(num);
	return (uint16_t)(float)((num_be + 827) / 33.2);
}

uint16_t convert_to_big_endian(uint32_t data) {
	/* Input: 0x49 D2 2A 00 Output:0x2A D2 */
	uint16_t relevant_data = (data & 0xFFFF00) >> 8;
	return ((relevant_data & 0xFF) << 8 | (relevant_data & 0xFF00) >> 8);
}

void ecu_dio_inverter_clear_error() {
	gpio_set_pin_high(INVERTER_DIN1);
	delay_us(200);
	gpio_set_pin_low(INVERTER_DIN1);
}

void get_new_data(fsm_ecu_data_t *ecu_data) {
	uint8_t i;
	for (i=0; i<QUEUE_INVERTER_RX_LEN; i++) {
		if (xQueueReceive( queue_from_inverter, &ecu_data->inverter_can_msg, 0 ) == pdTRUE) {
			handle_inverter_data(ecu_data);
			ecu_data->inverter_timeout = 0;
		} else {
			ecu_data->inverter_timeout++;
			break;
		}
	}
	
	for (i=0; i<QUEUE_DASH_MSG_LEN; i++) {
		if (xQueueReceive( queue_dash_msg, &ecu_data->dash_msg, 0 ) == pdTRUE) {
			handle_dash_data(ecu_data);
		} else {
			break;
		}
	}
	
	for (i=0; i<QUEUE_BMS_RX_LEN; i++) {
		if (xQueueReceive( queue_bms_rx, &ecu_data->bms_msg, 0 ) == pdTRUE) {
			handle_bms_data(ecu_data);
		} else {
			break;
		}
	}
}

void handle_bms_data(fsm_ecu_data_t *ecu_data) {
	/* Max period 300 ms, contactor req, battery current input msg */
	switch (ecu_data->bms_msg.id) {
		case (BMS_PRECHARGE_ID):
		if ((ecu_data->bms_msg.data.u8[3] & (1 << BMS_PRECHARGE_BIT)) != 0) {
			/* There is a hardwire contactor request */
			ecu_data->flag_start_precharge = 1;
		} else {
			ecu_data->flag_start_precharge = 0;
		}
		break;
		case (BMS_BATT_VOLT_ID):
		ecu_data->vdc_battery = ecu_data->bms_msg.data.u16[0];
		break;
		case (BMS_BATT_TEMP_ID):
		ecu_data->max_cell_temp = ecu_data->bms_msg.data.s8[4];
		break;
		
		case (0x42B):
		ecu_data->bms_current = ecu_data->bms_msg.data.s16[0];
		break;
		
		default:
		break;
	}
}

void handle_dash_data(fsm_ecu_data_t *ecu_data) {
	uint8_t rtds_plays;
	uint8_t start;
	uint8_t lc_filter_time;
	uint8_t state_of_lc = 0;
	switch (ecu_data->dash_msg.id) {
		case (CANR_FCN_PRI_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID0_ID):
		rtds_plays = ecu_data->dash_msg.data.u8[0];
		if (rtds_plays == 1) {
			ecu_data->flag_drive_enable = DRIVE_ENABLE_RTDS_PLAYS;
		}
		break;
		
		case (CANR_FCN_PRI_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID1_ID):
		start = ecu_data->dash_msg.data.u8[0];
		//uint8_t tractive = ecu_data->dash_msg.data.u8[1];
		ecu_data->kers_factor = ecu_data->dash_msg.data.s16[1];
		ecu_data->slip_target = ecu_data->dash_msg.data.u16[2]/100.0;

		if (start == 0) {
			ecu_data->flag_drive_enable = DRIVE_DISABLE_REQUEST;
		} else if (start == 1) {
			ecu_data->flag_drive_enable = DRIVE_ENABLE_REQUEST;
		}
		asm("nop");
		break;
		
		case (CANR_FCN_PRI_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID2_ID):
		//PID gains
		ecu_data->Kp = ecu_data->dash_msg.data.u16[0]/10.0;
		ecu_data->Ki = ecu_data->dash_msg.data.u16[1]/10.0;
		ecu_data->Kd = ecu_data->dash_msg.data.u16[2]/10.0;
		ecu_data->d_filter_gain = (N_filter*Ts/(ecu_data->Kd+N_filter*Ts));
		asm("nop");
		break;
		
		case (CANR_FCN_DATA_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID3_ID):
		//Only permit if ECU in error
		if (ecu_data->state == STATE_ERROR) {
			if (ecu_data->dash_msg.data.u8[1]==1) {// Message also contains the current driver
				ecu_data->reboot = 1;
			}
		}
		asm("nop");
		break;
		
		case (CANR_FCN_PRI_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID3_ID):
		ecu_data->config_max_trq = ecu_data->dash_msg.data.u8[1];
		break;
		
		case (CANR_FCN_PRI_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID4_ID):
		ecu_data->lc_trq_init = ecu_data->dash_msg.data.u8[0];
		lc_filter_time = ecu_data->dash_msg.data.u8[1];
		ecu_data->lc_filter_gain = Ts/(Ts + lc_filter_time);
		break;
		
		case (CANR_FCN_PRI_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID5_ID):
		state_of_lc = ecu_data->dash_msg.data.u8[0];
		ecu_data->launch_control_flag = (launch_control_t)state_of_lc;
		break;
		
		default:
		break;
	}
}

void handle_inverter_data(fsm_ecu_data_t *ecu_data) {
	/* Note on receiving inverter data
	 * Most data is 4 byte long, but e.g. error and state
	 * register will produce a 6 byte message.
	 * When checking for FRG_BIT it is implied that the message
	 * consist of 4 bytes etc. Refer to manual for the individual
	 * bits in state and error reg.
	 * Temp e.g.: 49d62a00 
	 */
	uint16_t temp;
	switch (ecu_data->inverter_can_msg.data.u8[0]) {
		case BTB_REG:
			break;
		case FRG_REG:
			break;
		case MOTOR_TEMP_REG:
			ecu_data->motor_temp = convert_to_big_endian(ecu_data->inverter_can_msg.data.u32[0]);
			break;
		case IGBT_TEMP_REG:
			ecu_data->inverter_temp = convert_to_big_endian(ecu_data->inverter_can_msg.data.u32[0]);
			break;
		case CURRENT_REG:
			break;
		case VDC_REG:
			/* 16 bit value */
			temp = convert_num_to_vdc(ecu_data->inverter_can_msg.data.u32[0]);
			if (temp < 30) {
				ecu_data->inverter_vdc = 0;	
			} else {
				ecu_data->inverter_vdc = temp;
			}	 
			break;
		case RPM_REG:
			ecu_data->rpm = (MAX_RPM * convert_to_big_endian(ecu_data->inverter_can_msg.data.u32[0])) / 32767;
			break;
		case ERROR_REG:
			ecu_data->inverter_error = (ecu_data->inverter_can_msg.data.u32[0] & 0x00FFFF00) >> 8;
			break;
		default:
			break;
	}
}

void map_pedal(fsm_ecu_data_t *ecu_data) {
	// Torque sensors = <0,1000>
	static float pedal_filter = 0.0F;
	float config_max_trq = (float)ecu_data->config_max_trq / 100.0;
	
	int16_t trq_sens = (int16_t)min(ecu_data->trq_sens0, ecu_data->trq_sens1);
	if (trq_sens > 20) { //Increase?
		// Handle values below 0
		trq_sens = max(0, trq_sens);
		// Handle values above 1000
		trq_sens = min(trq_sens, 1000);
		
		float pedal = (float)MAX_TORQUE*(float)trq_sens*(float)trq_sens*config_max_trq/1000000.0;
		//float pedal = (float)MAX_TORQUE*(float)trq_sens*config_max_trq/1000.0; //Linear curve
		pedal_filter = (1-PEDAL_FILTER_GAIN)*pedal_filter + PEDAL_FILTER_GAIN*pedal;
		
		ecu_data->trq_pedal = min(pedal_filter, pedal); //Selects filter when input increases, pedal when decreases
	} else {
		ecu_data->trq_pedal = 0;
	}
}


