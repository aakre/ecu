/*
 * ecu_can_messages.c
 *
 * Created: 25.05.2014 16:37:29
 *  Author: oyvinaak
 */ 

#include <asf.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "INVERTER_defines.h"
#include "queue_handles.h"
#include "revolve_can_definitions.h"
#include "ecu_can.h"
#include "ecu_can_mob.h"
#include "ecu_can_messages.h"


void ecu_can_send_to_inverter(uint8_t inverter_reg, uint16_t data) {
	inverter_can_msg_t message;
	
	message.data.u64 = 0x0LL;
	message.dlc = INVERTER_DLC_3;
	message.data.u32[0] = inverter_reg << 24 | data << 8;
	xQueueSendToBack(queue_to_inverter,&message,0);
}

void ecu_can_inverter_torque_cmd(int16_t torque) {
	/* This code also handles negative numbers */
	uint16_t torque_intel = ((torque >> 8) & 0xff) | ((torque & 0xff) << 8);
	
	inverter_can_msg_t message;
	
	message.data.u64 = 0x0LL;
	message.dlc = INVERTER_DLC_3;
	message.data.u32[0] = TORQUE_CMD << 24 | torque_intel << 8;
	
	xQueueSendToBack(queue_to_inverter,&message,0);
}

void ecu_can_inverter_read_torque_periodic() {
	ecu_can_send_to_inverter(READ_CMD, 0x90FA); //FA = 250 ms period
}

void ecu_can_inverter_read_reg(uint8_t inverter_reg) {
	/* Msg = 0x3D inverter_reg 00, ex: 0x3DE800 (read FRG_RUN) */
	
	inverter_can_msg_t message;
	
	message.data.u64 = 0x0LL;
	message.data.u32[0] = READ_CMD << 24 | inverter_reg << 16;
	message.dlc = INVERTER_DLC_3;
	
	xQueueSendToBack(queue_to_inverter,&message,0);
}


void ecu_can_blast(uint16_t data) {
	mob_debug.can_msg->data.u64		= 0x0LL;
	mob_debug.can_msg->data.u16[0]	= data;
	mob_debug.can_msg->id			= 0x123;
	mob_debug.dlc					= 2;
	
	can_tx(CAN_BUS_0,
	mob_debug.handle,
	mob_debug.dlc,
	CAN_DATA_FRAME,
	mob_debug.can_msg);
}


void ecu_can_blast32(uint32_t data) {
	mob_debug.can_msg->data.u64		= 0x0LL;
	mob_debug.can_msg->data.u32[0]	= data;
	mob_debug.can_msg->id			= 0x123;
	mob_debug.dlc					= 4;
	
	can_tx(CAN_BUS_0,
	mob_debug.handle,
	mob_debug.dlc,
	CAN_DATA_FRAME,
	mob_debug.can_msg);
}


void ecu_can_inverter_enable_drive() {
	ecu_can_send_to_inverter(MODE_REG, 0x0000);
}

void ecu_can_inverter_disable_drive() {
	ecu_can_send_to_inverter(MODE_REG, 0x0400);
}

void ecu_can_send_current_bms(uint16_t current) { //In use?
	// Current in [A] but BMS has unit 100[mA]
	mob_tx_bms.can_msg->data.u64	= 0x0LL;
	mob_tx_bms.can_msg->data.u16[0]	= current*10;
	
	can_tx(CAN_BUS_0,
	mob_tx_bms.handle,
	mob_tx_bms.dlc,
	CAN_DATA_FRAME,
	mob_tx_bms.can_msg);
}

void ecu_can_send_fast_data(uint16_t inverter_vdc, uint16_t ecu_error, uint16_t rpm, int16_t trq_cmd) {
	mob_ecu_fast_data.can_msg->data.u64	  = 0x0LL;
	mob_ecu_fast_data.can_msg->data.u16[0] = inverter_vdc;
	mob_ecu_fast_data.can_msg->data.u16[1] = ecu_error;
	mob_ecu_fast_data.can_msg->data.u16[2] = rpm;
	mob_ecu_fast_data.can_msg->data.s16[3] = trq_cmd;
	
	can_tx(CAN_BUS_0,
	mob_ecu_fast_data.handle,
	mob_ecu_fast_data.dlc,
	CAN_DATA_FRAME,
	mob_ecu_fast_data.can_msg);
}

void ecu_can_send_slow_data(uint16_t motor_temp, uint16_t inverter_temp, uint8_t max_trq) {
	mob_ecu_slow_data.can_msg->data.u64	 = 0x0LL;
	mob_ecu_slow_data.can_msg->data.u16[0] = motor_temp;
	mob_ecu_slow_data.can_msg->data.u16[1] = inverter_temp;
	mob_ecu_slow_data.can_msg->data.u8[4] = max_trq;
	
	can_tx(CAN_BUS_1,
	mob_ecu_slow_data.handle,
	mob_ecu_slow_data.dlc,
	CAN_DATA_FRAME,
	mob_ecu_slow_data.can_msg);
}

void ecu_can_speed_command(int16_t rpm_cmd) {
	// 	int16_t num = (rpm_cmd * 32767) / MAX_RPM;
	// 	uint16_t rpm_intel = ((num >> 8) & 0xff) | ((num & 0xff) << 8);
	// 	mob_tx_inverter.can_msg->data.u64 = 0x0LL;
	// 	mob_tx_inverter.can_msg->data.u32[0]	= SPEED_CMD << 24 | rpm_intel << 8;
	// 	mob_tx_inverter.can_msg->id				= INVERTER_ADDR_RX;
	// 	mob_tx_inverter.dlc						= INVERTER_DLC_3;
	//
	// 	can_tx(CAN_BUS_0,
	// 	mob_tx_inverter.handle,
	// 	mob_tx_inverter.dlc,
	// 	CAN_DATA_FRAME,
	// 	mob_tx_inverter.can_msg);
}

void ecu_can_send_tractive_system_active(void) {
	mob_tx_dash.can_msg->data.u64	 = 0x0LL;
	mob_tx_dash.can_msg->data.u16[0]  = 0x1;
	
	mob_tx_dash.can_msg->id = CANR_FCN_PRI_ID | CANR_GRP_ECU_ID | CANR_MODULE_ID0_ID;
	mob_tx_dash.dlc = 2;
	
	can_tx(CAN_BUS_0,
	mob_tx_dash.handle,
	mob_tx_dash.dlc,
	CAN_DATA_FRAME,
	mob_tx_dash.can_msg);
}

void ecu_can_send_play_rtds(void) {
	mob_tx_dash.can_msg->data.u64	 = 0x0LL;
	mob_tx_dash.can_msg->data.u16[0]  = 0x2;
	
	mob_tx_dash.can_msg->id = CANR_FCN_PRI_ID | CANR_GRP_ECU_ID | CANR_MODULE_ID0_ID;
	mob_tx_dash.dlc = 2;
	
	can_tx(CAN_BUS_0,
	mob_tx_dash.handle,
	mob_tx_dash.dlc,
	CAN_DATA_FRAME,
	mob_tx_dash.can_msg);
}

void ecu_can_send_ready_to_drive(void) {
	mob_tx_dash.can_msg->data.u64	 = 0x0LL;
	mob_tx_dash.can_msg->data.u16[0]  = 0x3;
	
	mob_tx_dash.can_msg->id = CANR_FCN_PRI_ID | CANR_GRP_ECU_ID | CANR_MODULE_ID0_ID;
	mob_tx_dash.dlc = 2;
	
	can_tx(CAN_BUS_0,
	mob_tx_dash.handle,
	mob_tx_dash.dlc,
	CAN_DATA_FRAME,
	mob_tx_dash.can_msg);
}

void ecu_can_send_drive_disabled(void) {
	mob_tx_dash.can_msg->data.u64	 = 0x0LL;
	mob_tx_dash.can_msg->data.u16[0]  = 0x4;
	
	mob_tx_dash.can_msg->id = CANR_FCN_PRI_ID | CANR_GRP_ECU_ID | CANR_MODULE_ID0_ID;
	mob_tx_dash.dlc = 2;
	
	can_tx(CAN_BUS_0,
	mob_tx_dash.handle,
	mob_tx_dash.dlc,
	CAN_DATA_FRAME,
	mob_tx_dash.can_msg);
}

void ecu_can_send_alive(uint8_t error) {
	mob_tx_dash.can_msg->data.u64	 = 0x0LL;
	mob_tx_dash.can_msg->data.u8[0]  = CANR_CMD_ALIVE;
	mob_tx_dash.can_msg->data.u8[1]  = DASH_ALIVE_ECU;
	if (error == 0) {
		mob_tx_dash.can_msg->data.u8[2]  = CANR_ALIVE_STATE_OPERATIVE;
		} else {
		mob_tx_dash.can_msg->data.u8[2]  = CANR_ALIVE_STATE_ERROR;
	}
	
	mob_tx_dash.can_msg->id = CANR_FCN_DATA_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID7_ID;
	mob_tx_dash.dlc = 3;
	
	can_tx(CAN_BUS_0,
	mob_tx_dash.handle,
	mob_tx_dash.dlc,
	CAN_DATA_FRAME,
	mob_tx_dash.can_msg);
}

void ecu_can_confirm_activate_launch(void) {
	mob_tx_dash.can_msg->data.u64	= 0x0LL;
	mob_tx_dash.can_msg->data.u8[0]	= 0x1;
	
	mob_tx_dash.can_msg->id = CANR_FCN_PRI_ID | CANR_GRP_ECU_ID | CANR_MODULE_ID1_ID;
	mob_tx_dash.dlc = 1;
	
	can_tx(CAN_BUS_0,
	mob_tx_dash.handle,
	mob_tx_dash.dlc,
	CAN_DATA_FRAME,
	mob_tx_dash.can_msg);
}

void ecu_can_send_launch_ready(void) {
	mob_tx_dash.can_msg->data.u64	= 0x0LL;
	mob_tx_dash.can_msg->data.u8[0]	= 0x2;
	
	mob_tx_dash.can_msg->id = CANR_FCN_PRI_ID | CANR_GRP_ECU_ID | CANR_MODULE_ID1_ID;
	mob_tx_dash.dlc = 1;
	
	can_tx(CAN_BUS_0,
	mob_tx_dash.handle,
	mob_tx_dash.dlc,
	CAN_DATA_FRAME,
	mob_tx_dash.can_msg);
}

void ecu_can_send_launch_stop(void) {
	mob_tx_dash.can_msg->data.u64	= 0x0LL;
	mob_tx_dash.can_msg->data.u8[0]	= 255;
	
	mob_tx_dash.can_msg->id = CANR_FCN_PRI_ID | CANR_GRP_ECU_ID | CANR_MODULE_ID1_ID;
	mob_tx_dash.dlc = 1;
	
	can_tx(CAN_BUS_0,
	mob_tx_dash.handle,
	mob_tx_dash.dlc,
	CAN_DATA_FRAME,
	mob_tx_dash.can_msg);
}

void ecu_can_send_slip_current(int16_t slip, uint16_t current) {
	mob_slip_current.can_msg->data.u64		= 0x0LL;
	mob_slip_current.can_msg->data.s16[0]	= slip;
	mob_slip_current.can_msg->data.u16[1]	= current;
	
	mob_slip_current.can_msg->id = (CANR_FCN_DATA_ID | CANR_GRP_ECU_ID | CANR_MODULE_ID7_ID);
	mob_slip_current.dlc = 4;
	
	can_tx(CAN_BUS_0,
	mob_slip_current.handle,
	mob_slip_current.dlc,
	CAN_DATA_FRAME,
	mob_slip_current.can_msg);
}

void fake_bms_can(void) {
	mob_slip_current.can_msg->data.u64	 = 0x0LL;
	mob_slip_current.can_msg->data.u8[3] = 1 << 3;//Set precharge
		
	mob_slip_current.can_msg->id = 0x429;
	mob_slip_current.dlc = 4;
		
	can_tx(CAN_BUS_1,
	mob_slip_current.handle,
	mob_slip_current.dlc,
	CAN_DATA_FRAME,
	mob_slip_current.can_msg);
}