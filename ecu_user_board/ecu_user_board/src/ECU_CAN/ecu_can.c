/*
 * ecu_can.c
 *
 * Created: 20.02.2014 13:00:52
 *  Author: oyvinaak
 */ 

#include <asf.h>
#include "ecu_can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "INVERTER_defines.h"
#include "queue_handles.h"
#include "ecu_can_mob.h"
/* Note on reception using .u64
 * Format 0xDATADATA		DATADATA
 *          u32[0]			u32[1]
 *			u16[0]u16[1]	u16[2]u16[3]
 */


/* Prototypes */
void can_out_callback_channel0(U8 handle, U8 event);
void can_out_callback_channel1(U8 handle, U8 event);

/* Allocate CAN mobs */
volatile can_msg_t mob_ram_ch0[NB_MOB_CHANNEL] __attribute__ ((section (".hsb_ram_loc")));
volatile can_msg_t mob_ram_ch1[NB_MOB_CHANNEL] __attribute__ ((section (".hsb_ram_loc")));


void ecu_can_init(void) {
	/* Setup the generic clock for CAN output */
	scif_gc_setup(
		AVR32_SCIF_GCLK_CANIF,
		SCIF_GCCTRL_OSC0,
		AVR32_SCIF_GC_NO_DIV_CLOCK,
		0
	);
	/* Now enable the generic clock input for the CAN module */
	scif_gc_enable(AVR32_SCIF_GCLK_CANIF);

	static const gpio_map_t CAN_GPIO_MAP = {
		{CAN0_RX_PIN, CAN0_RX_FUNCTION},
		{CAN0_TX_PIN, CAN0_TX_FUNCTION},
		{CAN1_RX_PIN, CAN1_RX_FUNCTION},
		{CAN1_TX_PIN, CAN1_TX_FUNCTION}
	};
	
	/* Assign GPIO to CAN. */
	gpio_enable_module(CAN_GPIO_MAP, sizeof(CAN_GPIO_MAP) / sizeof(CAN_GPIO_MAP[0]));
	

	/* Initialize interrupt vectors. */
	INTC_init_interrupts();
	
	/* Allocate channel message box */
	mob_rx_speed_sens_fl.handle	= 0;
	mob_rx_speed_sens_fr.handle	= 1;
	mob_rx_speed_sens_rl.handle	= 2;
	mob_rx_speed_sens_rr.handle	= 3;
	mob_rx_dash_pri.handle		= 4;
	mob_tx_dash.handle			= 5;
	mob_rx_trq_sens0.handle		= 6;
	mob_rx_trq_sens1.handle     = 7;
	mob_ecu_slow_data.handle	= 8;
	mob_rx_bms_precharge.handle	= 9;
	mob_brk.handle				= 10;
	mob_ecu_fast_data.handle	= 11;
	mob_rx_bms_battvolt.handle  = 12;
	mob_rx_bspd.handle			= 13;
	mob_rx_dash_data.handle		= 14;
	mob_slip_current.handle     = 15;


	/* Initialize CAN channels */
	can_init(CAN_BUS_0, ((uint32_t)&mob_ram_ch0[0]), CANIF_CHANNEL_MODE_NORMAL,	can_out_callback_channel0);
	can_init(CAN_BUS_1, ((uint32_t)&mob_ram_ch1[0]), CANIF_CHANNEL_MODE_NORMAL,	can_out_callback_channel1);
	
	
	/* Prepare for message reception */	
	can_rx(
		CAN_BUS_1
		, mob_rx_speed_sens_fl.handle
		, mob_rx_speed_sens_fl.req_type
		, mob_rx_speed_sens_fl.can_msg
	);
	
	can_rx(
		CAN_BUS_1
		, mob_rx_speed_sens_fr.handle
		, mob_rx_speed_sens_fr.req_type
		, mob_rx_speed_sens_fr.can_msg
	);
	
	
	can_rx(
		CAN_BUS_1
		, mob_rx_speed_sens_rl.handle
		, mob_rx_speed_sens_rl.req_type
		, mob_rx_speed_sens_rl.can_msg
	);
	
	can_rx(
		CAN_BUS_1
		, mob_rx_speed_sens_rr.handle
		, mob_rx_speed_sens_rr.req_type
		, mob_rx_speed_sens_rr.can_msg
	);
	
	can_rx(
		CAN_BUS_0
		, mob_rx_dash_pri.handle
		, mob_rx_dash_pri.req_type
		, mob_rx_dash_pri.can_msg
	);
	
	can_rx(
		CAN_BUS_0
		, mob_rx_dash_data.handle
		, mob_rx_dash_data.req_type
		, mob_rx_dash_data.can_msg
	);
	
	can_rx(
		CAN_BUS_0
		, mob_rx_trq_sens0.handle
		, mob_rx_trq_sens0.req_type
		, mob_rx_trq_sens0.can_msg
	);

	can_rx(
		CAN_BUS_1
		, mob_rx_trq_sens1.handle
		, mob_rx_trq_sens1.req_type
		, mob_rx_trq_sens1.can_msg
	);

	
	can_rx(
		CAN_BUS_1
		, mob_rx_bms_precharge.handle
		, mob_rx_bms_precharge.req_type
		, mob_rx_bms_precharge.can_msg
	);
	
	can_rx(
		CAN_BUS_1
		, mob_rx_bms_battvolt.handle
		, mob_rx_bms_battvolt.req_type
		, mob_rx_bms_battvolt.can_msg
	);
	
	can_rx(
		CAN_BUS_1
		, mob_brk.handle
		, mob_brk.req_type
		, mob_brk.can_msg
	);
	
	can_rx(
		CAN_BUS_0
		, mob_rx_bspd.handle
		, mob_rx_bspd.req_type
		, mob_rx_bspd.can_msg
	);
	asm("nop");
}

void can_out_callback_channel0(U8 handle, U8 event){
	if (handle == mob_rx_dash_pri.handle) {
		mob_rx_dash_pri.can_msg->data.u64	= can_get_mob_data(CAN_BUS_0, handle).u64;
		mob_rx_dash_pri.can_msg->id			= can_get_mob_id(CAN_BUS_0, handle);
		mob_rx_dash_pri.dlc					= can_get_mob_dlc(CAN_BUS_0, handle);
		mob_rx_dash_pri.status				= event;
		
		dash_can_msg_t dash_can_msg;
		
		dash_can_msg.data.u64 = mob_rx_dash_pri.can_msg->data.u64;
		dash_can_msg.id = mob_rx_dash_pri.can_msg->id;
		xQueueSendToBackFromISR(queue_dash_msg, &dash_can_msg, NULL);
		/* Empty message field */
		mob_rx_dash_pri.can_msg->data.u64 = 0x0LL;
		
		/* Prepare message reception */
		can_rx(CAN_BUS_0,
		mob_rx_dash_pri.handle,
		mob_rx_dash_pri.req_type,
		mob_rx_dash_pri.can_msg);
		
	} else if (handle == mob_rx_dash_data.handle) {
		mob_rx_dash_data.can_msg->data.u64	= can_get_mob_data(CAN_BUS_0, handle).u64;
		mob_rx_dash_data.can_msg->id			= can_get_mob_id(CAN_BUS_0, handle);
		mob_rx_dash_data.dlc					= can_get_mob_dlc(CAN_BUS_0, handle);
		mob_rx_dash_data.status				= event;
		
		dash_can_msg_t dash_can_msg;
		
		dash_can_msg.data.u64 = mob_rx_dash_data.can_msg->data.u64;
		dash_can_msg.id = mob_rx_dash_data.can_msg->id;
		xQueueSendToBackFromISR(queue_dash_msg, &dash_can_msg, NULL);
		/* Empty message field */
		mob_rx_dash_data.can_msg->data.u64 = 0x0LL;
		
		/* Prepare message reception */
		can_rx(CAN_BUS_0, 
		mob_rx_dash_data.handle,
		mob_rx_dash_data.req_type,
		mob_rx_dash_data.can_msg);
		
	} else if (handle == mob_rx_trq_sens0.handle) {
		mob_rx_trq_sens0.can_msg->data.u64	= can_get_mob_data(CAN_BUS_0, handle).u64;
		mob_rx_trq_sens0.can_msg->id		= can_get_mob_id(CAN_BUS_0, handle);
		mob_rx_trq_sens0.dlc				= can_get_mob_dlc(CAN_BUS_0, handle);
		mob_rx_trq_sens0.status				= event;
	
		xQueueOverwriteFromISR(queue_trq_sens0, &mob_rx_trq_sens0.can_msg->data.s16[0], NULL);
		xQueueOverwriteFromISR(queue_trq_sens0_err, &mob_rx_trq_sens0.can_msg->data.u8[2], NULL);
		asm("nop");
		/* Empty message field */
		mob_rx_trq_sens0.can_msg->data.u64 = 0x0LL;
		
		/* Prepare message reception */
		can_rx(CAN_BUS_0, 
		mob_rx_trq_sens0.handle,
		mob_rx_trq_sens0.req_type,
		mob_rx_trq_sens0.can_msg);
		
	}	else if (handle == mob_rx_bspd.handle) {
		mob_rx_bspd.can_msg->data.u64	= can_get_mob_data(CAN_BUS_0, handle).u64;
		mob_rx_bspd.can_msg->id			= can_get_mob_id(CAN_BUS_0, handle);
		mob_rx_bspd.dlc					= can_get_mob_dlc(CAN_BUS_0, handle);
		mob_rx_bspd.status				= event;
		
		xQueueOverwriteFromISR( queue_bspd, &mob_rx_bspd.can_msg->data.u8[0], NULL );
		/* Empty message field */
		mob_rx_bspd.can_msg->data.u64 = 0x0LL;
		/* Prepare message reception */
		can_rx(CAN_BUS_0,
		mob_rx_bspd.handle,
		mob_rx_bspd.req_type,
		mob_rx_bspd.can_msg);
	} 
}

/* Call Back called by can_drv, channel 1 */
void can_out_callback_channel1(U8 handle, U8 event){
	if (handle == mob_rx_speed_sens_fl.handle) {
		mob_rx_speed_sens_fl.can_msg->data.u64	= can_get_mob_data(CAN_BUS_1, handle).u64;
		mob_rx_speed_sens_fl.can_msg->id		= can_get_mob_id(CAN_BUS_1, handle);
		mob_rx_speed_sens_fl.dlc				= can_get_mob_dlc(CAN_BUS_1, handle);
		mob_rx_speed_sens_fl.status				= event;
		
		xQueueOverwriteFromISR(queue_wheel_fl, &mob_rx_speed_sens_fl.can_msg->data.u16[0], NULL);
		/* Empty message field */
		mob_rx_speed_sens_fl.can_msg->data.u64 = 0x0LL;
		
		/* Prepare message reception */
		can_rx(CAN_BUS_1,
		mob_rx_speed_sens_fl.handle,
		mob_rx_speed_sens_fl.req_type,
		mob_rx_speed_sens_fl.can_msg);
		
	} else if (handle == mob_rx_speed_sens_fr.handle) {
		mob_rx_speed_sens_fr.can_msg->data.u64	= can_get_mob_data(CAN_BUS_1, handle).u64;
		mob_rx_speed_sens_fr.can_msg->id		= can_get_mob_id(CAN_BUS_1, handle);
		mob_rx_speed_sens_fr.dlc				= can_get_mob_dlc(CAN_BUS_1, handle);
		mob_rx_speed_sens_fr.status				= event;
		
		xQueueOverwriteFromISR(queue_wheel_fr, &mob_rx_speed_sens_fr.can_msg->data.u16[0], NULL);
		/* Empty message field */
		mob_rx_speed_sens_fr.can_msg->data.u64 = 0x0LL;
		
		/* Prepare message reception */
		can_rx(CAN_BUS_1,
		mob_rx_speed_sens_fr.handle,
		mob_rx_speed_sens_fr.req_type,
		mob_rx_speed_sens_fr.can_msg);
	} else if (handle == mob_rx_speed_sens_rl.handle) {
		mob_rx_speed_sens_rl.can_msg->data.u64	= can_get_mob_data(CAN_BUS_1, handle).u64;
		mob_rx_speed_sens_rl.can_msg->id			= can_get_mob_id(CAN_BUS_1, handle);
		mob_rx_speed_sens_rl.dlc					= can_get_mob_dlc(CAN_BUS_1, handle);
		mob_rx_speed_sens_rl.status				= event;

		xQueueOverwriteFromISR(queue_wheel_rl, &mob_rx_speed_sens_rl.can_msg->data.u16[0], NULL);
	
		/* Empty message field */
		mob_rx_speed_sens_rl.can_msg->data.u64 = 0x0LL;
		
		/* Prepare message reception */
		can_rx(CAN_BUS_1, 
		mob_rx_speed_sens_rl.handle,
		mob_rx_speed_sens_rl.req_type,
		mob_rx_speed_sens_rl.can_msg);
		
	} else if (handle == mob_rx_speed_sens_rr.handle) {
		mob_rx_speed_sens_rr.can_msg->data.u64	= can_get_mob_data(CAN_BUS_1, handle).u64;
		mob_rx_speed_sens_rr.can_msg->id		= can_get_mob_id(CAN_BUS_1, handle);
		mob_rx_speed_sens_rr.dlc				= can_get_mob_dlc(CAN_BUS_1, handle);
		mob_rx_speed_sens_rr.status				= event;

		xQueueOverwriteFromISR(queue_wheel_rr, &mob_rx_speed_sens_rr.can_msg->data.u16[0], NULL);
		
		/* Empty message field */
		mob_rx_speed_sens_rr.can_msg->data.u64 = 0x0LL;
		
		/* Prepare message reception */
		can_rx(CAN_BUS_1,
		mob_rx_speed_sens_rr.handle,
		mob_rx_speed_sens_rr.req_type,
		mob_rx_speed_sens_rr.can_msg);
	
	}	else if (handle == mob_rx_trq_sens1.handle) {
		mob_rx_trq_sens1.can_msg->data.u64	= can_get_mob_data(CAN_BUS_1, handle).u64;
		mob_rx_trq_sens1.can_msg->id			= can_get_mob_id(CAN_BUS_1, handle);
		mob_rx_trq_sens1.dlc					= can_get_mob_dlc(CAN_BUS_1, handle);
		mob_rx_trq_sens1.status					= event;
		
		xQueueOverwriteFromISR(queue_trq_sens1, &mob_rx_trq_sens1.can_msg->data.s16[0], NULL);
		xQueueOverwriteFromISR(queue_trq_sens1_err, &mob_rx_trq_sens1.can_msg->data.u8[2], NULL);
		asm("nop");
		/* Empty message field */
		mob_rx_trq_sens1.can_msg->data.u64 = 0x0LL;
		
		/* Prepare message reception */
		can_rx(CAN_BUS_1, 
		mob_rx_trq_sens1.handle,
		mob_rx_trq_sens1.req_type,
		mob_rx_trq_sens1.can_msg);
	
	} else if (handle == mob_rx_bms_precharge.handle) {
		mob_rx_bms_precharge.can_msg->data.u64	= can_get_mob_data(CAN_BUS_1, handle).u64;
		mob_rx_bms_precharge.can_msg->id		= can_get_mob_id(CAN_BUS_1, handle);
		mob_rx_bms_precharge.dlc				= can_get_mob_dlc(CAN_BUS_1, handle);
		mob_rx_bms_precharge.status				= event;
		
		bms_can_msg_t bms_can_msg;
		bms_can_msg.data.u64 = mob_rx_bms_precharge.can_msg->data.u64;
		bms_can_msg.id = mob_rx_bms_precharge.can_msg->id;
		xQueueSendToBackFromISR(queue_bms_rx, &bms_can_msg, NULL);
		/* Empty message field */
		mob_rx_bms_precharge.can_msg->data.u64 = 0x0LL;
		/* Prepare message reception */
		can_rx(CAN_BUS_1,
		mob_rx_bms_precharge.handle,
		mob_rx_bms_precharge.req_type,
		mob_rx_bms_precharge.can_msg);
		
	} else if (handle == mob_rx_bms_battvolt.handle) {
		mob_rx_bms_battvolt.can_msg->data.u64	= can_get_mob_data(CAN_BUS_1, handle).u64;
		mob_rx_bms_battvolt.can_msg->id			= can_get_mob_id(CAN_BUS_1, handle);
		mob_rx_bms_battvolt.dlc					= can_get_mob_dlc(CAN_BUS_1, handle);
		mob_rx_bms_battvolt.status				= event;
		
		bms_can_msg_t bms_can_msg;
		bms_can_msg.data.u64 = mob_rx_bms_battvolt.can_msg->data.u64;
		bms_can_msg.id = mob_rx_bms_battvolt.can_msg->id;
		
		xQueueSendToBackFromISR(queue_bms_rx, &bms_can_msg, NULL);
		/* Empty message field */
		mob_rx_bms_battvolt.can_msg->data.u64 = 0x0LL;
		/* Prepare message reception */
		can_rx(CAN_BUS_1,
		mob_rx_bms_battvolt.handle,
		mob_rx_bms_battvolt.req_type,
		mob_rx_bms_battvolt.can_msg);	
	} else if (handle == mob_brk.handle) {
		mob_brk.can_msg->data.u64	= can_get_mob_data(CAN_BUS_1, handle).u64;
		mob_brk.can_msg->id			= can_get_mob_id(CAN_BUS_1, handle);
		mob_brk.dlc					= can_get_mob_dlc(CAN_BUS_1, handle);
		mob_brk.status				= event;
		
		if (mob_brk.can_msg->id == (CANR_FCN_DATA_ID | CANR_GRP_SENS_BRK_ID | CANR_MODULE_ID0_ID)) {
			xQueueSendToBackFromISR( queue_brake_front, &mob_brk.can_msg->data.u16[0], NULL );
		} else {
			xQueueSendToBackFromISR( queue_brake_rear, &mob_brk.can_msg->data.u16[0], NULL );
		}
		/* Empty message field */
		mob_brk.can_msg->data.u64 = 0x0LL;
		/* Prepare message reception */
		can_rx(CAN_BUS_1,
		mob_brk.handle,
		mob_brk.req_type,
		mob_brk.can_msg);
	}
}

