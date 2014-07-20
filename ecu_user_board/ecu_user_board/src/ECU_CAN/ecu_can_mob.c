/*
 * ecu_can_mob.c
 *
 * Created: 25.05.2014 16:48:02
 *  Author: oyvinaak
 */ 


#include <asf.h>
#include "INVERTER_defines.h"
#include "ecu_can_mob.h"
#include "revolve_can_definitions.h"



can_msg_t msg_rx_speed_sens_fl = {
	{
		{
			.id			= CANR_FCN_DATA_ID | CANR_GRP_SENS_SPEED_ID | CANR_MODULE_ID0_ID,
			.id_mask	= 0x7FF,
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_rx_speed_sens_fl = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_rx_speed_sens_fl,
	CAN_WHEEL_DLC,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};

can_msg_t msg_rx_speed_sens_fr = {
	{
		{
			.id			= CANR_FCN_DATA_ID | CANR_GRP_SENS_SPEED_ID | CANR_MODULE_ID1_ID,
			.id_mask	= 0x7FF,
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_rx_speed_sens_fr = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_rx_speed_sens_fr,
	CAN_WHEEL_DLC,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};

can_msg_t msg_rx_speed_sens_rl = {
	{
		{
			.id			= CANR_FCN_DATA_ID | CANR_GRP_SENS_SPEED_ID | CANR_MODULE_ID2_ID,
			.id_mask	= 0x7FF,
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_rx_speed_sens_rl = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_rx_speed_sens_rl,
	CAN_WHEEL_DLC,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};

can_msg_t msg_rx_speed_sens_rr = {
	{
		{
			.id			= CANR_FCN_DATA_ID | CANR_GRP_SENS_SPEED_ID | CANR_MODULE_ID3_ID,
			.id_mask	= 0x7FF,
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_rx_speed_sens_rr = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_rx_speed_sens_rr,
	CAN_WHEEL_DLC,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};

can_msg_t msg_rx_dash_data = {
	{
		{
			.id			= CANR_FCN_DATA_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID3_ID,
			.id_mask	= 0x7FF,
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_rx_dash_data = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_rx_dash_data,
	8,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};

can_msg_t msg_rx_dash_pri = {
	{
		{
			.id			= CANR_FCN_PRI_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID0_ID,
			.id_mask	= 0x7F8,
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_rx_dash_pri = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_rx_dash_pri,
	8,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};


can_msg_t msg_rx_trq_sens0  = {
	{
		{
			.id			= CANR_FCN_PRI_ID | CANR_GRP_SENS_TRQ_ID | CANR_MODULE_ID0_ID,
			.id_mask	= 0x7F8,
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_rx_trq_sens0 = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_rx_trq_sens0,
	3,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};

can_msg_t msg_rx_trq_sens1  = {
	{
		{
			.id			= CANR_FCN_PRI_ID | CANR_GRP_SENS_TRQ_ID | CANR_MODULE_ID1_ID,
			.id_mask	= 0x7F8,
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_rx_trq_sens1 = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_rx_trq_sens1,
	3,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};



can_msg_t msg_debug  = {
	{
		{
			.id			= 0x123,
			.id_mask	= 0x7F8,
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_debug  = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_debug,
	4,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};

//
can_msg_t msg_ecu_slow_data  = {
	{
		{
			.id			= CANR_FCN_DATA_ID | CANR_GRP_ECU_ID | CANR_MODULE_ID1_ID,
			.id_mask	= 0x7F8,
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_ecu_slow_data  = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_ecu_slow_data,
	5,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};

can_msg_t msg_ecu_fast_data  = {
	{
		{
			.id			= CANR_FCN_DATA_ID | CANR_GRP_ECU_ID | CANR_MODULE_ID0_ID,
			.id_mask	= 0x7F8,
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_ecu_fast_data = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_ecu_fast_data,
	8,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};

can_msg_t msg_tx_dash  = {
	{
		{
			.id			= CANR_FCN_CMD_ID | CANR_GRP_ECU_ID | CANR_MODULE_ID0_ID,
			.id_mask	= 0x7FF,
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_tx_dash = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_tx_dash,
	1,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};

can_msg_t msg_tx_bms  = {
	// Addr 633 by default
	{
		{
			.id			= CANR_FCN_CMD_ID | 0x27 | CANR_MODULE_ID4_ID,
			.id_mask	= 0x7F8,
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_tx_bms  = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_tx_bms,
	8,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};

can_msg_t msg_rx_bms_precharge  = {
	{
		{
			.id			= 0x429,
			.id_mask	= 0x7FF,
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_rx_bms_precharge  = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_rx_bms_precharge,
	8,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};

can_msg_t msg_rx_bms_battvolt  = {
	{
		{
			.id			= 0x42A,
			.id_mask	= 0x7FE, //Get battvolt and battcurr message
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_rx_bms_battvolt  = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_rx_bms_battvolt,
	8,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};

can_msg_t msg_rx_bspd  = {
	{
		{
			.id			= CANR_FCN_DATA_ID | CANR_GRP_SENS_BSPD_ID | CANR_MODULE_ID0_ID,
			.id_mask	= 0x7FF,
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_rx_bspd  = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_rx_bspd,
	1,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};

can_msg_t msg_brk  = {
	{
		{
			.id			= CANR_FCN_DATA_ID | CANR_GRP_SENS_BRK_ID | CANR_MODULE_ID0_ID,
			.id_mask	= 0x7FE,
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_brk  = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_brk,
	8,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};

can_msg_t msg_slip_current  = {
	{
		{
			.id			= CANR_FCN_DATA_ID | CANR_GRP_ECU_ID | CANR_MODULE_ID7_ID,
			.id_mask	= 0x7F8,
		},
	},
	.data.u64 = 0x0LL,
};
can_mob_t mob_slip_current  = {
	CAN_MOB_NOT_ALLOCATED,
	&msg_slip_current,
	4,
	CAN_DATA_FRAME,
	CAN_STATUS_NOT_COMPLETED,
};