/*
 * ecu_can.h
 *
 * Created: 20.02.2014 13:01:15
 *  Author: oyvinaak
 */ 


#ifndef ECU_CAN_H_
#define ECU_CAN_H_

#include "revolve_can_definitions.h"
#include "ecu_can_messages.h"

#define CAN_BUS_0	0
#define CAN_BUS_1	1

#define CANR_DASH_ID_TX	CANR_FCN_CMD_ID  | CANR_GRP_DASH_ID | CANR_MODULE_ID0_ID
#define DASH_MSG0		CANR_FCN_CMD_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID0_ID
#define DASH_MSG1		CANR_FCN_CMD_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID1_ID
#define DASH_MSG2		CANR_FCN_CMD_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID2_ID

typedef struct inverter_can_msg{
	Union64 data;
	uint32_t dlc;
}inverter_can_msg_t;

typedef struct bms_can_msg{
	Union64 data;
	uint32_t id;
}bms_can_msg_t;

typedef struct dash_can_msg{
	Union64 data;
	uint16_t id;
}dash_can_msg_t;

void ecu_can_init(void);
#endif /* ECU_CAN_H_ */