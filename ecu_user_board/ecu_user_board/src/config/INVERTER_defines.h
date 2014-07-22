/*
 * INVERTER_defines.h
 *
 * Created: 17.02.2014 19:47:03
 *  Author: oyvinaak
 */ 


#ifndef INVERTER_DEFINES_H_
#define INVERTER_DEFINES_H_

/* Address for sending to inverter */
#define INVERTER_ADDR_RX		0x100
/* Address for reading from inverter */
#define INVERTER_ADDR_TX		0x120
#define INVERTER_ADDR_MASK		0x7FF // 0 = don't care

/* Define state bits in REG 0x40 */
#define FRG_BIT						0
#define RDY_BIT						14
#define BRK_BIT						15

/* Bits in error reg */
#define PWR_FAULT					1
#define	RFE_FAULT					2
#define RESOLVER_FAULT				3

#define CURRENT_REG					0x20
#define MODE_REG					0x51
#define BTB_REG						0xE2
#define FRG_REG						0xE8
#define VDC_REG						0xEB
#define MOTOR_TEMP_REG				0x49
#define IGBT_TEMP_REG				0x4A
#define RPM_REG						0x30
#define ERROR_REG					0x8F

/* Registers in inverter */
#define TORQUE_CMD					0x90
#define READ_CMD					0x3D
#define SPEED_CMD					0x31

/* DLCs */
#define INVERTER_DLC_3				3
#define INVERTER_DLC_5				5

#define MAX_TORQUE					(int16_t)0x7FFF
#define MAX_RPM						5000											

#endif /* INVERTER_DEFINES_H_ */