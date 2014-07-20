	/*
 * revolve_can_definitions.h
 *
 * Created: 20.01.2013 18:47:51
 *  Author: Bruker
 */ 


#ifndef REVOLVE_CAN_DEFINITIONS_H_
#define REVOLVE_CAN_DEFINITIONS_H_

//------------------------------------------------------------------------------
// EXAMPLE: function = command, group = dash, module ID = 0
/* .id = CANR_FCN_CMD_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID0 */
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/*	-------	ARBITRATION ID DEFINITIONS ---------------------------------------*/
//------------------------------------------------------------------------------

//- ID[10..8]
#define CANR_FCN_BOOT_ID				0x000		// Most significant
#define CANR_FCN_PRI_ID					0x200		// Priority data
#define CANR_FCN_CMD_ID					0x400		// Module_id control
#define CANR_FCN_DATA_ID				0x600		// Sensor data

//- ID[7..3]
#define CANR_GRP_SENS_TRQ_ID			0x10		// Torque encoder
#define CANR_GRP_SENS_BRK_ID			0x18		// Brake encoder
#define CANR_GRP_SENS_SPEED_ID			0x20		// Wheel speed
#define CANR_GRP_BMS_ID					0x28		// BMS-data
#define CANR_GRP_ECU_ID					0x30		// ECU
#define CANR_GRP_SENS_STEER_ID			0x38		// Steering angle
#define CANR_GRP_SUSP_ID				0x40		// Suspension
#define CANR_GRP_SENS_ACC_ID			0x48		// Acc/Gyro
#define CANR_GRP_SENS_DAMPER_ID			0x50		// Damper position
#define CANR_GRP_SENS_GEAR_TEMP_ID		0x56		// Gearbox temp
#define CANR_GRP_SENS_VOLT_ID			0x58		// Voltage meter
#define CANR_GRP_DASH_ID				0x60		// Dashboard
#define CANR_GRP_SENS_BSPD_ID	 		0x68		// BSPD
#define CANR_GRP_SENS_LAP_ID			0x70		// Laptimer
#define CANR_GRP_TELEMETRY_ID			0x78		// Telemetry
#define CANR_GRP_DATALOGGER_ID			0x80		// Datalogger
#define CANR_GRP_FAN_CTRL_ID			0x88		// Fan control
#define CANR_GRP_AUDIO_ID				0x90		// DAVE

//- ID[2..0]
#define CANR_MODULE_ID0_ID				0x0
#define CANR_MODULE_ID1_ID				0x1
#define CANR_MODULE_ID2_ID				0x2
#define CANR_MODULE_ID3_ID				0x3
#define CANR_MODULE_ID4_ID				0x4
#define CANR_MODULE_ID5_ID				0x5
#define CANR_MODULE_ID6_ID				0x6
#define CANR_MODULE_ID7_ID				0x7

//------------------------------------------------------------------------------
/*	-------	FCN_CMD_ID DEFINITIONS	------------------------------------------*/
//------------------------------------------------------------------------------


#define CANR_CMD_B				0
#define CANR_CMD_DLC_STD			1
#define CANR_CMD_DLC_LONG			8

#define CANR_CMD_RESET			0x01
#define CANR_CMD_BOOT			0x02
#define CANR_CMD_INFO_REQ			0x03
#define CANR_CMD_INFO_ANS			0x04
#define CANR_CMD_INFO_WRITE		0x05
#define CANR_CMD_LED_ON			0x06		// DEBUG CHECK, no need to implement
#define CANR_CMD_LED_OFF			0x07		// DEBUG CHECK, no need to implement
#define CANR_CMD_SET_STATE		0x08		// Set state, standby
#define CANR_CMD_ALIVE			0x09		// Alive signal, must be implemented. Defined in warnings Section Default to DASH ID

//- INFO bytes

#define CANR_INFO_ID_B				1
#define CANR_INFO_STATE_B			2
#define CANR_INFO_NAME0_B			3
#define CANR_INFO_NAME1_B			4
#define CANR_INFO_NAME2_B			5
#define CANR_INFO_NAME3_B			6
#define CANR_INFO_NAME4_B			7

#define CANR_INFO_NAME_NUM			4

//- SET_STATE bytes
#define CANR_CMD_SET_STATE_B		1
#define CANR_INFO_STATE_OPERATIVE	0x00
#define CANR_INFO_STATE_STDBY		0x01

//- ALIVE BYTES
#define CANR_ALIVE_MSG_DLC			3
#define CANR_ALIVE_MODULE_B			1
#define CANR_ALIVE_STATE_B			2

#define CANR_ALIVE_STATE_ERROR		0x00
#define CANR_ALIVE_STATE_OPERATIVE	0x01
#define CANR_ALIVE_STATE_STBY		0x02
#define CANR_ALIVE_STATE_FAULT		0x03
#define CANR_ALIVE_STATE_MANUAL		0x04


//------------------------------------------------------------------------------
//- Device specific command bytes
#define CANR_CMD_ADC_WRITE		0x20

// Dash
#define CANR_CMD_DASH_CONFIG_WRITE	0x20
#define CANR_CMD_DASH_CONFIG_READ	0x21
#define CANR_CMD_DASH_CONFIG_ANS	0x22


//------------------------------------------------------------------------------
/*	-------	FCN_DATA_ID DEFINITIONS	------------------------------------------*/
//------------------------------------------------------------------------------

//- ADC MODULES

#define CANR_ADC_INPUT_BYTE			0
#define CANR_ADC_DATA0_BYTE			1
#define CANR_ADC_DATA1_BYTE			2
#define CANR_ADC_DATA2_BYTE			3
#define CANR_ADC_DATA3_BYTE			4



//- DASH
#define DASH_STATEBIT 8
//Bit 0..7
#define SUS1_BIT						0  // BIT 0 and 1
#define SUS2_BIT						1
#define CLUTCHACTUATOR_BIT				2
#define STARTBUTTON_BIT				3
#define DATALOG_BIT					6
#define TRACTION_CONTROL_BIT			5
#define IGNITION_BIT					4
#define AUTOGEAR_BIT					7


//------------------------------------------------------------------------------
/*	-------	THRESHOLDS -------------------------------------------------------*/
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
/*	-------	DASH CODES -------------------------------------------------------*/
//------------------------------------------------------------------------------


// Used for CANR_CMD_DASH_WRITE and configurations
#define DASH_CONF_B					1
#define DASH_CONF_DATA_B				2
#define DASH_CONF_DATA_SIZE			2
#define DASH_CONF_EEPROM_OFFSET		0x10

#define	DASH_CONF_OIL_P_MIN			0
#define	DASH_CONF_OIL_P_MAX			1
#define DASH_CONF_OIL_T_MIN				2
#define DASH_CONF_OIL_T_MAX			3
#define DASH_CONF_ECT_MIN				4
#define DASH_CONF_ECT_MAX				5
#define DASH_CONF_FUEL_P_MIN			6
#define DASH_CONF_FUEL_P_MAX			7
#define DASH_CONF_FUEL_T_MIN			8
#define DASH_CONF_FUEL_T_MAX			9
#define DASH_CONF_PWM				10
#define DASH_CONF_SHIFTLIGHT			11
#define DASH_CONF_RPM_1_LO			12
#define DASH_CONF_RPM_1_MID			13
#define DASH_CONF_RPM_1_HIGH			14
#define DASH_CONF_RPM_2_LO			15
#define DASH_CONF_RPM_2_MID			16
#define DASH_CONF_RPM_2_HIGH			17
#define DASH_CONF_RPM_3_LO			18
#define DASH_CONF_RPM_3_MID			19
#define DASH_CONF_RPM_3_HIGH			20			
#define DASH_CONF_RPM_4_LO			21
#define DASH_CONF_RPM_4_MID			22
#define DASH_CONF_RPM_4_HIGH			23
#define DASH_CONF_RPM_5_LO			24
#define DASH_CONF_RPM_5_MID			25
#define DASH_CONF_RPM_5_HIGH			26
#define DASH_CONF_RPM_6_LO			27
#define DASH_CONF_RPM_6_MID			28
#define DASH_CONF_RPM_6_HIGH			29
#define DASH_CONF_WARNINGS			30

#define	DASH_CONF_LENGTH			31			// Length of dash conf array.

#define DASH_CONF_SHIFTLIGHT_AUTO		0
#define DASH_CONF_SHIFTLIGHT_OFF			1
#define DASH_CONF_SHIFTLIGHT_ON			2

// Alive register

#define DASH_ALIVE_SIZE					12

#define DASH_ALIVE_ECU					0
#define DASH_ALIVE_AUX					1
#define DASH_ALIVE_IO					2
#define DASH_ALIVE_GEARPOS				3
#define DASH_ALIVE_GEAR				4

#define DASH_ALIVE_STWHEEL				6
#define DASH_ALIVE_DASH				7
#define DASH_ALIVE_TELEMETRY			8
#define DASH_ALIVE_DATALOGGER			9
#define DASH_ALIVE_VARI				10
#define DASH_ALIVE_SUS					11

//------------------------------------------------------------------------------
/*	-------	ALIVE	 ---------------------------------------------------------*/
//------------------------------------------------------------------------------

// ---------------------------------------------------------------------//
// Functionality description:											//
// Each module sends out a message with the following properties:		//
// ID = CANR_FCN_CMD_ID | CANR_GRP_DASH_ID								//
// DATA[0] = CANR_CMD_ALIVE												//
// DATA[1] = MODULE ID													//
// DATA[2] = MODULE STATE												//
// Frequency = 1 Hz														//
// ---------------------------------------------------------------------//

//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
/*	-------	EEPROM CONFIG ----------------------------------------------------*/
//------------------------------------------------------------------------------

#define EEPROM_BOOT_ADDR				0x00
#define EEPROM_INFO_ADDR				0x10
#define EEPROM_CONF_ADDR				0x30

//- Boot values
#define EEPROM_BOOT_NO				0x00
#define	EEPROM_BOOT_YES			0xF0
#define EEPROM_BOOT_UNWRITTEN	0xFF

#endif /* REVOLVE_CAN_DEFINITIONS_H_ */

