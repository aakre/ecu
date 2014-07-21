/*
 * fsm_ecu.h
 *
 * Created: 26.03.2014 19:20:41
 *  Author: oyvinaak
 */ 


#ifndef FSM_ECU_H_
#define FSM_ECU_H_

#include "ecu_can.h"

#define TRQ_MISSED_LIMIT 2

/* ECU error register			BIT	 Explanation */
#define ERR_BMS_COM				0 // Lost communicating with BMS
#define ERR_INVERTER_VDC_LOW	1 // Inverter side voltage not >90 % of battery pack voltage
#define ERR_INVERTER_COM		2 // Lost communication with inverter
#define ERR_INVERTER_INTERNAL	3 // Internal error in inverter
#define ERR_TRQ_SENSORS			4 // Lost communication with torque sensors
#define ERR_SPEED_SENSORS		5 // Lost communication with speed sensors
#define ERR_FRG					6 // FRG is low
#define ERR_AIR_PLUS			7 // AIR PLUS is low
#define ERR_BSPD				8 // BSPD signal loss
#define ERR_BRAKE_SENS_FRONT	9 // Lost communication with front brake sensor
#define ERR_BRAKE_SENS_REAR		10 // Lost communication with rear brake sensor

// PID parameters
#define Kp_default				100.0F	// Proportional gain
#define Ki_default				0.0F	// Integral gain (Ki = 1/Ti)
#define Kd_default				30.0F	// Derivation gain
#define N_filter				100.0F	// -> Derivate filter time constant Tf = Kd / N, (Kd = Td)

// Filter parameters
#define Ts						0.02F // System sampling time 50 Hz
#define SLIP_TARGET_DEFAULT		0.2F
#define D_FILTER_GAIN_DEFAULT	(N_filter*Ts/(Kd_default+N_filter*Ts))
#define PEDAL_FILTER_T			0 // Safe to choose 0.02F
#define PEDAL_FILTER_GAIN		Ts/(PEDAL_FILTER_T + Ts)
#define LC_FILTER_T_DEFAULT		1.5F // Launch control rise time
#define LC_FILTER_GAIN_DEFAULT	Ts/(LC_FILTER_T_DEFAULT + Ts); // Low pass filter gain
#define LC_TRQ_INIT_DEFAULT		(float)30*MAX_TORQUE/100.0				


typedef enum fsm_ecu_state{
	STATE_STARTUP,
	STATE_CHARGED,
	STATE_ENABLE_DRIVE,
	STATE_READY,
	STATE_INIT_LAUNCH,
	STATE_LAUNCH_CONTROL,
	STATE_DEACTIVATE_LAUNCH,
	STATE_PLAUSIBILITY_ERROR,
	STATE_ERROR,
	FSM_ECU_NUM_STATES,
} fsm_ecu_state_t;

typedef enum flag_drive_enable{
	DRIVE_DISABLED,
	DRIVE_ENABLE_REQUEST,
	DRIVE_ENABLED,
	DRIVE_ENABLE_RTDS_PLAYS,
	DRIVE_DISABLE_REQUEST,
} flag_drive_enable_t;

typedef enum arctos_mode{
	ARCTOS_MODE_NORMAL,
	ARCTOS_MODE_TRACTION_CONTROL,
	ARCTOS_MODE_INDOORS,
} arctos_mode_t;

typedef enum launch_control_type {
	LAUNCH_CONTROL_INACTIVE,
	LAUNCH_CONTROL_INITIATE,
	LAUNCH_CONTROL_COUNTDOWN_COMPLETE,
	LAUNCH_CONTROL_ACTIVE,
} launch_control_t;



typedef struct fsm_ecu_data{
	fsm_ecu_state_t state;
	inverter_can_msg_t inverter_can_msg;
	int16_t trq_sens0;
	int16_t trq_sens1;
	uint8_t trq_sens0_err;
	uint8_t trq_sens1_err;
	int16_t trq_cmd;
	float trq_pid;
	float trq_pedal;
	dash_can_msg_t dash_msg;
	bms_can_msg_t bms_msg;
	uint16_t vdc_battery;
	uint16_t inverter_vdc;
	uint16_t rpm;
	uint16_t motor_temp;
	uint16_t inverter_temp;
	uint16_t brake_front;
	uint16_t brake_rear;
	uint8_t flag_start_precharge;
	uint8_t flag_brake_implausible;
	int8_t max_cell_temp;
	flag_drive_enable_t flag_drive_enable;
	arctos_mode_t arctos_mode;
	uint16_t inverter_error;
	uint16_t ecu_error;
	uint16_t WFL_sens;
	uint16_t WFR_sens;
	uint16_t WRL_sens;
	uint16_t WRR_sens;
	launch_control_t launch_control_flag;
	uint8_t reboot;
	uint8_t config_max_trq;
	float Kp;
	float Ki;
	float Kd;
	float slip_target;
	float d_filter_gain;
	uint8_t inverter_timeout; //Not used (practically)
	float lc_filter_gain;
	float lc_trq_init;
	int16_t kers_factor;
	int16_t bms_current;
}fsm_ecu_data_t;

void fsm_ecu_init(fsm_ecu_data_t *data);

typedef fsm_ecu_state_t fsm_ecu_state_func_t( fsm_ecu_data_t *data );

fsm_ecu_state_t fsm_ecu_run_state( fsm_ecu_state_t current_state, fsm_ecu_data_t *data);
fsm_ecu_state_t fsm_ecu_state_startup_func( fsm_ecu_data_t *data );
fsm_ecu_state_t fsm_ecu_state_charged_func( fsm_ecu_data_t *data );
fsm_ecu_state_t fsm_ecu_state_enable_drive_func( fsm_ecu_data_t *data );
fsm_ecu_state_t fsm_ecu_state_ready_func( fsm_ecu_data_t *data );
fsm_ecu_state_t fsm_ecu_state_init_launch_func( fsm_ecu_data_t *data );
fsm_ecu_state_t fsm_ecu_state_launch_control_func( fsm_ecu_data_t *data );
fsm_ecu_state_t fsm_ecu_state_deactivate_launch_func( fsm_ecu_data_t *data );
fsm_ecu_state_t fsm_ecu_state_plausibility_error_func( fsm_ecu_data_t *data );
fsm_ecu_state_t fsm_ecu_state_error_func( fsm_ecu_data_t *data );


#endif /* FSM_ECU_H_ */