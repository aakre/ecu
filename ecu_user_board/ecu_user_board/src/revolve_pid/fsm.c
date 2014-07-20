/*
 * fsm.c
 *
 * Created: 02.03.2014 21:28:59
 *  Author: oyvinaak
 */ 

#include <asf.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "queue_handles.h"
#include "revolve_pid.h"
#include "rtwtypes.h"
#include "fsm.h"

state_func_t *const state_table[ NUM_STATES ] = {
	state_inactive_func, 
	state_traction_control_func,
	state_launch_control_func,
};

state_t run_state( state_t current_state, state_data_t *data) {
	return state_table[ current_state ]( data );
}

state_t state_inactive_func( state_data_t *data ) {
	state_t new_state;
	state_t next_state = STATE_INACTIVE;
	data->torque_correction = 10;
	if ( xQueueReceive(queue_ecu_pid_comm, &new_state, 10) == pdTRUE ) {
		next_state = new_state;
	}
	return next_state;
}

state_t state_traction_control_func( state_data_t *data ) {
	static portBASE_TYPE queue_receive_ok = pdFALSE;
	static boolean_T OverrunFlag = 0;
	
	/* Get wheel speed measurements*/
	queue_receive_ok = xQueueReceive(queue_wheel_fl, &data->wFrontLeft, 0);
	if (queue_receive_ok == pdFALSE) {
		data->wFrontLeft = 0;
	}
	queue_receive_ok = xQueueReceive(queue_wheel_fr, &data->wFrontRight, 0);
	if (queue_receive_ok == pdFALSE) {
		data->wFrontRight = 0;
	}
	queue_receive_ok = xQueueReceive(queue_wheel_rl, &data->wRearLeft, 0);
	if (queue_receive_ok == pdFALSE) {
		data->wRearLeft = 0;
	}
	queue_receive_ok = xQueueReceive(queue_wheel_rr, &data->wRearRight, 0);
	if (queue_receive_ok == pdFALSE) {
		data->wRearRight = 0;
	}
	
	/** Start PID **/
	/* Check for overrun */
	if (OverrunFlag) {
		rtmSetErrorStatus(data->pid_M, "Overrun");
		return STATE_INACTIVE;
	}
	OverrunFlag = TRUE;
	
	/* Set inputs */
	data->pid_U.wFrontLeft	= data->wFrontLeft;
	data->pid_U.wFrontRight	= data->wFrontRight;
	data->pid_U.wRearLeft	= data->wRearLeft;
	data->pid_U.wRearRight	= data->wRearRight;
	data->pid_U.slip_target	= data->slip_target;
	data->pid_U.pid_es		= data->external_reset;
	data->pid_P.pid_Kp_Gain   = data->Kp_gain;
	data->pid_P.wFront_Gain_Gain = 10*data->Kp_gain;
	data->pid_P.wRear_Gain_Gain  = data->Kp_gain;
	
	/* Step model */
	revolve_pid_step();
	
	/* Collect output */
	OverrunFlag = FALSE;
	data->torque_correction = data->wFrontLeft;//(int16_t) data->pid_Y.pid_out;
	
	return STATE_TC;
}

state_t state_launch_control_func( state_data_t *data ) {
	/* Do LC sequence
	 * and end up in TC */
	data->torque_correction = 0x3713;
	return STATE_LC;
}