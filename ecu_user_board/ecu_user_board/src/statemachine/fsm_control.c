/*
 * fsm.c
 *
 * Created: 02.03.2014 21:28:59
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
#include "fsm_control.h"

void traction_control(fsm_ecu_data_t* ecu_data) {
	static float delta_u	= 0.0F;
	static float e			= 0.0F;
	static float e_filter	= 0.0F;
	static float e_prev			= 0;
	static float e_filter_prev	= 0;
	static float e_filter_pprev	= 0;
	float torque_limit			= 0;
	
	if (ecu_data->slip_target > 0) {
		float Kp = ecu_data->Kp;
		float Ki = ecu_data->Ki;
		float Kd = ecu_data->Kd;
		float slip_target = ecu_data->slip_target;
		float d_filter_gain = ecu_data->d_filter_gain;
		
		//Calculate slip and error
		e = 100*(calculate_slip(ecu_data) - slip_target); //Slip is a percentage value
		
		//e_filter = (1-d_filter_gain)*e_filter_prev + d_filter_gain*e;
		//float d_action = Kd/Ts*(e_filter - 2*e_filter_prev + e_filter_pprev);
		
		delta_u = Kp*(e-e_prev) + Ki*Ts*e; //PI
		e_prev = e;
		//e_filter_pprev = e_filter_prev;
		//e_filter_prev  = e_filter;
		ecu_data->control_u += delta_u;
		ecu_data->control_u = max(50, min(100, ecu_data->control_u));
		
		torque_limit = 100 - ecu_data->control_u;
		ecu_data->traction_control_limit = (int16_t)(MAX_TORQUE*torque_limit)/100;
		asm("nop");
	} else {
		ecu_data->traction_control_limit = MAX_TORQUE;
	}
	
}

void p_controller(fsm_ecu_data_t *ecu_data) {
	float Kp = ecu_data->Kp;
	static float e = 0.0F;
	float slip_target = ecu_data->slip_target;
	//Calculate slip and error
	e = 100*(slip_target - calculate_slip(ecu_data)); //Slip is a percentage value
	if (ecu_data->launch_control_flag == LAUNCH_CONTROL_INACTIVE) {
		if (e > 0) { //For TC only care if slip > slip_target
			e = 0;
		}
	}
	ecu_data->control_u = Kp*e;
}

void launch_control(fsm_ecu_data_t *ecu_data) {
	static float filter_output = 0.0F;
	static uint8_t first_run = 1;
	float filter_gain = ecu_data->lc_filter_gain;
	int16_t trq_min = min(ecu_data->trq_sens0, ecu_data->trq_sens1);
	
	//Because statics cannot be initialized with variables...
	if (first_run) {
		first_run = 0;
		filter_output = (1-filter_gain)*ecu_data->lc_trq_init + filter_gain*trq_min*(float)MAX_TORQUE/1000.0; 
	} else {
		filter_output = (1-filter_gain)*filter_output + filter_gain*trq_min*(float)MAX_TORQUE/1000.0;	
	}
	
	//Add feedback from speed sensors to do slip control here (if needed)
	
	ecu_data->trq_cmd = filter_output;
	asm("nop");
}

float calculate_slip(fsm_ecu_data_t *ecu_data) {
	float slip;
	float v_f = ecu_data->WFL_sens & 0xFF;
	float v_r = ecu_data->WRR_sens & 0xFF; //WRL and WFR is not working
	
	//Actual speed is v*2.574 (average sensor),
	//but the constant will be mathematically cancelled when
	//calculating slip 
	
	if (v_r != 0) {
		slip = (v_r - v_f)/v_r;
	} else {
		slip = ecu_data->slip_target; //Feed zero error to traction controller
	}
	return slip;
}


