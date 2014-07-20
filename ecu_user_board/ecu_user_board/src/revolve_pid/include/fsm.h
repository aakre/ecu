/*
 * fsm.h
 *
 * Created: 02.03.2014 21:59:25
 *  Author: oyvinaak
 */ 


#ifndef FSM_H_
#define FSM_H_

typedef enum { 
	STATE_INACTIVE, 
	STATE_TC, 
	STATE_LC, 
	NUM_STATES,
} state_t;

typedef struct state_data {
	/* Measurement */
	uint16_t wFrontLeft;
	uint16_t wFrontRight;
	uint16_t wRearLeft;
	uint16_t wRearRight;
	int16_t torque_correction;
	
	/* PID parameters */
	float slip_target;
	float external_reset;
	float Kp_gain;
	float Ki_gain;
	float Kd_gain;
	
	/* PID model objects*/
	RT_MODEL_revolve_pid *pid_M;
	ExternalInputs_revolve_pid pid_U;
	ExternalOutputs_revolve_pid pid_Y;
	Parameters_revolve_pid pid_P; 
} state_data_t;

typedef state_t state_func_t( state_data_t *data );

state_t run_state( state_t current_state, state_data_t *data);
state_t state_inactive_func( state_data_t *data );
state_t state_traction_control_func( state_data_t *data );
state_t state_launch_control_func( state_data_t *data );

#endif /* FSM_H_ */