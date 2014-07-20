/*
 * fsm.h
 *
 * Created: 02.03.2014 21:59:25
 *  Author: oyvinaak
 */ 


#ifndef FSM_CONTROL_H_
#define FSM_CONTROL_H_

void traction_control(fsm_ecu_data_t* ecu_data);
void p_controller(fsm_ecu_data_t *ecu_data);
void launch_control(fsm_ecu_data_t *ecu_data);
float calculate_slip(fsm_ecu_data_t *ecu_data);

#endif /* FSM_H_ */