/*
 * fsm_ecu_functions.h
 *
 * Created: 22.05.2014 17:26:21
 *  Author: oyvinaak
 */ 


#ifndef FSM_ECU_FUNCTIONS_H_
#define FSM_ECU_FUNCTIONS_H_

bool brake_over_travel_check (fsm_ecu_data_t* ecu_data);
bool brake_plausibility_check(fsm_ecu_data_t *ecu_data);
bool torque_plausibility_check(fsm_ecu_data_t *ecu_data);
int16_t calc_kers(fsm_ecu_data_t *ecu_data);
uint16_t calc_bms_power(fsm_ecu_data_t *ecu_data);
uint16_t calc_inverter_power(fsm_ecu_data_t *ecu_data);
uint16_t calc_max_current_allowed(fsm_ecu_data_t* ecu_data);
uint8_t check_bspd(void);
uint8_t check_inverter_error(fsm_ecu_data_t *ecu_data);
uint8_t check_inverter_timeout(fsm_ecu_data_t *ecu_data);
uint8_t get_brake_sens(fsm_ecu_data_t *ecu_data);
uint8_t get_speed_sens(fsm_ecu_data_t *ecu_data);
uint8_t get_trq_sens(fsm_ecu_data_t *ecu_data);
uint16_t convert_num_to_vdc(uint32_t num);
uint16_t convert_to_big_endian(uint32_t data);
void ecu_dio_inverter_clear_error(void);
void get_new_data(fsm_ecu_data_t *ecu_data);
void handle_bms_data(fsm_ecu_data_t *ecu_data);
void handle_dash_data(fsm_ecu_data_t *ecu_data);
void handle_inverter_data(fsm_ecu_data_t *ecu_data);
void map_pedal(fsm_ecu_data_t *ecu_data);








#endif /* FSM_ECU_FUNCTIONS_H_ */