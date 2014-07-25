/*
 * ecu_can_messages.h
 *
 * Created: 25.05.2014 16:37:44
 *  Author: oyvinaak
 */ 


#ifndef ECU_CAN_MESSAGES_H_
#define ECU_CAN_MESSAGES_H_

void ecu_can_send_to_inverter(uint8_t inverter_reg, uint16_t data);
void ecu_can_inverter_torque_cmd(int16_t torque);
void ecu_can_blast(uint16_t data);
void ecu_can_inverter_read_reg(uint8_t inverter_reg);
void ecu_can_send_to_dash(uint16_t data);
void ecu_can_send_tractive_system_active(void);
void ecu_can_blast32(uint32_t data);
void ecu_can_inverter_enable_drive(void);
void ecu_can_inverter_disable_drive(void);
void ecu_can_send_current_bms(uint16_t current);
void ecu_can_send_slow_data(uint16_t motor_temp, uint16_t inverter_temp, uint8_t max_trq);
void ecu_can_send_fast_data(uint16_t inverter_vdc, uint16_t ecu_error, uint16_t rpm, int16_t trq_cmd);
void ecu_can_send_ready_to_drive(void);
void ecu_can_speed_command(int16_t rpm_cmd);
void ecu_can_send_play_rtds(void);
void ecu_can_send_alive(uint8_t error);
void ecu_can_send_drive_disabled(void);
void ecu_can_confirm_activate_launch(void);
void ecu_can_send_launch_ready(void);
void ecu_can_send_launch_stop(void);
void ecu_can_send_slip_current(int16_t val1, int16_t val2);
void ecu_can_inverter_read_torque_periodic(void);


void fake_bms_can(void);


#endif /* ECU_CAN_MESSAGES_H_ */