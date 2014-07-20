/*
 * queue_handles.h
 *
 * Created: 20.02.2014 22:05:03
 *  Author: oyvinaak
 */ 


#ifndef QUEUE_HANDLES_H_
#define QUEUE_HANDLES_H_

#include "FreeRTOS.h"
#include "queue.h"

#define QUEUE_INVERTER_RX_LEN	5
#define QUEUE_DASH_MSG_LEN		5
#define QUEUE_BMS_RX_LEN		5

/* Queues for task communication */
xQueueHandle queue_from_inverter;
xQueueHandle queue_to_inverter;
xQueueHandle queue_wheel_fl;
xQueueHandle queue_wheel_fr;
xQueueHandle queue_wheel_rl;
xQueueHandle queue_wheel_rr;
xQueueHandle queue_traction_control;
xQueueHandle queue_dash_msg;
xQueueHandle queue_trq_sens0;
xQueueHandle queue_trq_sens1;
xQueueHandle queue_bms_rx;
xQueueHandle queue_brake_front;
xQueueHandle queue_brake_rear;
xQueueHandle queue_bspd;
xQueueHandle queue_trq_sens0_err;
xQueueHandle queue_trq_sens1_err;



#endif /* QUEUE_HANDLES_H_ */