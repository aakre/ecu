/*
 * main.c
 *
 * Created: 17.02.2014 21:34:38
 *  Author: oyvinaak
 */ 

#include <asf.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "INVERTER_defines.h"
#include "queue_handles.h"
#include "ecu_can.h"
#include "revolve_pid.h"
#include "rtwtypes.h"
#include "fsm_ecu.h"
#include "mcp2515.h"
#include <stdbool.h>


#define WDT_TIMEOUT TASK_WATCHDOG_PERIOD_MS*10000 // us

/* Declare task functions */
static portTASK_FUNCTION_PROTO(task_watchdog, pvParameters);
static portTASK_FUNCTION_PROTO(task_main, pvParameters);
static portTASK_FUNCTION_PROTO(task_spi_can, pvParameters);

/* Create an array of flags that will be used by the Watchdog task to check
 * if the tasks are still alive */
static volatile signed portBASE_TYPE task_check_alive[NUMBER_OF_TASKS-1] = { (signed portBASE_TYPE) pdFALSE };
	
/* Create an array of task handles so that the Watchdog task can reference
 * tasks and kill/revive them if necessary */
static volatile xTaskHandle task_handles[NUMBER_OF_TASKS-1] = { (xTaskHandle) NULL };

/* Set up watchdog */
wdt_opt_t opt = {
	.dar   = false,     // After a watchdog reset, the WDT will still be enabled.
	.mode  = WDT_BASIC_MODE,    // The WDT is in basic mode, only PSEL time is used.
	.sfv   = false,     // WDT Control Register is not locked.
	.fcd   = false,     // The flash calibration will be redone after a watchdog reset.
	.cssel = WDT_CLOCK_SOURCE_SELECT_RCSYS,       // Select the system RC oscillator (RCSYS) as clock source.
	.us_timeout_period = WDT_TIMEOUT  // Timeout Value
};

/* Declare spi and state variables */
struct spi_device mcp2515_spiModule;
fsm_ecu_data_t ecu_data;
static volatile uint16_t data_timer	= 1;
static volatile uint8_t power_on_reset = 0;


void spi_init_pins(void);
struct spi_device spi_init_module(void);
void wdt_scheduler(void);
uint16_t get_and_send_periodic_data(fsm_ecu_data_t *ecu_data, uint16_t data_timer);


int main(void){	
	board_init();
	spi_init_pins();
	mcp2515_spiModule = spi_init_module();
	queue_from_inverter	= xQueueCreate(QUEUE_INVERTER_RX_LEN, sizeof(inverter_can_msg_t));
	queue_to_inverter	= xQueueCreate(QUEUE_INVERTER_RX_LEN+5, sizeof(inverter_can_msg_t));
	queue_wheel_fl		= xQueueCreate(1, sizeof(uint16_t));	
	queue_wheel_fr		= xQueueCreate(1, sizeof(uint16_t));	
	queue_wheel_rl		= xQueueCreate(1, sizeof(uint16_t));	
	queue_wheel_rr		= xQueueCreate(1, sizeof(uint16_t));
	//queue_traction_control = xQueueCreate(1, sizeof(int16_t));
	queue_dash_msg		= xQueueCreate(QUEUE_DASH_MSG_LEN, sizeof(dash_can_msg_t));	
	queue_trq_sens0		= xQueueCreate(1, sizeof(int16_t));
	queue_trq_sens1		= xQueueCreate(1, sizeof(int16_t));
	queue_bms_rx		= xQueueCreate(QUEUE_BMS_RX_LEN, sizeof(bms_can_msg_t));
	queue_brake_front	= xQueueCreate(2, sizeof(uint16_t));
	queue_brake_rear	= xQueueCreate(2, sizeof(uint16_t));
	queue_bspd			= xQueueCreate(1, sizeof(uint8_t));
	queue_trq_sens0_err = xQueueCreate(1, sizeof(uint8_t));
	queue_trq_sens1_err = xQueueCreate(1, sizeof(uint8_t));
	
	xTaskCreate(
		task_main
		, (signed portCHAR *) "Main ECU"
		, configMINIMAL_STACK_SIZE
		, (void *) &task_check_alive[0]
		, TASK_MAIN_PRIORITY
		, (xTaskHandle *) &task_handles[0]
	);
	
 	xTaskCreate(
 		task_spi_can
 		, (signed portCHAR *) "CAN3 SPI"
 		, configMINIMAL_STACK_SIZE
 		, (void *) &task_check_alive[1]
 		, TASK_SPI_CAN_PRIORITY
 		, (xTaskHandle *) &task_handles[1]
 	);
	 	 
	xTaskCreate(
	 	task_watchdog
	 	, (signed portCHAR *) "Watchdog"
	 	, configMINIMAL_STACK_SIZE
	 	, NULL
	 	, TASK_WATCHDOG_PRIORITY
	 	, NULL
	);
		
	#ifdef USE_WDT
		wdt_scheduler();
	#else
		fsm_ecu_init(&ecu_data);
	#endif
	vTaskStartScheduler();
}


static portTASK_FUNCTION(task_watchdog, pvParameters) {
	portTickType first_run = xTaskGetTickCount();
	short task;
	static uint8_t seppuku = 0;
	static uint8_t por_timer = 0;
	while (1) {
		vTaskDelayUntil(&first_run, TASK_WATCHDOG_PERIOD);
		/* Perform routine work.
		 * Check if tasks update their flags. If not, restart ECU by not
		 * clearing the watchdog timer.
		 */
		portENTER_CRITICAL();
		for (task = 0; task < NUMBER_OF_TASKS-1; task++) {
			if (task_check_alive[task] == pdFALSE) {
				seppuku++;
				gpio_set_pin_high(LED1);	
			}
			/* Task has well-behaved. Reset flag and wait for next round. */
			task_check_alive[task] = pdFALSE;
		}
		
		if (power_on_reset == 1) {
			por_timer++;
		} else {
			por_timer = 0;
		}
		
		if (por_timer < 10) {
			if (!seppuku) {
				/* Clear watchdog timer */
				wdt_clear();
			} else {
				asm("nop");
			}
		}
		portEXIT_CRITICAL();
		gpio_toggle_pin(LED2);
	}
}


static portTASK_FUNCTION( task_main, pvParameters ) {	
	volatile signed portBASE_TYPE *pxTaskHasExecuted = ( volatile signed portBASE_TYPE * ) pvParameters;
	ecu_can_inverter_disable_drive();
	portTickType first_run = xTaskGetTickCount();
	while(1) {
		vTaskDelayUntil(&first_run, TASK_MAIN_PERIOD);
		/* Run state machine */
		ecu_data.state = fsm_ecu_run_state(ecu_data.state, &ecu_data);
		data_timer = get_and_send_periodic_data(&ecu_data, data_timer);
	
		portENTER_CRITICAL();
		data_timer++;
		*pxTaskHasExecuted = pdTRUE;
		portEXIT_CRITICAL();	
		gpio_toggle_pin(LED4);
	}
}
	
static portTASK_FUNCTION(task_spi_can, pvParameters) {
	volatile signed portBASE_TYPE *pxTaskHasExecuted = ( volatile signed portBASE_TYPE * ) pvParameters;
	portTickType first_run = xTaskGetTickCount();
	inverter_can_msg_t inverter_can_msg;
	mcp2515_init (&mcp2515_spiModule);
	
	while(1) {
		vTaskDelayUntil(&first_run, TASK_SPI_CAN_PERIOD);
		if (gpio_pin_is_low(INT1)) { // Data has been received. Start reception of data
			//portENTER_CRITICAL(); // unsure if this is necessary?
			
			uint8_t canintfRegister;
			canintfRegister = mcp2515_readRegister(&mcp2515_spiModule,CANINTF); // read the interrupt register
			
			bool messageReceivedOnBuffer0 = canintfRegister & ( 1 << RX0IF);	// determine where messages have come from
			bool messageReceivedOnBuffer1 = canintfRegister & ( 1 << RX1IF);
			
			if ( messageReceivedOnBuffer0){
				gpio_toggle_pin(LED3);
				inverter_can_msg.data.u64 = 0x00L;
				inverter_can_msg.dlc = mcp2515_getReceivedMessage(&mcp2515_spiModule,0,inverter_can_msg.data.u8,6);
				xQueueSendToBack( queue_from_inverter, &inverter_can_msg, 0 );
				
			}
			
			if ( messageReceivedOnBuffer1){
				inverter_can_msg.data.u64 = 0x00L;
				inverter_can_msg.dlc = mcp2515_getReceivedMessage(&mcp2515_spiModule,1,inverter_can_msg.data.u8,6);
				xQueueSendToBack( queue_from_inverter, &inverter_can_msg, 0 );
				
			}
			
			//portEXIT_CRITICAL();
		}
		

		uint8_t TXBuffer0controlReg = mcp2515_readRegister(&mcp2515_spiModule, TXB0CTRL);	//check if transmit register 0 is ready to receive new data
		bool TXbuffer0Empty = !(TXBuffer0controlReg & (1 << TXREQ));
		uint8_t TXBuffer1controlReg = mcp2515_readRegister(&mcp2515_spiModule, TXB1CTRL); //check if transmit register 1 is ready to receive new data
		bool TXbuffer1Empty = !(TXBuffer1controlReg & ( 1 << TXREQ));
		
		bool messageSent = false;	
				
		if ( TXbuffer0Empty && !messageSent){
			inverter_can_msg.dlc = 0;
			inverter_can_msg.data.u64 = 0x00L;
			
			if ( xQueueReceive(queue_to_inverter, &inverter_can_msg,0) == pdTRUE){
				mcp2515_sendCanMessage(&mcp2515_spiModule,inverter_can_msg.dlc,inverter_can_msg.data.u8,INVERTER_ADDR_RX,0);
				messageSent = true;
			}
		}
		
		if ( TXbuffer1Empty && !messageSent){
			inverter_can_msg.dlc = 0;
			inverter_can_msg.data.u64 = 0x00L;
			
			if (xQueueReceive(queue_to_inverter, & inverter_can_msg,0) == pdTRUE){
				mcp2515_sendCanMessage(&mcp2515_spiModule,inverter_can_msg.dlc,inverter_can_msg.data.u8,INVERTER_ADDR_RX,1);
				messageSent = true;
			}
		}
		gpio_toggle_pin(LED3);
		portENTER_CRITICAL();
		*pxTaskHasExecuted = pdTRUE;
		portEXIT_CRITICAL();
	}
}


void wdt_scheduler(void) {
	// Watchdog reset
	if(AVR32_PM.RCAUSE.wdt) {
		power_on_reset = 0;
		load_state(&mcp2515_spiModule, &ecu_data);
		wdt_reenable();
	} else if (AVR32_PM.RCAUSE.por) {
		power_on_reset = 1;
		fsm_ecu_init(&ecu_data);
		wdt_enable(&opt);
	} else {
		power_on_reset = 0;
		fsm_ecu_init(&ecu_data);
		wdt_enable(&opt);
	}
}

uint16_t get_and_send_periodic_data(fsm_ecu_data_t *ecu_data, uint16_t data_timer) {
	if ((data_timer % TIMER_10_HZ) == 0) {
		ecu_can_inverter_read_reg(VDC_REG);
		ecu_can_inverter_read_reg(RPM_REG);
		ecu_can_send_fast_data(ecu_data->inverter_vdc, ecu_data->ecu_error, ecu_data->rpm, ecu_data->trq_cmd);
	}
	
	if ((data_timer % TIMER_2_HZ) == 0) {
		if (ecu_data->state == STATE_ERROR) {
			ecu_can_send_alive(1);
			} else {
			ecu_can_send_alive(0);
		}
	}
	
	if ((data_timer % TIMER_1_HZ) == 0) {
		ecu_can_inverter_read_reg(MOTOR_TEMP_REG);
		ecu_can_inverter_read_reg(IGBT_TEMP_REG);
		ecu_can_send_slow_data(ecu_data->motor_temp, ecu_data->inverter_temp, ecu_data->config_max_trq);
		save_state(&mcp2515_spiModule, ecu_data);
		data_timer = 0;
		asm("nop");
	}
	return data_timer;
}