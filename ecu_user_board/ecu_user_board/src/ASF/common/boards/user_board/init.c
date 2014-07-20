/**
 * \file
 *
 * \brief User board initialization template
 *
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>
#include "ecu_can.h"
#include "mcp2515.h"

void board_init(void)
{
	/* This function is meant to contain board-specific initialization code
	 * for, e.g., the I/O pins. The initialization can rely on application-
	 * specific board configuration, found in conf_board.h.
	 */
	#ifdef USE_WDT
		wdt_disable();
	#endif
	/* Disable all interrupts. */
	Disable_global_interrupt();
	sysclk_init();
	delay_init(sysclk_get_cpu_hz());
	
	gpio_configure_pin(LED1,  GPIO_INIT_HIGH|GPIO_DIR_OUTPUT);
	gpio_configure_pin(LED2,  GPIO_INIT_HIGH|GPIO_DIR_OUTPUT);
	gpio_configure_pin(LED3,  GPIO_INIT_HIGH|GPIO_DIR_OUTPUT);
	gpio_configure_pin(LED4,  GPIO_INIT_HIGH|GPIO_DIR_OUTPUT);
	gpio_configure_pin(AIR_PLUS,GPIO_INIT_LOW|GPIO_DIR_OUTPUT);
	gpio_configure_pin(FRG_PIN,GPIO_INIT_LOW|GPIO_DIR_OUTPUT);
	gpio_configure_pin(RFE_PIN,GPIO_INIT_LOW|GPIO_DIR_OUTPUT);
	gpio_configure_pin(INVERTER_BTB,GPIO_PULL_DOWN|GPIO_DIR_INPUT);
	
	
	gpio_configure_pin(INVERTER_DIN1, GPIO_INIT_LOW|GPIO_DIR_OUTPUT);
	gpio_configure_pin(INVERTER_DIN2, GPIO_INIT_LOW|GPIO_DIR_OUTPUT);
	gpio_configure_pin(INVERTER_DOUT1,GPIO_PULL_DOWN|GPIO_DIR_INPUT);
	gpio_configure_pin(INVERTER_DOUT2,GPIO_PULL_DOWN|GPIO_DIR_INPUT);
	gpio_configure_pin(INVERTER_DOUT3,GPIO_PULL_DOWN|GPIO_DIR_INPUT);
	gpio_configure_pin(END1, GPIO_INIT_LOW|GPIO_DIR_OUTPUT);
	gpio_configure_pin(END2, GPIO_INIT_LOW|GPIO_DIR_OUTPUT);
	
	gpio_configure_pin(INT1, GPIO_DIR_INPUT| GPIO_PULL_UP); 
	
	ecu_can_init();
	
	Enable_global_interrupt();
	
}
