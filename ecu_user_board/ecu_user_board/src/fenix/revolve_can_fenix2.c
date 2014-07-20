/*
 * revolve_can_fenix2.c
 *
 * Created: 01.06.2014 18:04:51
 *  Author: Simen
 */

#include <asf.h>

void fenix_master(){
	gpio_configure_pin(CAN0_TX_PIN, GPIO_DIR_OUTPUT|GPIO_INIT_LOW);
	gpio_configure_pin(CAN0_RX_PIN, GPIO_DIR_INPUT|GPIO_INIT_HIGH);
	
	gpio_configure_pin(CAN1_TX_PIN, GPIO_DIR_OUTPUT|GPIO_INIT_LOW);
	gpio_configure_pin(CAN1_RX_PIN, GPIO_DIR_INPUT|GPIO_INIT_HIGH);
	
	delay_ms(1000);
	
	gpio_set_pin_high(CAN0_TX_PIN);
	gpio_set_pin_high(CAN1_TX_PIN);
	
	/*delay_ms(10);
	
	gpio_set_pin_high(CAN0_TX_PIN);
	gpio_set_pin_high(CAN1_TX_PIN);*/
}

void fenix_slave(){
	delay_ms(300);
	gpio_configure_pin(CAN0_TX_PIN, GPIO_DIR_OUTPUT|GPIO_INIT_HIGH);
	gpio_configure_pin(CAN0_RX_PIN, GPIO_DIR_INPUT|GPIO_INIT_LOW);
	
	gpio_configure_pin(CAN1_TX_PIN, GPIO_DIR_OUTPUT|GPIO_INIT_HIGH);
	gpio_configure_pin(CAN1_RX_PIN, GPIO_DIR_INPUT|GPIO_INIT_LOW);
	
	while(gpio_pin_is_low(CAN0_RX_PIN) && gpio_pin_is_low(CAN1_RX_PIN));
	while(gpio_pin_is_high(CAN0_RX_PIN) && gpio_pin_is_high(CAN1_RX_PIN));
	
}