/*
 * mcp2515.h
 *
 * Inspired by this library: https://github.com/fazjaxton/CAN
 *
 * Created: 05.04.2014 17:42:28
 *  Author: Kristian Windows
 */ 


#ifndef MCP2515_H_
#define MCP2515_H_
#include <asf.h>
#include "fsm_ecu.h"

#define MCP2515_SPI_ADDRESS				&AVR32_SPI0
#define MCP2515_CAN_CTRL_REG_ADDR		0x0F
#define MCP2515_CAN_STAT_REG_ADDR		0x0E
#define MCP2515_MODE_bm					( 0b111 << 5 )



#define SJW_LENGTH_1XTQ_bm	(	0x0 << 6	)
#define SJW_LENGTH_2XTQ_bm	(	0x1 << 6	)
#define SJW_LENGTH_3XTQ_bm	(	0x2 << 6	)
#define SJW_LENGTH_4XTQ_bm	(	0x3	<< 6	)

#define CAN_BAUD_RATE_PRESCALER 0

#define PRSEG_2_bm				(	0x2 << 0	)			// propagation segment length value (p. 44 of datasheet )
#define PHSEG1_1_bm				(	0x1 << 3	)			// length of PS1 (length is 1+ PHSEG1_1_bm)
#define BTLMODE_1_bm			(	0x1 << 7	)			// length of PS2 determined by PHSEG22:PHSEG20 bits of CNF3

#define PHSEG2_1_bm				(	0x1 << 0	)			// length of PS2 (length is 1+ PHSEG2_1_bm)

#define RX0IE_bm				( 1 << 0 )	// Enable Buffer 0 Full Interrupt Enable
#define RX1IE_bm				( 1 << 1 )	// Enable buffer 1 Full Interrupt Enable

#define RX_MASK_FILTER_OFF_bm	( 0b11 << 5 )

#define BUFFER0					0
#define BUFFER1					1
#define BUFFER_1_AND_2			2


#define RX_DLC_BITS_bm			( 0b1111 << 0 )


#define MCP2515_CAN_TRANSMIT_REQUEST_bp				3
#define MCP2515_CAN_TRANSMIT_REQUEST_bm				( 1 << 3)
#define MCP2515_CAN_MESSAGE_PRIORITY_LOWEST_bm		0x0
#define MCP2515_CAN_MESSAGE_PRIORITY_LOW_bm			0x1
#define MCP2515_CAN_MESSAGE_PRIORITY_HIGH_bm		0x2
#define MCP2515_CAN_MESSAGE_PRIORITY_HIGHEST_bm		0x3

#define RX0IF	0
#define RX1IF	1

#define	TXREQ	3




/* :::::::::::::::::Register adresses MCP2515::::::::::::::: */

//		---- TX ----

#define TXRTSCTRL			0x0D	//TXnRTS Pin Control and Status Register

#define TXB0CTRL			0x30	// Transmit Buffer 0 Control Register
#define TXB0SIDH			0x31	// Transmit Buffer 0 Standard Identifier High Address
#define TXB0SIDL			0x32	// Transmit Buffer 0 Standard Identifier Low Address
#define TXB0DLC				0x35	// Transmit Buffer 0 Data Length code


#define TXB0D0				0x36	// Transmit Buffer 0 Data Byte 0
#define TXB0D1				0x37	// Transmit Buffer 0 Data Byte 1
#define TXB0D2				0x38	// Transmit Buffer 0 Data Byte 2
#define TXB0D3				0x39	// Transmit Buffer 0 Data Byte 3
#define TXB0D4				0x3A	// Transmit Buffer 0 Data Byte 4
#define TXB0D5				0x3B	// Transmit Buffer 0 Data Byte 5
#define TXB0D6				0x3C	// Transmit Buffer 0 Data Byte 6
#define TXB0D7				0x3D	// Transmit Buffer 0 Data Byte 7


#define TXB1CTRL			0x40	// Transmit Buffer 1 Control Register
#define TXB1SIDH			0x41	// Transmit Buffer 1 Standard Identifier High Address
#define TXB1SIDL			0x42	// Transmit Buffer 1 Standard Identifier Low Address
#define TXB1DLC				0x45	// Transmit Buffer 1 Data Length code

#define TXB1D0				0x46	// Transmit Buffer 1 Data Byte 0
#define TXB1D1				0x47	// Transmit Buffer 1 Data Byte 1
#define TXB1D2				0x48	// Transmit Buffer 1 Data Byte 2
#define TXB1D3				0x49	// Transmit Buffer 1 Data Byte 3
#define TXB1D4				0x4A	// Transmit Buffer 1 Data Byte 4
#define TXB1D5				0x4B	// Transmit Buffer 1 Data Byte 5
#define TXB1D6				0x4C	// Transmit Buffer 1 Data Byte 6
#define TXB1D7				0x4D	// Transmit Buffer 1 Data Byte 7



#define TXB2CTRL			0x50	// Transmit Buffer 1 Control Register
#define	TXB2SIDH			0x51	// Transmit Buffer 2 Standard Identifier High Address
#define TXB2SIDL			0x42	// Transmit Buffer 2 Standard Identifier Low Address
#define TXB2DLC				0x55	// Transmit Buffer 2 Data Length code

#define TXB2D0				0x56	// Transmit Buffer 2 Data Byte 0
#define TXB2D1				0x57	// Transmit Buffer 2 Data Byte 1
#define TXB2D2				0x58	// Transmit Buffer 2 Data Byte 2
#define TXB2D3				0x59	// Transmit Buffer 2 Data Byte 3
#define TXB2D4				0x5A	// Transmit Buffer 2 Data Byte 4
#define TXB2D5				0x5B	// Transmit Buffer 2 Data Byte 5
#define TXB2D6				0x5C	// Transmit Buffer 2 Data Byte 6
#define TXB2D7				0x5D	// Transmit Buffer 2 Data Byte 7

#define CNF1				0x2A	// Timing Configuration Register 1
#define CNF2				0x29	// Timing Configuration Register 2
#define CNF3				0x28	// Timing Configuration Register 3	


//		---- RX ----

#define BFPCTRL				0x0C	// Pin Control And Status


#define RXB0CTRL			0x60	// Receive Buffer 0 Control
#define RXB0SIDH			0x61	// Receive Buffer 0 Standard Identifier High
#define RXB0SIDL			0x62	// Receive Buffer 0 Standard Identifier Low
#define RXB0DLC				0x65	// Receive Buffer 0 Data Length Code
#define RXB0D0				0x66	// Receive Buffer 0 Data Byte 0
#define RXB0D1				0x67	// Receive Buffer 0 Data Byte 1
#define RXB0D2				0x68	// Receive Buffer 0 Data Byte 2
#define RXB0D3				0x69	// Receive Buffer 0 Data Byte 3
#define RXB0D4				0x6A	// Receive Buffer 0 Data Byte 4
#define RXB0D5				0x6B	// Receive Buffer 0 Data Byte 5
#define RXB0D6				0x6C	// Receive Buffer 0 Data Byte 6
#define RXB0D7				0x6D	// Receive Buffer 0 Data Byte 7




#define RXB1CTRL			0x70	// Receive Buffer 1 Control
#define RXB1SIDH			0x71	// Receive Buffer 1 Standard Identifier High		
#define RXB1SIDL			0x72	// Receive Buffer 1 Standard Identifier Low
#define RXB1DLC				0x75	// Receive Buffer 1 Data Length Code
#define RXB1D0				0x76	// Receive Buffer 1 Data Byte 0
#define RXB1D1				0x77	// Receive Buffer 1 Data Byte 1
#define RXB1D2				0x78	// Receive Buffer 1 Data Byte 2
#define RXB1D3				0x79	// Receive Buffer 1 Data Byte 3
#define RXB1D4				0x7A	// Receive Buffer 1 Data Byte 4
#define RXB1D5				0x7B	// Receive Buffer 1 Data Byte 5
#define RXB1D6				0x7C	// Receive Buffer 1 Data Byte 6
#define RXB1D7				0x7D	// Receive Buffer 1 Data Byte 7


#define RXF0SIDH			0x00	// Filter 0 Standard Identifier High
#define RXF1SIDH			0x04	// Filter 1 Standard Identifier High
#define RXF2SIDH			0x08	// Filter 2 Standard Identifier High
#define RXF3SIDH			0x10	// Filter 3 Standard Identifier High
#define RXF4SIDH			0x14	// Filter 4 Standard Identifier High
#define RXF5SIDH			0x18	// Filter 5 Standard Identifier High

#define RXF0SIDL			0x01	// Filter 0 Standard Identifier Low
#define RXF1SIDL			0x05	// Filter 1 Standard Identifier Low
#define RXF2SIDL			0x09	// Filter 2 Standard Identifier Low
#define RXF3SIDL			0x11	// Filter 3 Standard Identifier Low
#define RXF4SIDL			0x15	// Filter 4 Standard Identifier Low
#define RXF5SIDL			0x19	// Filter 5 Standard Identifier Low

#define RXM0SIDH			0x20	// Mask 0 Standard Identifier High
#define RXM1SIDH			0x24	// Mask 1 Standard Identifier High

#define RXM0SIDL			0x21	// Mask 0 Standard Identifier Low
#define RXM1SIDL			0x25	// Mask 1 Standard Identifier Low

//		-- Interrupts -- 

#define CANINTE				0x2B	// Interrupt Enable
#define CANINTF				0x2C	// Interrupt Flag

			




typedef enum {	
		NORMAL		=	0b000,
		SLEEP		=	0b001,
		LOOPBACK	=	0b010,
		LISTEN_ONLY	=	0b011,
		CONFIG		=	0b100 
} mcp2515_mode_t;




uint8_t mcp2515_init ( struct spi_device * spi_dev );

void mcp2515_reset( struct spi_device * spi_dev );

void mcp2515_writeRegister ( struct spi_device * spi_dev, uint8_t reg_addr, uint8_t reg_data );

uint8_t mcp2515_readRegister ( struct spi_device * spi_dev, uint8_t reg_addr );

void mcp2515_sendCanMessage ( struct spi_device * spi_dev, uint8_t DLC, uint8_t * data, uint16_t address, uint8_t bufferNumber);


void mcp2515_setToMode (struct spi_device * spi_dev, mcp2515_mode_t mode );

mcp2515_mode_t mcp2515_getCurrentMode ( struct spi_device * spi_dev );

void mcp2515_write_array (struct spi_device * spi_dev, uint8_t * data, uint8_t len, uint8_t first_reg_addr );	//Not working, must fix

uint8_t mcp2515_identifyRecievedChannel ( struct spi_device * spi_dev );

uint8_t mcp2515_getReceivedMessage ( struct spi_device * spi_dev, uint8_t bufferNum, uint8_t * data, uint8_t len );

uint8_t mcp2515_bitModifyRegister ( struct spi_device * spi_dev, uint8_t regAddres, uint8_t dataMask, uint8_t data);

uint8_t mcp2515_readRXbuffer ( struct spi_device * spi_dev, uint8_t bufferNumber, uint8_t * data, uint8_t dlc);
void mcp2515_read_array (struct spi_device * spi_dev, uint8_t * data, uint8_t len, uint8_t first_reg_addr );

/* Save and load ecu data */
void save_state(struct spi_device * spi_dev, fsm_ecu_data_t *ecu_data);
void load_state(struct spi_device * spi_dev, fsm_ecu_data_t *ecu_data);
void spi_init_pins(void);
struct spi_device spi_init_module(void);





#endif /* MCP2515_H_ */