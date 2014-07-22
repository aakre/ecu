/*
 * mcp2515.c
 *
 * Created: 05.04.2014 17:42:17
 *  Author: Kristian Windows
 */ 

#include "mcp2515.h"
#include <asf.h>

#define ECU_DATA_BUF		TXB2D0
#define ECU_DATA_BUF_LEN	7


#define MCP2515_INSTR_RESET			0xC0
#define MCP2515_INSTR_READ			0x03
#define	MCP2515_INSTR_READ_RX_BUF	0x90			//must be or'ed with the correct address
#define	MCP2515_INSTR_WRITE			0x02					
#define	MCP2515_INSTR_LOAD_TX_BUF	0x00			//not yet understood
#define	MCP2515_INSTR_RTS			0x80		//must be OR'ed with desired TX-buffers to send
#define	MCP2515_INSTR_READ_STATUS	0xA0
#define	MCP2515_INSTR_RX_STATUS		0xB0		// not yet understood
#define	MCP2515_INSTR_BIT_MODIFY	0x05		





typedef enum {
	MCP2515_CMD_RESET = 0xC0,
	MCP2515_CMD_READ = 0x03,
	MCP2515_CMD_WRITE = 0x02,
	MCP2515_CMD_RTS = 0x80,
	MCP2515_CMD_READ_STATUS = 0xA0,
	MCP2515_CMD_BIT_MODIFY = 0x05,
} MCP2515_CMD_t;

void spi_init_pins(void) {
	static const gpio_map_t MCP2515_SPI_GPIO_MAP = {
		{SPI_CS_PIN,	SPI_CS_FUNCTION},
		{SPI_SCK_PIN,	SPI_SCK_FUNCTION },
		{SPI_MISO_PIN,	SPI_MISO_FUNCTION},
		{SPI_MOSI_PIN,	SPI_MOSI_FUNCTION},
		{SPI_CS_PIN,	SPI_CS_FUNCTION},
	};

	// Assign I/Os to SPI.
	gpio_enable_module(MCP2515_SPI_GPIO_MAP,
	sizeof(MCP2515_SPI_GPIO_MAP) / sizeof(MCP2515_SPI_GPIO_MAP[0]));

}

struct spi_device spi_init_module(void) {
	struct spi_device spi_device_conf = {
		.id = 0
	};

	spi_master_init(SPI_ADRESS);
	spi_master_setup_device(SPI_ADRESS, &spi_device_conf, SPI_MODE_0, 1000000, 0);
	spi_enable(SPI_ADRESS);
	return spi_device_conf;
}


uint8_t mcp2515_init ( struct spi_device * spi_dev ){

	
	mcp2515_reset(spi_dev);
	
	mcp2515_mode_t currentMode = mcp2515_getCurrentMode(spi_dev); 	
	if ( currentMode != CONFIG ){
		return 0;
	}
	
	
	// setup receive buffers:
	uint8_t receiveBuffer0Settings = RX_MASK_FILTER_OFF_bm;
	mcp2515_writeRegister(spi_dev, RXB0CTRL, receiveBuffer0Settings );
	
	uint8_t receiveBuffer1Settings = RX_MASK_FILTER_OFF_bm;
	mcp2515_writeRegister(spi_dev, RXB1CTRL, receiveBuffer1Settings );
	
	
	uint8_t interruptSettings = RX0IE_bm | RX1IE_bm;		//enable interrupts on full receive buffers
	mcp2515_writeRegister(spi_dev,CANINTE,interruptSettings);
	
	
	
	// set timing bits
	uint8_t cnf1RegisterSettings = SJW_LENGTH_1XTQ_bm | CAN_BAUD_RATE_PRESCALER;
	mcp2515_writeRegister(spi_dev, CNF1, cnf1RegisterSettings );
	
	uint8_t cnf2RegisterSettings = PRSEG_2_bm | PHSEG1_1_bm | BTLMODE_1_bm; 
	mcp2515_writeRegister(spi_dev,CNF2,cnf2RegisterSettings);
	
	uint8_t cnf3RegisterSettings = 	PHSEG2_1_bm;
	mcp2515_writeRegister(spi_dev,CNF3,cnf3RegisterSettings);
	
	
	
	
	mcp2515_setToMode(spi_dev,NORMAL);			// finish initializing
	currentMode = mcp2515_getCurrentMode(spi_dev);
	if ( currentMode != NORMAL ){
		return 0;
	}

	return 1;
	
}

void mcp2515_reset( struct spi_device * spi_dev ){
	
	spi_select_device(MCP2515_SPI_ADDRESS, spi_dev);
	spi_write_single(MCP2515_SPI_ADDRESS, MCP2515_INSTR_RESET);
	spi_deselect_device(MCP2515_SPI_ADDRESS, spi_dev);
}

void mcp2515_writeRegister ( struct spi_device * spi_dev, uint8_t reg_addr, uint8_t reg_data ){

	spi_select_device(MCP2515_SPI_ADDRESS, spi_dev);
	
	uint8_t data[3] = {
		MCP2515_INSTR_WRITE,
		reg_addr,
		reg_data
	};
	
	spi_write_packet(MCP2515_SPI_ADDRESS, data,3);
	
	spi_deselect_device(MCP2515_SPI_ADDRESS, spi_dev);
}


uint8_t mcp2515_readRegister ( struct spi_device * spi_dev, uint8_t reg_addr ){
	
	spi_select_device(MCP2515_SPI_ADDRESS, spi_dev);
	
	uint8_t data[2] = {
		MCP2515_INSTR_READ,
		reg_addr
	};
		
	spi_write_packet(MCP2515_SPI_ADDRESS, data, 2 );
	uint8_t registerValue[1] = {0x00};
	
	spi_read_packet(MCP2515_SPI_ADDRESS, registerValue, 1 );

	spi_deselect_device(MCP2515_SPI_ADDRESS, spi_dev);
	return registerValue[0];
	
}

void mcp2515_sendCanMessage ( struct spi_device * spi_dev, uint8_t DLC, uint8_t * data, uint16_t address, uint8_t bufferNumber ){
	
	gpio_set_pin_low(LED2);
	uint8_t messageAddress;
	uint8_t messageDLCAddress;
	uint8_t dataAddress;
	uint8_t controlRegisterAddress;
	
	switch(bufferNumber){
		
		case BUFFER0:
			messageAddress = TXB0SIDH;
			messageDLCAddress = TXB0DLC;
			dataAddress = TXB0D0;
			controlRegisterAddress = TXB0CTRL;

			break;
			
		case BUFFER1:
			messageAddress = TXB1SIDH;
			messageDLCAddress = TXB1DLC;
			dataAddress = TXB1D0;
			controlRegisterAddress = TXB1CTRL;
			break;
			
		default:
			return; // fault!!
	}
	
	uint8_t dataToSend[13] = {( address >> 3),(address << 5),0,0,DLC};
		
	//for ( uint8_t i = 5; i < 5+DLC; i++){
	//	dataToSend[i] = data[i-5];
	//}
	//mcp2515_write_array(spi_dev,dataToSend,5+DLC,messageAddress);
	
	// Set up length of message
	mcp2515_writeRegister(spi_dev, messageDLCAddress, DLC );
	
	// set up address
	uint8_t addressMSB = ( address >> 3);
	mcp2515_writeRegister(spi_dev, messageAddress, addressMSB );
	uint8_t addressLSB = (address << 5);
	mcp2515_writeRegister(spi_dev, messageAddress + 1 , addressLSB);
	
	// set up data
	
	mcp2515_write_array(spi_dev,data,DLC,dataAddress);
	
	// Request data transmission, priority set to low
	uint8_t dataTransmissionSetting = MCP2515_CAN_MESSAGE_PRIORITY_HIGHEST_bm | MCP2515_CAN_TRANSMIT_REQUEST_bm;
	mcp2515_writeRegister (spi_dev, controlRegisterAddress, dataTransmissionSetting );
	gpio_set_pin_high(LED2);
	//done
}

void mcp2515_setToMode (struct spi_device * spi_dev, mcp2515_mode_t mode ){
	
	uint8_t data[4] = {
		MCP2515_CMD_BIT_MODIFY,
		MCP2515_CAN_CTRL_REG_ADDR,	
		MCP2515_MODE_bm ,	
		( mode << 5 )				// data
	};
	spi_select_device(MCP2515_SPI_ADDRESS, spi_dev);
	spi_write_packet(MCP2515_SPI_ADDRESS, data, 4);
	spi_deselect_device(MCP2515_SPI_ADDRESS, spi_dev);
	
	
}

mcp2515_mode_t mcp2515_getCurrentMode ( struct spi_device * spi_dev ){
	
	uint8_t regValue = mcp2515_readRegister(spi_dev, MCP2515_CAN_CTRL_REG_ADDR);
	regValue &= MCP2515_MODE_bm;
	
	return ( regValue >> 5);
}

void mcp2515_write_array (struct spi_device * spi_dev, uint8_t * data, uint8_t len, uint8_t first_reg_addr ){		//This function is not working, must be fixed
	
	uint8_t dataToSend[10] = {MCP2515_INSTR_WRITE,first_reg_addr};
	
	for ( uint8_t i = 2 ; i < 11 ; ++i){
		dataToSend[i] = data[i-2]; 
	}
	
	spi_select_device(MCP2515_SPI_ADDRESS, spi_dev);	
	spi_write_packet(MCP2515_SPI_ADDRESS,dataToSend,len+2);
	spi_deselect_device(MCP2515_SPI_ADDRESS, spi_dev);

}


uint8_t mcp2515_identifyRecievedChannel ( struct spi_device * spi_dev ){	//function not used
	
	uint8_t interruptFlagRegister;
	interruptFlagRegister = mcp2515_readRegister(spi_dev, CANINTF );
	
	return interruptFlagRegister & ( 0b11 << 0 );
}

uint8_t mcp2515_getReceivedMessage ( struct spi_device * spi_dev, uint8_t bufferNum, uint8_t * data, uint8_t len ){
	gpio_set_pin_low(LED1);
	uint8_t messageAddress;
	uint8_t messageDLCAddress;
	uint8_t receivedMessageInterrupt_bm;

	switch(bufferNum){
		
		case BUFFER0:
			messageAddress			= RXB0D0;
			messageDLCAddress		= RXB0DLC;
			receivedMessageInterrupt_bm = ( 1 << RX0IF );

			break;
			
		case BUFFER1:
			messageAddress			= RXB1D0;
			messageDLCAddress		= RXB1DLC;
			receivedMessageInterrupt_bm = ( 1 << RX1IF  );

			break; 
			
		default:
			return 0xFF; //faulty call of function
	}
	
	
	//get DLC:
	uint8_t DLC =  mcp2515_readRegister(spi_dev, messageDLCAddress);
	DLC &= RX_DLC_BITS_bm;	// filter away any unwanted bits
	
	// get Data:
	mcp2515_readRXbuffer(spi_dev,bufferNum,data,DLC);	
	//mcp2515_read_array(spi_dev,data,DLC,messageAddress);
	
	
	//reset interrupt flag: (perhaps no longer needed due to the readRXbuffer function over - it should automatically reset the flag)
	//mcp2515_bitModifyRegister(spi_dev, CANINTF, receivedMessageInterrupt_bm,0x00);
	gpio_set_pin_high(LED1);
	return DLC;

}

void save_state(struct spi_device * spi_dev, fsm_ecu_data_t *ecu_data) {
	uint8_t addr = ECU_DATA_BUF;
	uint8_t dataToWrite[ECU_DATA_BUF_LEN] = { 
		ecu_data->state,
		ecu_data->flag_start_precharge,
		(uint8_t)ecu_data->flag_drive_enable,
		ecu_data->flag_brake_implausible,
		(int8_t)((int16_t)ecu_data->control_u >> 8),
		(int8_t)(ecu_data->control_u),
		ecu_data->config_max_trq
	}; 
	mcp2515_write_array(spi_dev,dataToWrite, ECU_DATA_BUF_LEN ,addr);
}

void load_state(struct spi_device * spi_dev, fsm_ecu_data_t *ecu_data) {
	uint8_t addr = ECU_DATA_BUF;
	uint8_t data[ECU_DATA_BUF_LEN] = {0};
	mcp2515_read_array(spi_dev, data, ECU_DATA_BUF_LEN, addr);
	ecu_data->state					 = (fsm_ecu_state_t)data[0];
	ecu_data->flag_start_precharge	 = data[1];
	ecu_data->flag_drive_enable		 = (flag_drive_enable_t)data[2];
	ecu_data->flag_brake_implausible = data[3];
	uint16_t temp					 = data[4] << 8;
	ecu_data->control_u				 = (int16_t)(temp | (uint16_t)data[5]);
	ecu_data->config_max_trq		 = (uint8_t)data[6];
}

uint8_t mcp2515_bitModifyRegister ( struct spi_device * spi_dev, uint8_t regAddres, uint8_t dataMask, uint8_t data){
	
	spi_select_device( MCP2515_SPI_ADDRESS, spi_dev);
	uint8_t bitModifyMessage[4] = {MCP2515_CMD_BIT_MODIFY,regAddres,dataMask, data};
	spi_write_packet(MCP2515_SPI_ADDRESS,bitModifyMessage,4);
	spi_deselect_device(MCP2515_SPI_ADDRESS,spi_dev);
	
}

uint8_t mcp2515_readRXbuffer ( struct spi_device * spi_dev, uint8_t bufferNumber, uint8_t * data, uint8_t dlc){
	
	uint8_t rxBufferReadAddress;
	switch ( bufferNumber){
		
		case 0:
			rxBufferReadAddress = 0b010;
			break;
			
		case 1:
			rxBufferReadAddress = 0b110;
			break;
		default:
			rxBufferReadAddress = 0b010; //Added 21/5-14
			break;
	}
	
	spi_select_device(MCP2515_SPI_ADDRESS,spi_dev);
	
	uint8_t spi_cmd = MCP2515_INSTR_READ_RX_BUF | rxBufferReadAddress;
	spi_write_single(MCP2515_SPI_ADDRESS, spi_cmd);
	spi_read_packet(MCP2515_SPI_ADDRESS,data,dlc);	
	
	spi_deselect_device(MCP2515_SPI_ADDRESS,spi_dev);
	
	//Missing return? / --> Added 21/5-14
}

void mcp2515_read_array (struct spi_device * spi_dev, uint8_t * data, uint8_t len, uint8_t first_reg_addr ){
	
	uint8_t dataToSend[2] = {MCP2515_INSTR_READ,first_reg_addr};
		
	spi_select_device(MCP2515_SPI_ADDRESS, spi_dev);
	spi_write_packet(MCP2515_SPI_ADDRESS,dataToSend,2);
	spi_read_packet(MCP2515_SPI_ADDRESS,data,len);
	spi_deselect_device(MCP2515_SPI_ADDRESS, spi_dev);
	
}