/* ****************************************************************************/
/*		  PWr																	*/
/*			01-12-2010r															*/
/*																			*/
/*			Program Grzegorza Ko�odziejczyka										*/
/*			 RFM70 - header														*/
/*			uC STM8s - RFM70 (2,4 GHz)										    */
/*			ver. 1.0															  */
/*			 												*/
/*																		*/
/* ****************************************************************************/


#ifndef _RFM70_H_
#define _RFM70_H_
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "main.h"

typedef uint8_t    u8;
typedef uint16_t    u16;

typedef struct _SpiPort
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *CE_PORT;
	uint16_t CE_PIN;
	GPIO_TypeDef *CSN_PORT;
	uint16_t CSN_PIN;
}SpiPort;

//#define CE_PORT		GPIOA
//#define CE_PIN		GPIO_PIN_3
//
//#define CSN_PORT	GPIOA
//#define CSN_PIN		GPIO_PIN_4


//#define SCK_PORT	GPIOA
//#define SCK_PIN		GPIO_PIN_5
//
//#define MISO_PORT	GPIOA
//#define MISO_PIN	GPIO_Pin_6
//
//#define MOSI_PORT	GPIOA
//#define MOSI_PIN	GPIO_Pin_7

#define MAX_PACKET_LEN  32// max value is 32

//************************FSK COMMAND and REGISTER****************************************//


// SPI(RFM70) commands
typedef enum {
	RD_RX_PLOAD							=	(uint8_t)0x61,  // Define RX payload register address
	WR_TX_PLOAD							=	(uint8_t)0xA0,  // Define TX payload register address
	FLUSH_TX								=	(uint8_t)0xE1,  // Define flush TX register command
	FLUSH_RX								=	(uint8_t)0xE2,  // Define flush RX register command
	REUSE_TX_PL							=	(uint8_t)0xE3,  // Define reuse TX payload register command
	NOP_NOP									=	(u8)0xFF  // Define No Operation, might be used to read status register
}	RFM70_Commands_TypeDef;

typedef enum {
	W_TX_PAYLOAD_NOACK_CMD	=	(uint8_t)0xb0,
	W_ACK_PAYLOAD_CMD				=	(uint8_t)0xa8,
	R_RX_PL_WID_CMD					=	(uint8_t)0x60
}	RFM70_TRx_TypeDef;

typedef enum {
	ACTIVATE_CMD						=	(u8)0x50
}	RFM70_CommandsW_Typedef;
typedef enum {
	READ_REG								= (u8)0x00,  // Define read command to register
	WRITE_REG								=	(u8)0x20  // Define write command to register
} RFM70_RWReg_TypeDef;

// SPI(RFM70) registers(addresses)
typedef enum {
 CONFIG				=	(u8)0x00,  // 'Config' register address
 EN_AA				=	(u8)0x01,  // 'Enable Auto Acknowledgment' register address
 EN_RXADDR		=	(u8)0x02,  // 'Enabled RX addresses' register address
 SETUP_AW			=	(u8)0x03,  // 'Setup address width' register address
 SETUP_RETR		=	(u8)0x04,  // 'Setup Auto. Retrans' register address
 RF_CH				=	(u8)0x05,  // 'RF channel' register address
 RF_SETUP			=	(u8)0x06,  // 'RF setup' register address
 STATUS				=	(u8)0x07,  // 'Status' register address
 OBSERVE_TX		=	(u8)0x08,  // 'Observe TX' register address
 CD						=	(u8)0x09,  // 'Carrier Detect' register address
 RX_ADDR_P0		=	(u8)0x0A,  // 'RX address pipe0' register address
 RX_ADDR_P1		=	(u8)0x0B,  // 'RX address pipe1' register address
 RX_ADDR_P2		=	(u8)0x0C,  // 'RX address pipe2' register address
 RX_ADDR_P3		=	(u8)0x0D,  // 'RX address pipe3' register address
 RX_ADDR_P4		=	(u8)0x0E,  // 'RX address pipe4' register address
 RX_ADDR_P5		=	(u8)0x0F,  // 'RX address pipe5' register address
 TX_ADDR			=	(u8)0x10,  // 'TX address' register address
 RX_PW_P0			=	(u8)0x11,  // 'RX payload width, pipe0' register address
 RX_PW_P1			=	(u8)0x12,  // 'RX payload width, pipe1' register address
 RX_PW_P2			=	(u8)0x13,  // 'RX payload width, pipe2' register address
 RX_PW_P3			=	(u8)0x14,  // 'RX payload width, pipe3' register address
 RX_PW_P4			=	(u8)0x15,  // 'RX payload width, pipe4' register address
 RX_PW_P5			=	(u8)0x16,  // 'RX payload width, pipe5' register address
 FIFO_STATUS	=	(u8)0x17,  // 'FIFO Status Register' register address
 FEATURE			=	(u8)0x1D,	 // 'FEATURE register address
 PAYLOAD_WIDTH=	(u8)0x1f  // 'payload length of 256 bytes modes register address
}	RFM70_RegAddr_TypeDef;

typedef enum {
	A_00	=	(u8)0x00,
	A_01	=	(u8)0x01,
	A_02	=	(u8)0x02,
	A_03	=	(u8)0x03,
	A_04	=	(u8)0x04,
	A_05	=	(u8)0x05,
	A_0C	=	(u8)0x0C,
	A_0D	=	(u8)0x0D,
	A_0E	=	(u8)0x0E
} RFM70_RegAddr1_TypeDef;	

//interrupt status
typedef enum {
	STATUS_RX_DR 	= 	(u8)0x40,
	STATUS_TX_DS 	=		(u8)0x20,
	STATUS_MAX_RT =		(u8)0x10,
	STATUS_TX_FULL =	(u8)0x01
} RFM70_IrqStat_TypeDef;

//FIFO_STATUS
typedef enum {
FIFO_STATUS_TX_REUSE = 	(u8)0x40,
FIFO_STATUS_TX_FULL = 	(u8)0x20,
FIFO_STATUS_TX_EMPTY = 	(u8)0x10,

FIFO_STATUS_RX_FULL = 	(u8)0x02,
FIFO_STATUS_RX_EMPTY = 	(u8)0x01
}	RFM70_FifoStat_Typedef;

#define PRIM_RX     (1<<0)

/*****************************

	 FUNCTION DECLARATION

****************************/

void SPI_Read_Buf(u8 reg, u8 *pBuf, u8 bytes);
void SPI_Write_Buf(RFM70_RWReg_TypeDef RW,RFM70_RegAddr_TypeDef addr, u8 *pBuf, u8 length);

u8 SPI_Read_Reg(RFM70_RegAddr_TypeDef addr);
void SPI_Write_Reg(RFM70_RWReg_TypeDef RW,RFM70_RegAddr_TypeDef addr, u8 value);

void SPI_Command(RFM70_Commands_TypeDef cmd);

void RFM70_SwitchToTxMode(void);
void RFM70_SwitchToRxMode(void);

void SPI_Bank1_Read_Reg(u8 reg, u8 *pBuf);
void SPI_Bank1_Write_Reg(u8 reg, u8 *pBuf);
void SwitchCFG(char _cfg);

void CSN_Low(void);
void CSN_High(void);
void CE_Low(void);
void CE_High(void);

void RFM70_Initialize(SpiPort *port);

void Send_Packet(RFM70_TRx_TypeDef type,u8* pbuf,u8 len);
int Receive_Packet(u8 *rx_buf, u16 *packet_len, uint32_t timeout);


// Banks initialize
void Ini_Bank0(void);
void Ini_Bank1(void);


void DelayMs(int ms);

#endif
