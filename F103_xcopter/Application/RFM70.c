/* ****************************************************************************/
/*																			 PWr																	*/
/*																	 01-12-2010r															*/
/*																																						*/
/*												Program Grzegorza Ko�odziejczyka										*/
/*													  		  RFM70 - library														*/
/*													 uC STM8s - RFM70 (2,4 GHz)										    */
/*																		ver. 1.0															  */
/*																  							     												*/
/*																																						*/
/* ****************************************************************************/

//#include "stm8s.h"
//#include "stm8s_spi.h"
#include <stddef.h>
#include <stdint.h>
#include "RFM70.h"

SpiPort g_SpiPort = {0};

// ***********************************************
// 		sending to RFM70 MSB first
// **********************************************/

//Banks1 registers which must be writen
//0-8;	Byte(MSB->LSB), Bit(MSB->LSB)
const u8 Bank1_Reg00[]={0x40,0x4B,0x01,0xE2};
const u8 Bank1_Reg01[]={0xC0,0x4B,0x00,0x00};
const u8 Bank1_Reg02[]={0xD0,0xFC,0x8C,0x02};
const u8 Bank1_Reg03[]={0x99,0x00,0x39,0x41};
const u8 Bank1_Reg04[]={0xD9,0x9E,0x86,0x0B};
const u8 Bank1_Reg05[]={0x24,0x06,0x7F,0xA6};
//12-14;	Byte(LSB->MSB), Bit(MSB->LSB)
const u8 Bank1_Reg0C[]={0x00,0x12,0x73,0x00};
const u8 Bank1_Reg0D[]={0x36,0xB4,0x80,0x00};
const u8 Bank1_Reg0E[]={0x41,0x20,0x08,0x04,0x81,0x20,0xCF,0xF7,0xFE,0xFF,0xFF};

//Initializing values of Bank0 registres
const u8 Bank0_Reg[][2]={
{0x00,0x0F},//	0CONFIG	//	reflect RX_DR\TX_DS\MAX_RT,Enable CRC ,2byte,POWER UP,PRX
{0x01,0x3F},//	1EN_AA		//	Enable auto acknowledgement data pipe5\4\3\2\1\0
{0x02,0x3F},//	2EX_RXADDR	//	Enable RX Addresses pipe5\4\3\2\1\0
{0x03,0x03},//	3SETUP_AW	//RX/TX address field width 5byte
{0x04,0xff},//	4SETUP_RETR	//auto retransmission dalay (4000us),auto retransmission count(15)
{0x05,0x17},//	5RF_CH	//	23 channel
{0x06,0x17},//	6RF_SETUP	//air data rate-1M,out power 0dbm,setup LNA gain
{0x07,0x07},//	7STATUS	//
{0x08,0x00},//	8OBSERVER_TX	//
{0x09,0x00},//	9CD	//
{0x0C,0xc3},//	10RX_ADDR_P2	//	only LSB Receive address data pipe 2, MSB bytes is equal to RX_ADDR_P1[39:8]
{0x0D,0xc4},//	11RX_ADDR_P3	//	only LSB Receive address data pipe 3, MSB bytes is equal to RX_ADDR_P1[39:8]
{0x0E,0xc5},//	12RX_ADDR_P4	//	only LSB Receive address data pipe 4, MSB bytes is equal to RX_ADDR_P1[39:8]
{0x0F,0xc6},//	13RX_ADDR_P5	//	only LSB Receive address data pipe 5, MSB bytes is equal to RX_ADDR_P1[39:8]
{0x11,0x20},//	14RX_PW_P0	//	Number of bytes in RX payload in data pipe0(32 byte) 
{0x12,0x20},//	15RX_PW_P1	//	Number of bytes in RX payload in data pipe1(32 byte)
{0x13,0x20},//	16RX_PW_P2	//	Number of bytes in RX payload in data pipe2(32 byte)
{0x14,0x20},//	17RX_PW_P3	//	Number of bytes in RX payload in data pipe3(32 byte)
{0x15,0x20},//	18RX_PW_P4	//	Number of bytes in RX payload in data pipe4(32 byte)
{0x16,0x20},//	19RX_PW_P5	//	Number of bytes in RX payload in data pipe5(32 byte)
{0x17,0x00},//	20FIFO_STATUS	//	fifo status
{0x1C,0x3F},//	21DYNPD	//	Enable dynamic payload length data pipe5\4\3\2\1\0
{0x1D,0x07}//	22FEATURE	//	Enables Dynamic Payload Length,Enables Payload with ACK,Enables the W_TX_PAYLOAD_NOACK command 
};
//Address = Bank1_Reg0A pipe 0 & Bank1_Reg0B pipe 1; in Bank 0
const u8 RX0_Address[]={0xE7,0xE7,0xE7,0xE7,0xE7};//Receive address data pipe 0	-> default value
const u8 RX1_Address[]={0xC2,0xC2,0xC2,0xC2,0xC2};//Receive address data pipe 1 -> default value

/*******************************************************************/
/*******************************************************************/

extern u8 rx_buf[MAX_PACKET_LEN];

extern void delay_200ms(void);
extern void delay_50ms(void);


u8 status;

// **************************************************
// Function: SPI_R/W(void);
// **************************************************/
u8 SPI_R(void)
{
//	while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET) {}	// wait until TX buffer will be empty
	//while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)== RESET) {}	// wait until TX buffer will be empty
	//SPI_I2S_SendData(SPI1,0);																		// Send no data - to read the value from register
	//while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY) == SET) {}		// wait until SPI is ready
	//while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)== RESET) {}	// wait until Rx buffer will be ready to read
	
	//return(SPI_I2S_ReceiveData(SPI1));
	uint8_t RxData = 0;
	HAL_SPI_Receive(g_SpiPort.hspi, &RxData, 1, -1);
	return RxData;
}
void SPI_W(u8 value)	// OK
{
	//while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) {}		// wait until SPI is ready
	//while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)== RESET) {}	// wait until TX buffer will be empty
	//status=0;
	
	//SPI_I2S_SendData(SPI1, value);																// Write the read register command
	//while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) {}		// wait until SPI is ready
	//while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)== RESET) {}	// wait until Rx buffer will be ready to read
	//status=SPI_I2S_ReceiveData(SPI1);
	uint8_t TxData = value;
	//uint8_t RxData = 0;
	HAL_SPI_Transmit(g_SpiPort.hspi, &TxData, 1, -1);
// Read the status register
}

/**************************************************         
Function: CSN_Low(); CSN_High();                                  
**************************************************/
void CSN_Low(void)
{
	//GPIO_ResetBits(CSN_PORT, CSN_PIN);
	HAL_GPIO_WritePin(g_SpiPort.CSN_PORT , g_SpiPort.CSN_PIN, GPIO_PIN_RESET);
}
void CSN_High(void)
{
	//GPIO_SetBits(CSN_PORT, CSN_PIN);
	HAL_GPIO_WritePin(g_SpiPort.CSN_PORT, g_SpiPort.CSN_PIN, GPIO_PIN_SET);
}

/**************************************************         
Function: CE_Low(); CE_High();                                  
**************************************************/
void CE_Low(void){
	HAL_GPIO_WritePin(g_SpiPort.CE_PORT , g_SpiPort.CE_PIN, GPIO_PIN_RESET);
	//GPIO_ResetBits(CE_PORT, CE_PIN);
}

void CE_High(void){
	//GPIO_SetBits(CE_PORT, CE_PIN);
	HAL_GPIO_WritePin(g_SpiPort.CE_PORT, g_SpiPort.CE_PIN, GPIO_PIN_SET);
}

void SPI_Command(RFM70_Commands_TypeDef cmd)
{
	CSN_Low();	// CSN low, init SPI transaction
	SPI_W(cmd);	// Do cmd
	CSN_High();	// CSN high again, end SPI transaction
}
/**************************************************         
Function: SPI_Write_Reg();                                  
 
Description:                                                
	Writes value 'value' to register 'reg'              
**************************************************/
void SPI_Write_Reg(RFM70_RWReg_TypeDef write,RFM70_RegAddr_TypeDef addr, u8 value)
{
	CSN_Low();	// CSN low, init SPI transaction
	SPI_W(write|addr);	// select register
	SPI_W(value);	// ..and write value to it..
	CSN_High();	// CSN high again, end SPI transaction
}

/**************************************************         
Function: SPI_Read_Reg();                                   
 
Description:                                                
	Read one u8 from RFM70 register, 'reg'           
**************************************************/
u8 SPI_Read_Reg(RFM70_RegAddr_TypeDef addr)                               
{                                                           
	u8 value;
	
	CSN_Low();	// CSN low, initialize SPI transaction     
	SPI_W(addr);	// Select register to read from.   		
	value = SPI_R();	// ..then read register value 					 
	CSN_High();	// CSN high, terminate SPI communication

	return(value);	// return register value
}                                                           
/**************************************************/        

/**************************************************         
Function: SPI_Read_Buf();                                   
 
Description:                                                
	Reads 'length' #of length from register 'reg'         
**************************************************/
void SPI_Read_Buf(u8 reg, u8 *pBuf, u8 length)     
{                                                           
	//u8 byte_ctr;

	CSN_Low();	
	SPI_W(reg);	// Select register to write, and read status u8
	//for(byte_ctr=0;byte_ctr<length;byte_ctr++)
	//	pBuf[byte_ctr] = SPI_R();	// Perform SPI_RW to read u8 from RFM70
	HAL_SPI_Receive(g_SpiPort.hspi, pBuf, length, -1);
	CSN_High();	// Set CSN high again
}                                                           
/**************************************************/        

/**************************************************         
Function: SPI_Write_Buf();                                  

Description:                                                
	Writes contents of buffer '*pBuf' to RFM70         
**************************************************/
void SPI_Write_Buf(RFM70_RWReg_TypeDef RW,RFM70_RegAddr_TypeDef addr, u8 *pBuf, u8 length)    
{                                                           
	//u8 byte_ctr;

	CSN_Low();	// Set CSN low, init SPI tranaction   
	SPI_W(RW|addr);	// Select register to write to and read status u8
	//for(byte_ctr=0; byte_ctr<length; byte_ctr++) // then write all u8 in buffer(*pBuf)
	//	SPI_W(*pBuf++);

	//HAL_SPI_TransmitReceive(g_SpiPort.hspi, &TxData, &RxData, length, -1);
	HAL_SPI_Transmit(g_SpiPort.hspi, pBuf, length, -1);

	CSN_High();	// Set CSN high again
	     
}

/**************************************************
Function: SPI_Activate(RFM_CommandsW_Typedef activate);
Description:
	Change the bank0<->bank1
**************************************************/
void SPI_Activate(RFM70_CommandsW_Typedef activate, u8 cmd)
{
	CSN_Low();
	SPI_W(activate);
	SPI_W(cmd);
	CSN_High();
}
/***********INITIALIZE FUNCTIONS***************************

**************************************************
Function: SPI_Read_Buf();                                   
 
Description:                                                
	Reads 'length' #of length from register 'reg'         
**************************************************/
void SPI_Read_Ini_Buf(u8 reg, u8 *pBuf, u8 length)     
{                                                           
	u8 byte_ctr;

	CSN_Low();	
	SPI_W(reg);	// Select register to write, and read status u8
	for(byte_ctr=0;byte_ctr<length;byte_ctr++)              
		pBuf[byte_ctr] = SPI_R();	// Perform SPI_RW to read u8 from RFM70 
	CSN_High();	// Set CSN high again
}                                                           
/**************************************************/        

/**************************************************         
Function: SPI_Write_Ini_Buf();                                  

Description:                                                
	Writes contents of buffer '*pBuf' to RFM70         
*************************************************/
void SPI_Write_Ini_Buf(RFM70_RWReg_TypeDef RW,RFM70_RegAddr1_TypeDef addr, u8 *pBuf, u8 length)    
{                                                           
	u8 byte_ctr;                                

	CSN_Low();	// Set CSN low, init SPI tranaction   
	SPI_W(RW|addr);	// Select register to write to and read status u8
	for(byte_ctr=0; byte_ctr<length; byte_ctr++) // then write all u8 in buffer(*pBuf) 
		SPI_W(*pBuf++);                
	CSN_High();	// Set CSN high again
	     
}

/**************************************************
Function: SwitchToRxMode();
Description:
	switch to Rx mode
**************************************************/
void RFM70_SwitchToRxMode()
{
	//printf("%s\n", __FUNCTION__);
	u8 valueStatus;
	u8 valueConfig;

	SPI_Command(FLUSH_RX);//flush Rx
	valueStatus=SPI_Read_Reg(STATUS);	// read register STATUS's value
	SPI_Write_Reg(WRITE_REG,STATUS,valueStatus);// clear RX_DR or TX_DS or MAX_RT interrupt flag
	CE_Low();
	valueConfig=SPI_Read_Reg(CONFIG);	// read register CONFIG's value

	//if( (valueConfig & PRIM_RX) == 0 ){
//PRX
		valueConfig=valueConfig|0x01|0x02;
		SPI_Write_Reg(WRITE_REG, CONFIG, valueConfig); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled..
	//}

	CE_High();
	//printf("%s status=%d, config=%d\n", __FUNCTION__, valueStatus, valueConfig);
}

/*
 *
 *
 *
poprawka z forum
https://forbot.pl/forum/topic/2530-c-rfm70-24ghz/page/2/#comments

void RFM70_SwitchToRxMode()
{
uint8_t value;

//SPI_Command(FLUSH_RX);//flush Rx
SPI_Write_Reg(WRITE_REG,FLUSH_RXX,0);
value=SPI_Read_Reg(STATUS);	// read register STATUS's value
SPI_Write_Reg(WRITE_REG,STATUS,value);// clear RX_DR or TX_DS or MAX_RT interrupt flag

CE_Low();
value=SPI_Read_Reg(CONFIG);	// read register CONFIG's value//PRX

value|=0x01;
value|=0x02;
 	SPI_Write_Reg(WRITE_REG, CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled..
CE_High();
}
*/

/**************************************************
Function: RFM70_SwitchToTxMode();
Description:
	switch to Tx mode
**************************************************/
void RFM70_SwitchToTxMode()
{
	//printf("%s\n", __FUNCTION__);
	u8 valueRaw, value;
	SPI_Command(FLUSH_TX);//flush Tx
	CE_Low();
	valueRaw=SPI_Read_Reg(CONFIG);	// read register CONFIG's value
//PTX
	//if( (valueRaw & PRIM_RX)!=0 ){
		value=valueRaw&0xfe;	//	mask all bits without first / switch to PTX
		value |= 0x02; // set PWR_UP bit
  		SPI_Write_Reg(WRITE_REG, CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled.
	//}
	CE_High();
	//printf("%s configRaw=%d, config=%d\n", __FUNCTION__, valueRaw, value);
}

/*
 * poprawka z forum
https://forbot.pl/forum/topic/2530-c-rfm70-24ghz/page/2/#comments

void RFM70_SwitchToTxMode()
{
uint8_t value;
SPI_Command(FLUSH_TX);//flush Tx
value=SPI_Read_Reg(STATUS);	// read register STATUS's value
SPI_Write_Reg(WRITE_REG,STATUS,value);// clear RX_DR or TX_DS or MAX_RT interrupt flag

CE_Low();
value=SPI_Read_Reg(CONFIG);	// read register CONFIG's value
//PTX
	value=value&0xfe;	//	mask all bits without first / switch to PTX
	value|=0x02;
 	SPI_Write_Reg(WRITE_REG, CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled.
CE_High();
}
*/


/*
 *
 *
kolejna wersja z forum
https://forbot.pl/forum/topic/2530-c-rfm70-24ghz/page/2/#comments
#define Chip_Enable()           CE_PORT |= (1<<CE_BIT)
#define Chip_Disable()          CE_PORT &=~(1<<CE_BIT)

#define FLUSH_TX            0xE1
#define FLUSH_RX            0xE2
#define PRIM_RX     (1<<0)

//switches module to RX-mode, use after Send_Packet to receive data
void Select_RX_Mode(){
uint8_t Value;
SPI_Write_Command(FLUSH_RX);    //flush RX

Value=SPI_Read_STATUS();
SPI_Write_Register(STATUS,Value);  //clear RX_DR or TX_DS or MAX_RT interrupt flag

Chip_Disable();                                 //without this, the module won't switch modes properly
Value=SPI_Read_Register(CONFIG);	            //keep CONFIG's value
if( (Value&PRIM_RX)==0 ){                       //switch if NOT in RX-mode
    SPI_Write_Register(CONFIG, Value|PRIM_RX);  //set PRIM_RX in CONFIG's value and write it back back
}
Chip_Enable();                                  //without this, the module won't switch modes properly
}

//switches module to TX-mode
void Select_TX_Mode(void){
uint8_t Config;
SPI_Write_Command(FLUSH_TX); //flush TX

Chip_Disable();                                     //without this, the module won't switch modes properly
Config=SPI_Read_Register(CONFIG);                   //keep CONFIG's value
if( (Config&PRIM_RX)!=0 ){                          //switch if in RX-mode
	SPI_Write_Register(CONFIG, Config&(~PRIM_RX));  //clear PRIM_RX in CONFIG's value and write it back back
}
Chip_Enable();                                      //without this, the module won't switch modes properly
}
*/
/**************************************************
Function: SwitchCFG();
                                                            
Description:
	 access switch between Bank1 and Bank0 

Parameter:
	_cfg      1:register bank1
	          0:register bank0
Return:
     None
**************************************************/
void SwitchCFG(char _cfg)//1:Bank1 0:Bank0
{
	u8 Tmp;

	Tmp=SPI_Read_Reg(STATUS);	//Read the STATUS register
	Tmp=Tmp&0x80;	//Mask only the RBANK bit
	if( ( (Tmp)&&(_cfg==0) )
	||( ((Tmp)==0)&&(_cfg) ) )	//If bank isn't the same that is required, change it
	{
		SPI_Activate(ACTIVATE_CMD, 0x53);	//sending the required command
	}
}

/**************************************************
Function: SetChannelNum();
Description:
	set channel number

**************************************************/
void SetChannelNum(u8 channel)
{
	SPI_Write_Reg(WRITE_REG,RF_CH,channel);	// Write the channel number which works on the RFM70
}

/**************************************************
Function: Send_Packet
Description:
	fill FIFO to send a packet
Parameter:
	type: WR_TX_PLOAD or  W_TX_PAYLOAD_NOACK_CMD
	pbuf: a buffer pointer
	len: packet length
Return:
	None
**************************************************/
void Send_Packet(RFM70_TRx_TypeDef type,u8* pbuf,u8 len)
{
	u8 fifo_sta;
	
	do
	{
		fifo_sta=SPI_Read_Reg(FIFO_STATUS);	// read register FIFO_STATUS's value
	} while((fifo_sta&FIFO_STATUS_TX_EMPTY) == 0);

	SPI_Write_Buf(WRITE_REG, type, pbuf, len); // Writes data to buffer

	do{
		fifo_sta=SPI_Read_Reg(FIFO_STATUS);	// read register FIFO_STATUS's value
	} while((fifo_sta&FIFO_STATUS_TX_EMPTY) == 0);
}

/**************************************************
Function: Receive_Packet
Description:
	read FIFO to read a packet
Parameter:
	None
Return:
	None
**************************************************/

int Receive_Packet(u8 *rx_buf, u16 *packet_len, uint32_t timeout)
{
	uint32_t begin = HAL_GetTick();
	int ret = 0;
	int cur_len = 0;
	int total_to_be_received = -1;

	u8 len = 0,i = 0, sta = 0,fifo_sta = 0;

	do
	{
		sta=SPI_Read_Reg(STATUS);	// read register STATUS's value

		if((STATUS_RX_DR&sta) == 0x40)				// if receive data ready (RX_DR) interrupt
		{
			do
			{
				len = SPI_Read_Reg(R_RX_PL_WID_CMD);	// read length of recived packet

				if(cur_len + len <= *packet_len)
				{
					SPI_Read_Buf(RD_RX_PLOAD, rx_buf + cur_len, len);// read receive payload from RX_FIFO buffer

					if(total_to_be_received == -1){
						total_to_be_received = rx_buf[cur_len];
					}

					begin = HAL_GetTick();
				}
				else
				{
					break;
				}

				cur_len += len;

				if(cur_len >= total_to_be_received){
					SPI_Write_Reg(WRITE_REG, STATUS, sta);// clear RX_DR or TX_DS or MAX_RT interrupt flag
					*packet_len = cur_len;
					return 0;
				}

				fifo_sta=SPI_Read_Reg(FIFO_STATUS);	// read register FIFO_STATUS's value

			}while((fifo_sta&FIFO_STATUS_RX_EMPTY)==0); //while not empty

			SPI_Write_Reg(WRITE_REG, STATUS, sta);// clear RX_DR or TX_DS or MAX_RT interrupt flag
		}

		if(HAL_GetTick() - begin >= timeout){
			ret = 1;
			break;
		}
	} while(HAL_GetTick() - begin < timeout);

	*packet_len = cur_len;
	return ret;
}

void Ini_Bank0()
{
	u8 i, j;
 	u8 WriteArr[12];
	
	SwitchCFG(0);	//switch to bank0
	
	for(j=0;j<23;j++)	//initialize RFM70 - REGISTERS
	{
		SPI_Write_Reg(WRITE_REG, Bank0_Reg[j][0], Bank0_Reg[j][1]);
	}
	
	for(j=0;j<5;j++)	//initialize RFM70 - PIPES (RX0)
	{
		WriteArr[j]=RX0_Address[j];
	}
	SPI_Write_Buf(WRITE_REG, RX_ADDR_P0, &(WriteArr[0]), 5);
	
	for(j=0;j<5;j++)	//initialize RFM70 - PIPES (RX1)
	{
		WriteArr[j]=RX1_Address[j];
	}
	SPI_Write_Buf(WRITE_REG, RX_ADDR_P1, &(WriteArr[0]), 5);
	
	for(j=0;j<5;j++)	// initialize RFM70 - TX_ADDR
	{
		WriteArr[j]=RX0_Address[j];
	}
	SPI_Write_Buf(WRITE_REG, TX_ADDR, &(WriteArr[0]), 5);
	
	i=SPI_Read_Reg(FEATURE);
	if(i==0) // i!=0 showed that chip has been actived.so do not active again.
		SPI_Activate(ACTIVATE_CMD,0x73); // Activate
	for(j=22;j>=21;j--)
	{
		SPI_Write_Reg(WRITE_REG, Bank0_Reg[j][0], Bank0_Reg[j][1]); // Reinitialize the start value of DYNPD and FEATURE
	}
}
void Ini_Bank1()
{
	u8 j;
 	u8 WriteArr[12];
	
	SwitchCFG(1);
//	reg:00
	for(j=0; j<4; j++)
	{
		WriteArr[j]=Bank1_Reg00[j];
	}
	SPI_Write_Ini_Buf(WRITE_REG, A_00, &(WriteArr[0]),4);
//	reg:01
	for(j=0; j<4; j++)
	{
		WriteArr[j]=Bank1_Reg01[j];
	}
	SPI_Write_Ini_Buf(WRITE_REG, A_01, &(WriteArr[0]),4);
//	reg:02
	for(j=0; j<4; j++)
	{
		WriteArr[j]=Bank1_Reg02[j];
	}
	SPI_Write_Ini_Buf(WRITE_REG, A_02, &(WriteArr[0]),4);
//	reg:03
	for(j=0; j<4; j++)
	{
		WriteArr[j]=Bank1_Reg03[j];
	}
	SPI_Write_Ini_Buf(WRITE_REG, A_03, &(WriteArr[0]),4);
//	reg:04
	for(j=0; j<4; j++)
	{
		WriteArr[j]=Bank1_Reg04[j];
	}
	SPI_Write_Ini_Buf(WRITE_REG, A_04, &(WriteArr[0]),4);
//	reg:05
	for(j=0; j<4; j++)
	{
		WriteArr[j]=Bank1_Reg05[j];
	}
	SPI_Write_Ini_Buf(WRITE_REG, A_05, &(WriteArr[0]),4);
//	reg:0C
	for(j=0; j<4; j++)
	{
		WriteArr[j]=Bank1_Reg0C[j];
	}
	SPI_Write_Ini_Buf(WRITE_REG, A_0C, &(WriteArr[0]),4);
//	reg:0D
	for(j=0; j<4; j++)
	{
		WriteArr[j]=Bank1_Reg0D[j];
	}
	SPI_Write_Ini_Buf(WRITE_REG, A_0D, &(WriteArr[0]),4);
//	reg:0E
	for(j=0; j<11; j++)
	{
		WriteArr[j]=Bank1_Reg0E[j];
	}
	SPI_Write_Ini_Buf(WRITE_REG, A_0E, &(WriteArr[0]),11);
}
/**************************************************/   
void RFM70_Initialize(SpiPort *port)
{
	printf("%s\n", __FUNCTION__);
	g_SpiPort = *port;

	//delay_200ms();	//delay more than 50ms.
	HAL_Delay(200);
    Ini_Bank0();
	Ini_Bank1();	
	//delay_50ms();
	HAL_Delay(50);
	SwitchCFG(0);	//switch back to Bank0 register access
}

/**************************************************         
Function: DelayMs();                                  

Description:                                                
	delay ms,please implement this function according to your MCU.
**************************************************/
void DelayMs(int ms)
{


}
