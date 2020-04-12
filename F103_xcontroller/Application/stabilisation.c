#include <stdio.h>
#include "main.h"

double a[3],w[3],Angle[3],T;
unsigned char data[11];

unsigned char *readFrame()
{
    int counter = 0;
    data[0] = 0;
    while(data[0] != 0x55){
        //int c = ::read(fd, data, 1);
    	HAL_UART_Receive(&huart3, data, 1, -1);
        if(counter > 20){
            //std::cout << std::endl;
            counter = 0;
        } else{
            counter++;
        }
        //std::cout << " " << std::hex << (int)data[0];
        //std::cout << " " << data[0];
    }

    //::read(fd, data + 1, sizeof(data) - 1);
    HAL_UART_Receive(&huart3, data + 1, sizeof(data) - 1, -1);

    return data;
}

void DecodeIMUData()
{
	int done = 0;

	while(!done){
		unsigned char *chrTemp = readFrame();

		switch(chrTemp[1])
		{
		case 0x51:
			a[0] = ((short)(chrTemp[3]<<8|chrTemp[2]))/32768.0*16;
			a[1] = ((short) (chrTemp[5]<<8|chrTemp[4]))/32768.0*16;
			a[2] = ((short) (chrTemp[7]<<8|chrTemp[6]))/32768.0*16;
			T = ((short) (chrTemp[9]<<8|chrTemp[8]))/340.0+36.25;
		break;
		case 0x52:
			w[0] = ((short) (chrTemp[3]<<8|chrTemp[2]))/32768.0*2000;
			w[1] = ((short) (chrTemp[5]<<8|chrTemp[4]))/32768.0*2000;
			w[2] = ((short) (chrTemp[7]<<8|chrTemp[6]))/32768.0*2000;
			T = ((short) (chrTemp[9]<<8|chrTemp[8]))/340.0+36.25;
		break;
		case 0x53:
			Angle[0] = ((short) (chrTemp[3]<<8|chrTemp[2]))/32768.0*180;
			Angle[1] = ((short) (chrTemp[5]<<8|chrTemp[4]))/32768.0*180;
			Angle[2] = ((short)(chrTemp[7]<<8|chrTemp[6]))/32768.0*180;
			T = ((short)(chrTemp[9]<<8|chrTemp[8]))/340.0+36.25;

			printf("%04.3f %04.3f %04.3f\n",a[0],a[1],a[2]);
			//printf("%04.3f %04.3f %04.3f\n",w[0],w[1],w[2]);
			//printf("%04.03f %04.03f %04.03f\n",Angle[0],Angle[1],Angle[2]);

//			printf("%04.03f %04.03f %04.03f %04.03f %04.03f %04.03f %04.02f %04.02f %04.02f\n",
//					a[0],a[1],a[2],
//					w[0],w[1],w[2],
//					Angle[0],Angle[1],Angle[2]);
			done = 1;
		break;
		}
	}
}

//HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);


//extern "C"{

void printStabilisation(){
	DecodeIMUData();
}

//}
