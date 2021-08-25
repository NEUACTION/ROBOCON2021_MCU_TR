#include "APP.h"
#include "stdint.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "usart.h"
#include "movebase.h"

void SendData2BlueTooth(float Voltage,float paraVel,float paraEpos,float paraTpos)
{
  SendVoltageToApp(Voltage);
	SendVelToApp(paraVel);
	SendEEAngleToApp(paraEpos);
	SendTurnTableAngle2App(paraTpos);
}

void SendVoltageToApp(float input)
{
	union data_t{
		uint8_t data[4];
		float  value;
	}data;
	data.value= input;

	USART_SendData(APP_USART, 'A');
	
	USART_SendData(APP_USART, 'T');
	
	USART_SendData(APP_USART, 'E');
	
	USART_SendData(APP_USART, '1');
	
	USART_SendData(APP_USART, data.data[0]);
	
	USART_SendData(APP_USART, data.data[1]);
	
	USART_SendData(APP_USART, data.data[2]);
	
	USART_SendData(APP_USART, data.data[3]);
	
	USART_SendData(APP_USART, '\r');
	
	USART_SendData(APP_USART, '\n');
}


void SendTurnTableAngle2App(float input)
{
	union data_t{
		uint8_t data[4];
		float  value;
	}data;
	data.value=input;

	USART_SendData(APP_USART, 'A');
	
	USART_SendData(APP_USART, 'T');
	
	USART_SendData(APP_USART, 'D');
	
	USART_SendData(APP_USART, 'D');
	
	USART_SendData(APP_USART, data.data[0]);
	
	USART_SendData(APP_USART, data.data[1]);
	
	USART_SendData(APP_USART, data.data[2]);
	
	USART_SendData(APP_USART, data.data[3]);
	
	USART_SendData(APP_USART, '\r');
	
	USART_SendData(APP_USART, '\n');
}

void SendVelToApp(float input)
{
	union data_t{
		uint8_t data[4];
		float  value;
	}data;
	data.value= input;

	USART_SendData(APP_USART, 'A');
	
	USART_SendData(APP_USART, 'T');
	
	USART_SendData(APP_USART, 'V');
	
	USART_SendData(APP_USART, '1');
	
	USART_SendData(APP_USART, data.data[0]);
	
	USART_SendData(APP_USART, data.data[1]);
	
	USART_SendData(APP_USART, data.data[2]);
	
	USART_SendData(APP_USART, data.data[3]);
	
	USART_SendData(APP_USART, '\r');
	
	USART_SendData(APP_USART, '\n');
}

void SendEEAngleToApp(float input)
{
	union data_t{
		uint8_t data[4];
		float  value;
	}data;
	data.value=input;

	USART_SendData(APP_USART, 'A');
	
	USART_SendData(APP_USART, 'T');
	
	USART_SendData(APP_USART, 'A');
	
	USART_SendData(APP_USART, 'A');
	
	USART_SendData(APP_USART, data.data[0]);
	
	USART_SendData(APP_USART, data.data[1]);
	
	USART_SendData(APP_USART, data.data[2]);
	
	USART_SendData(APP_USART, data.data[3]);
	
	USART_SendData(APP_USART, '\r');
	
	USART_SendData(APP_USART, '\n');
}

void SendShootToApp(void)
{
	union data_t{
			uint8_t data[4];
			float  value;
		}data;
		data.value=1.00f;

	USART_SendData(APP_USART, 'A');
	
	USART_SendData(APP_USART, 'T');
	
	USART_SendData(APP_USART, 'O');

	USART_SendData(APP_USART, 'K');

	USART_SendData(APP_USART, data.data[0]);
	
	USART_SendData(APP_USART, data.data[1]);
	
	USART_SendData(APP_USART, data.data[2]);
	
	USART_SendData(APP_USART, data.data[3]);
	
	USART_SendData(APP_USART, '\r');
	
	USART_SendData(APP_USART, '\n');
}

void SendVisionAngleToApp(float input)
{
	union data_t{
		uint8_t data[4];
		float  value;
	}data;
	data.value=input;

	USART_SendData(APP_USART, 'A');
	
	USART_SendData(APP_USART, 'T');
	
	USART_SendData(APP_USART, 'A');
	
	USART_SendData(APP_USART, 'A');
	
	USART_SendData(APP_USART, data.data[0]);
	
	USART_SendData(APP_USART, data.data[1]);
	
	USART_SendData(APP_USART, data.data[2]);
	
	USART_SendData(APP_USART, data.data[3]);
	
	USART_SendData(APP_USART, '\r');
	
	USART_SendData(APP_USART, '\n');
}

void SendDistanceToApp(float input)
{
	union data_t{
		uint8_t data[4];
		float  value;
	}data;
	data.value=input;

	USART_SendData(APP_USART, 'A');
	
	USART_SendData(APP_USART, 'T');
	
	USART_SendData(APP_USART, 'D');
	
	USART_SendData(APP_USART, 'D');
	
	USART_SendData(APP_USART, data.data[0]);
	
	USART_SendData(APP_USART, data.data[1]);
	
	USART_SendData(APP_USART, data.data[2]);
	
	USART_SendData(APP_USART, data.data[3]);
	
	USART_SendData(APP_USART, '\r');
	
	USART_SendData(APP_USART, '\n');
}

void SendOKStartToApp(uint8_t input)
{ 
	USART_SendData(APP_USART, 'A');
	
	USART_SendData(APP_USART, 'T');
	
	USART_SendData(APP_USART, 'R');
	
	USART_SendData(APP_USART, 'B');
	
	USART_SendData(APP_USART, input);
	
	USART_SendData(APP_USART, '\r');
	
	USART_SendData(APP_USART, '\n');
}

//发送收到的桶号给APP
void SendPotLineToApp(uint8_t* input)
{
	USART_SendData(APP_USART, 'A');
	
	USART_SendData(APP_USART, 'T');
	
	USART_SendData(APP_USART, 'R');
	
	USART_SendData(APP_USART, 'C');
	
	USART_SendData(APP_USART, input[0]);
	
	USART_SendData(APP_USART, input[1]);
	
	USART_SendData(APP_USART, input[2]);
	
	USART_SendData(APP_USART, input[3]);
	
	USART_SendData(APP_USART, input[4]);
	
	USART_SendData(APP_USART, '\r');
	
	USART_SendData(APP_USART, '\n');
}
