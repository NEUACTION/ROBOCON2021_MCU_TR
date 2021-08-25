#include "teleController.h"
#include "string.h"

#define BT_USART UART4
#define WIFI_USART USART2

static char teleMsg_BT[MAX_TELECMD_BYTE_LENGTH] = {0};
static uint8_t teleCntCnt_BT = 0;
static uint8_t teleCntFlag_BT = 0;
static char teleMsg_WIFI[MAX_TELECMD_BYTE_LENGTH] = {0};
static uint8_t teleCntCnt_WIFI = 0;
static uint8_t teleCntFlag_WIFI = 0;

uint8_t teleController=0;
uint8_t teleClampClose=0;

//手柄字符存入
//先判断指令格式正确再判断指令内容正确 节省中断内运行时间
//BT
void TeleCmdBufferInput_BT(uint8_t ch)
{	
	static uint8_t count = 0;
	static uint8_t i=0;
	
	switch(count)
	{
		case 0:
		{
			if(ch=='A')
				count++;
			else
				count=0;
			break;
		}
		case 1:
		{
			if(ch=='T')
			{
				count++;
			}
			else
				count=0;
			break;
		}
		case 2:
		{
			if(ch=='+')
			{
				i=0;
				count++;
			}
			else
				count=0;
			break;
		}
		case 3:
		{
			teleMsg_BT[i]=ch;
			
			i++;
			
			if(i>=MAX_TELECMD_BYTE_LENGTH)
			{
				i=0;
				count++;
			}
			break;
		}
		case 4:
		{
			if(ch=='\r')
				count++;
			else
				count=0;
			break;
		}
		case 5:
		{
			if(ch=='\n')
			{
				//手柄消息处理BT
				TeleCmdPross_BT(teleMsg_BT);
			}
			count=0;
			break;
		}
		default:
			count=0;
		break;		 
	}
}

//WIFI
void TeleCmdBufferInput_WIFI(uint8_t ch)
{	
	static uint8_t count = 0;
	static uint8_t i=0;

	switch(count)
	{
		case 0:
		{
			if(ch=='A')
				count++;
			else
				count=0;
			break;
		}
		case 1:
		{
			if(ch=='T')
			{
				count++;
			}
			else
				count=0;
			break;
		}
		case 2:
		{
			if(ch=='+')
			{
				i=0;
				count++;
			}
			else
				count=0;
			break;
		}
		case 3:
		{
			teleMsg_WIFI[i]=ch;
			
			i++;
			
			if(i>=MAX_TELECMD_BYTE_LENGTH)
			{
				i=0;
				count++;
			}
			break;
		}
		case 4:
		{
			if(ch=='\r')
				count++;
			else
				count=0;
			break;
		}
		case 5:
		{
			if(ch=='\n')
			{
				//手柄消息处理WIFI
				TeleCmdPross_WIFI(teleMsg_WIFI);
			}
			count=0;
			break;
		}
		default:
			count=0;
		break;		 
	}
}

//手柄消息处理
//stop 直接停止 left/right 累计(限幅15.0f) next 置下一步标志位 
//接收到“AT+CNT\r\n”后 回复“AT+WF\r\n” "AT+BT\r\n"
void TeleCmdPross_BT(char* teleMsg)
{
	if(strncmp(&teleMsg[0],"NXT",3)==0 || strncmp(&teleMsg[0],"STP",3)==0 || strncmp(&teleMsg[0],"LFT",3)==0 || strncmp(&teleMsg[0],"RGT",3)==0 || strncmp(&teleMsg[0],"CNT",3)==0)
	{	
		if(strncmp(&teleMsg[0],"NXT",3)==0)
		{
			teleCntCnt_BT = 0;
			
			if(teleCntFlag_BT == 1 && teleController != STOP)
			{
				//轨迹多了继续添加，现在只是启动
				teleController++;
				
				if(teleController > 8)
					teleController=8;
				
				teleCntFlag_BT = 0;
							
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"BTNEXT!!\r\n");
				
				USART_OUT(BT_USART,(uint8_t *)"AT+FH\r\n");
			}
//			else
//			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"BTINN\r\n");
//			}
		}
		if(strncmp(&teleMsg[0],"STP",3)==0)
		{
			teleCntCnt_BT = 0;
			
			if(teleCntFlag_BT == 1)
			{
				//停止
				teleController=STOP;
				
				teleCntFlag_BT = 0;
				
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"BTSTOP!!\r\n");
				
				USART_OUT(BT_USART,(uint8_t *)"AT+FH\r\n");
			}
//			else
//			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"BTINS\r\n");
//			}
		}
		if(strncmp(&teleMsg[0],"LFT",3)==0)
		{
			teleCntCnt_BT = 0;
		
			if(teleCntFlag_BT == 1)
			{
				
				teleCntFlag_BT = 0;
				
				teleClampClose++;
				if(teleClampClose >= 4)
					teleClampClose=4;
				
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"BTLEFT!!\r\n");
				
				USART_OUT(BT_USART,(uint8_t *)"AT+FH\r\n");
			}
//			else
//			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"BTINL\r\n");
//			}
		}
		if(strncmp(&teleMsg[0],"RGT",3)==0)
		{
			teleCntCnt_BT = 0;
			
			if(teleCntFlag_BT == 1 )
			{
				//暂时没用到
				
				teleCntFlag_BT = 0;
				
	//			AngleShiftLimit();
				
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"BTRIGHT!!\r\n");
				
				USART_OUT(BT_USART,(uint8_t *)"AT+FH\r\n");
			}
//			else
//			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"BTINR\r\n");
//			}
		}
		if(strncmp(&teleMsg[0],"CNT",3)==0)
		{
			//连续2次收到CNT 开始下一次读取指令 即相邻两次按键时间间隔必须大于150*2ms！！！
			teleCntCnt_BT++;
			
			if(teleCntCnt_BT >= 2)
			{		
				teleCntFlag_BT = 1;
			}
					
	//		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
	//				(uint8_t *)"BTCNTCMD!!\r\n");
			
			USART_OUT(BT_USART,(uint8_t *)"AT+BT\r\n");
		}
	}
	else
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"BT%s ERROR TELECMD\r\n",teleMsg);
	}
}

void TeleCmdPross_WIFI(char* teleMsg)
{
	if(strncmp(&teleMsg[0],"NXT",3)==0 || strncmp(&teleMsg[0],"STP",3)==0 || strncmp(&teleMsg[0],"LFT",3)==0 || strncmp(&teleMsg[0],"RGT",3)==0 || strncmp(&teleMsg[0],"CNT",3)==0)
	{
		if(strncmp(&teleMsg[0],"NXT",3)==0)
		{
			teleCntCnt_WIFI = 0;
			
			if(teleCntFlag_WIFI == 1)
			{
				//接收信息，置位相应星信号
				
				
				//标志位清零
				teleCntFlag_WIFI = 0;
							
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"WFNEXT!!\r\n");
				
				USART_OUT(WIFI_USART,(uint8_t *)"AT+FH\r\n");
			}
//			else
//			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"WFINN\r\n");
//			}
		}
		if(strncmp(&teleMsg[0],"STP",3)==0)
		{
			teleCntCnt_WIFI = 0;
			
			if(teleCntFlag_WIFI == 1)
			{
				//接收信息，置位相应星信号
				
				
				//标志位清零
				teleCntFlag_WIFI = 0;
				
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"WFSTOP!!\r\n");
				
				USART_OUT(WIFI_USART,(uint8_t *)"AT+FH\r\n");
			}
//			else
//			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"WFINS\r\n");
//			}
		}
		if(strncmp(&teleMsg[0],"LFT",3)==0)
		{
			teleCntCnt_WIFI = 0;
			
			if(teleCntFlag_WIFI == 1)
			{
				//接收信息，置位相应星信号
				
				
				//标志位清零
				teleCntFlag_WIFI = 0;
				
				
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"WFLEFT!!\r\n");
				
				USART_OUT(WIFI_USART,(uint8_t *)"AT+FH\r\n");
			}
//			else
//			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"WFINL!\r\n");
//			}
		}
		if(strncmp(&teleMsg[0],"RGT",3)==0)
		{
			teleCntCnt_WIFI = 0;
			
			if(teleCntFlag_WIFI == 1)
			{
				//接收信息，置位相应星信号
				
				
				//标志位清零
				teleCntFlag_WIFI = 0;
				
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"WFRIGHT!!\r\n");
				
				USART_OUT(WIFI_USART,(uint8_t *)"AT+FH\r\n");
			}
//			else
//			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"WFINR\r\n");
//			}
		}
		if(strncmp(&teleMsg[0],"CNT",3)==0)
		{
			//连续2次收到CNT 开始下一次读取指令 即相邻两次按键时间间隔必须大于150*2ms！！！
			teleCntCnt_WIFI++;
			
			if(teleCntCnt_WIFI >= 2)
			{		
				teleCntFlag_WIFI = 1;
			}
			
	//		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
	//				(uint8_t *)"WIFICNTCMD!!\r\n");

			USART_OUT(WIFI_USART,(uint8_t *)"AT+WF\r\n");
		}
	}	
	else
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"WIFI%s ERROR TELECMD\r\n",teleMsg);
	}
}




void TeleInit(void)
{
	for(uint8_t i = 0;i < MAX_TELECMD_BYTE_LENGTH;i++)
	{
		teleMsg_BT[i] = 0;
		teleMsg_WIFI[i] = 0;
	}
	
	teleCntCnt_BT = 0;
	teleCntFlag_BT = 0;
	
	teleCntCnt_WIFI = 0;
	teleCntFlag_WIFI = 0;
}

void MsgReceive_BT(uint8_t ch)
{
	static uint8_t count = 0;
	static uint8_t i = 0;
	static char receData[2] = {0};
	switch(count)
	{
		case 0:
		{
			i = 0;
			if(ch=='A')
				count++;
			else
				count=0;
			break;
		}
		case 1:
		{
			
			if(ch=='T')
				count++;
			else
				count=0;
			break;
		}
		case 2:
		{
			receData[i] = ch;
			  i++;
 			if(i>=2)
			{
				i = 0;
				count++;
			}
			break;
		}
		case 3:
		{
			
			if(ch=='\r')
				count++;
			else
				count=0;
			break;
		}
		case 4:
		{
			if(ch=='\n')
			{
  				MsgProcess_BT(receData);
			}
			count = 0;
			break;
		}
	}
}
uint8_t trNextFlag = 0;
uint8_t trPressureFlag = 0;
uint8_t beepOnFlag = 0;
void MsgProcess_BT(char* data)
{

	if(strcmp(data,"PN") == 0)
					teleController = TELENEXT;
	if(strcmp(data,"PS") == 0)
					teleController = TELESTOP;
	if(strcmp(data,"CL") == 0)
					teleClampClose = TELECLOSE;
	if(strcmp(data,"BP") == 0)
		beepOnFlag = 1;
	if(strcmp(data,"TN") == 0)
		trNextFlag = TELETRNEXT;
	if(strcmp(data,"TP") == 0)
		trPressureFlag = 1;
	
	for(uint8_t i=0 ; i<2 ; i++)
		data[i]=0;

}


//void MsgJudge_BT(void)
//{
//	if(rHCMsg.walkStatus != waitForStart && 
//		 rHCMsg.walkStatus != load1stKickball && 
//		 rHCMsg.walkStatus != kick1stKickball && 
//		 rHCMsg.walkStatus != load2ndKickball && 
//		 rHCMsg.walkStatus != kick2ndKickball && 
//		 rHCMsg.walkStatus != Load3rdKickball && 
//		 rHCMsg.walkStatus != kick3rdKickball && 
//		 rHCMsg.walkStatus != Load4thKickball && 
//		 rHCMsg.walkStatus != kick4thKickball && 
//		 teleController != TELESTOP)
//	
//		teleController=TELENOCMD;
//	
//}

















