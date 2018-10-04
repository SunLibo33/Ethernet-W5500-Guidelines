/**********************************************************************************
 * ???  :W5500.c
 * ??    :W5500 ?????         

 * ??    :http://yixindianzikeji.taobao.com/
**********************************************************************************/
		
#include "W5500.h"	

/***************----- ???????? -----***************/
unsigned char Gateway_IP[4];//??IP?? 
unsigned char Sub_Mask[4];	//???? 
unsigned char Phy_Addr[6];	//????(MAC) 
unsigned char IP_Addr[4];	//??IP?? 

unsigned char S0_Port[2];	//??0????
unsigned char S0_DIP[4];	//??0??IP?? 
unsigned char S0_DPort[2];	//??0?????

unsigned char UDP_DIPR[4];	//UDP(??)??,????IP??
unsigned char UDP_DPORT[2];	//UDP(??)??,???????

/***************----- ??????? -----***************/
unsigned char S0_Mode =3;	//??0?????,0:TCP?????,1:TCP?????,2:UDP(??)??

#define TCP_SERVER	0x00	//TCP?????
#define TCP_CLIENT	0x01	//TCP????? 
#define UDP_MODE	0x02	//UDP(??)?? 

/***************----- ??????? -----***************/
unsigned char S0_State =0;	//??0????,1:???????,2??????(????????) 
#define S_INIT		0x01	//??????? 
#define S_CONN		0x02	//??????,???????? 

/***************----- ????????? -----***************/
unsigned char S0_Data;		//??0??????????,1:???????,2:???????? 
#define S_RECEIVE	 0x01	//?????????? 
#define S_TRANSMITOK 0x02	//??????????? 

/***************----- ??????? -----***************/
unsigned char Rx_Buffer[30];	//????????? 
unsigned char Tx_Buffer[30];	//????????? 

unsigned char W5500_Interrupt;	//W5500????(0:???,1:???)
extern unsigned int ETH_Receive_Data_Number;
unsigned int Receive_ETH_Data(SOCKET s);
/*******************************************************************************
* ???  : SPI_ReadByte
* ??    : ??????SPI?????
* ??    : ?
* ??    : ?
* ???  : ?????????
* ??    : ?
*******************************************************************************/
unsigned char SPI_Read_Byte(void)
{
	unsigned char i,rByte=0;
	
	W5500_SCLK_L;
	for(i=0;i<8;i++)
	{
		W5500_SCLK_H;
		rByte<<=1;
		if( ((GPIOE->IDR)&(0x0010)) == (0x0010) ) rByte |= 0x01;
		W5500_SCLK_L;
	}
	return rByte;
}

/*******************************************************************************
* ???  : SPI_SendByte
* ??    : SPI????????
* ??    : dt:??????
* ??    : ?
* ???  : ?
* ??    : ?
*******************************************************************************/
void SPI_Send_Byte(unsigned char dt)
{
	unsigned char i;
		
	for(i=0;i<8;i++)
	{	
		W5500_SCLK_L;
		if((dt<<i)&0x80)
			W5500_MOSI_H;
		else
			W5500_MOSI_L;					
		W5500_SCLK_H;
	}
	W5500_SCLK_L;
}

/*******************************************************************************
* ???  : SPI_Send_Short
* ??    : SPI??2?????(16?)
* ??    : dat:????16???
* ??    : ?
* ???  : ?
* ??    : ?
*******************************************************************************/
void SPI_Send_Short(unsigned short dt)
{
	SPI_Send_Byte((unsigned char)(dt/256));	//?????
	SPI_Send_Byte(dt);						//?????
}

/*******************************************************************************
* ???  : Write_W5500_1Byte
* ??    : ??SPI?????????1?????
* ??    : reg:16??????,dat:??????
* ??    : ?
* ???  : ?
* ??    : ?
*******************************************************************************/
void Write_W5500_1Byte(unsigned short reg, unsigned char dat)
{
	W5500_SCS_L;

	SPI_Send_Short(reg);//??SPI?16??????
	SPI_Send_Byte(FDM1|RWB_WRITE|COMMON_R);//??SPI?????,1???????,???,???????
	SPI_Send_Byte(dat);//?1?????

	W5500_SCS_H;
}

/*******************************************************************************
* ???  : Write_W5500_2Byte
* ??    : ??SPI?????????2?????
* ??    : reg:16??????,dat:16???????(2???)
* ??    : ?
* ???  : ?
* ??    : ?
*******************************************************************************/
void Write_W5500_2Byte(unsigned short reg, unsigned short dat)
{
	W5500_SCS_L;
		
	SPI_Send_Short(reg);//??SPI?16??????
	SPI_Send_Byte(FDM2|RWB_WRITE|COMMON_R);//??SPI?????,2???????,???,???????
	SPI_Send_Short(dat);//?16???

	W5500_SCS_H;
}

/*******************************************************************************
* ???  : Write_W5500_nByte
* ??    : ??SPI?????????n?????
* ??    : reg:16??????,*dat_ptr:??????????,size:????????
* ??    : ?
* ???  : ?
* ??    : ?
*******************************************************************************/
void Write_W5500_nByte(unsigned short reg, unsigned char *dat_ptr, unsigned short size)
{
	unsigned short i;

	W5500_SCS_L;	
		
	SPI_Send_Short(reg); 
	SPI_Send_Byte(VDM|RWB_WRITE|COMMON_R); 

	for(i=0;i<size;i++)  
	{
		SPI_Send_Byte(*dat_ptr++); 
	}

	W5500_SCS_H;
}

/*******************************************************************************
* ???  : Write_W5500_SOCK_1Byte
* ??    : ??SPI?????????1?????
* ??    : s:???,reg:16??????,dat:??????
* ??    : ?
* ???  : ?
* ??    : ?
*******************************************************************************/
void Write_W5500_SOCK_1Byte(SOCKET s, unsigned short reg, unsigned char dat)
{
	W5500_SCS_L;
		
	SPI_Send_Short(reg);//??SPI?16??????
	SPI_Send_Byte(FDM1|RWB_WRITE|(s*0x20+0x08));//??SPI?????,1???????,???,????s????
	SPI_Send_Byte(dat);//?1?????

	W5500_SCS_H;
}

/*******************************************************************************
* ???  : Write_W5500_SOCK_2Byte
* ??    : ??SPI?????????2?????
* ??    : s:???,reg:16??????,dat:16???????(2???)
* ??    : ?
* ???  : ?
* ??    : ?
*******************************************************************************/
void Write_W5500_SOCK_2Byte(SOCKET s, unsigned short reg, unsigned short dat)
{
	W5500_SCS_L;
			
	SPI_Send_Short(reg);//??SPI?16??????
	SPI_Send_Byte(FDM2|RWB_WRITE|(s*0x20+0x08));//??SPI?????,2???????,???,????s????
	SPI_Send_Short(dat);//?16???

	W5500_SCS_H;
}

/*******************************************************************************
* ???  : Write_W5500_SOCK_4Byte
* ??    : ??SPI?????????4?????
* ??    : s:???,reg:16??????,*dat_ptr:????4????????
* ??    : ?
* ???  : ?
* ??    : ?
*******************************************************************************/
void Write_W5500_SOCK_4Byte(SOCKET s, unsigned short reg, unsigned char *dat_ptr)
{
	W5500_SCS_L;
			
	SPI_Send_Short(reg);//??SPI?16??????
	SPI_Send_Byte(FDM4|RWB_WRITE|(s*0x20+0x08));//??SPI?????,4???????,???,????s????

	SPI_Send_Byte(*dat_ptr++);//??1?????
	SPI_Send_Byte(*dat_ptr++);//??2?????
	SPI_Send_Byte(*dat_ptr++);//??3?????
	SPI_Send_Byte(*dat_ptr++);//??4?????

	W5500_SCS_H;
}

/*******************************************************************************
* ???  : Read_W5500_1Byte
* ??    : ?W5500????????1?????
* ??    : reg:16??????
* ??    : ?
* ???  : ???????1?????
* ??    : ?
*******************************************************************************/
unsigned char Read_W5500_1Byte(unsigned short reg)
{
	unsigned char i;

	W5500_SCS_L;
			
	SPI_Send_Short(reg);//??SPI?16??????
	SPI_Send_Byte(FDM1|RWB_READ|COMMON_R);//??SPI?????,1???????,???,???????
	i=SPI_Read_Byte();

	W5500_SCS_H;
	return i;//???????????
}

/*******************************************************************************
* ???  : Read_W5500_SOCK_1Byte
* ??    : ?W5500????????1?????
* ??    : s:???,reg:16??????
* ??    : ?
* ???  : ???????1?????
* ??    : ?
*******************************************************************************/
unsigned char Read_W5500_SOCK_1Byte(SOCKET s, unsigned short reg)
{
	unsigned char i;

	W5500_SCS_L;
			
	SPI_Send_Short(reg);//??SPI?16??????
	SPI_Send_Byte(FDM1|RWB_READ|(s*0x20+0x08));//??SPI?????,1???????,???,????s????
	i=SPI_Read_Byte();

	W5500_SCS_H;
	return i;//???????????
}

/*******************************************************************************
* ???  : Read_W5500_SOCK_2Byte
* ??    : ?W5500????????2?????
* ??    : s:???,reg:16??????
* ??    : ?
* ???  : ???????2?????(16?)
* ??    : ?
*******************************************************************************/
unsigned short Read_W5500_SOCK_2Byte(SOCKET s, unsigned short reg)
{
	unsigned short i;

	W5500_SCS_L;
			
	SPI_Send_Short(reg);//??SPI?16??????
	SPI_Send_Byte(FDM2|RWB_READ|(s*0x20+0x08));//??SPI?????,2???????,???,????s????

	i=SPI_Read_Byte();
	i*=256;
	i+=SPI_Read_Byte();//??????

	W5500_SCS_H;
	return i;//???????????
}

/*******************************************************************************
* ???  : Read_SOCK_Data_Buffer
* ??    : ?W5500????????????
* ??    : s:???,*dat_ptr:?????????
* ??    : ?
* ???  : ????????,rx_size???
* ??    : ?
*******************************************************************************/
unsigned short Read_SOCK_Data_Buffer(SOCKET s, unsigned char *dat_ptr)
{
	unsigned short rx_size;
	unsigned short offset, offset1;
	unsigned short i;
	unsigned char j;

	rx_size=Read_W5500_SOCK_2Byte(s,Sn_RX_RSR);
	if(rx_size==0) return 0;//?????????
	if(rx_size>1460) rx_size=1460;

	offset=Read_W5500_SOCK_2Byte(s,Sn_RX_RD);
	offset1=offset;
	offset&=(S_RX_SIZE-1);//?????????

	W5500_SCS_L;

	SPI_Send_Short(offset);//?16???
	SPI_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//?????,N???????,???,????s????
	
	if((offset+rx_size)<S_RX_SIZE)//?????????W5500?????????????
	{
		for(i=0;i<rx_size;i++)//????rx_size?????
		{
			j=SPI_Read_Byte();//??1?????
			*dat_ptr=j;//?????????????????
			dat_ptr++;//?????????????1
		}
	}
	else//????????W5500?????????????
	{
		offset=S_RX_SIZE-offset;
		for(i=0;i<offset;i++)//??????offset?????
		{
			j=SPI_Read_Byte();//??1?????
			*dat_ptr=j;//?????????????????
			dat_ptr++;//?????????????1
		}
		W5500_SCS_H;

		W5500_SCS_L;

		SPI_Send_Short(0x00);//?16???
		SPI_Send_Byte(VDM|RWB_READ|(s*0x20+0x18));//?????,N???????,???,????s????

		for(;i<rx_size;i++)//?????rx_size-offset?????
		{
			j=SPI_Read_Byte();//??1?????
			*dat_ptr=j;//?????????????????
			dat_ptr++;//?????????????1
		}
	}
	W5500_SCS_H;

	offset1+=rx_size;//????????,????????????????
	Write_W5500_SOCK_2Byte(s, Sn_RX_RD, offset1);
	Write_W5500_SOCK_1Byte(s, Sn_CR, RECV);//????????
	return rx_size;//??????????
}

/*******************************************************************************
* ???  : Write_SOCK_Data_Buffer
* ??    : ?????W5500????????
* ??    : s:???,*dat_ptr:?????????,size:????????
* ??    : ?
* ???  : ?
* ??    : ?
*******************************************************************************/
void Write_SOCK_Data_Buffer(SOCKET s, unsigned char *dat_ptr, unsigned short size)
{
	unsigned short offset,offset1;
	unsigned short i;

	//???UDP??,???????????IP????
	if((Read_W5500_SOCK_1Byte(s,Sn_MR)&0x0f) != SOCK_UDP)//??Socket????
	{		
		Write_W5500_SOCK_4Byte(s, Sn_DIPR, UDP_DIPR);//??????IP  		
		Write_W5500_SOCK_2Byte(s, Sn_DPORTR, UDP_DPORT[0]*256+UDP_DPORT[1]);//?????????				
	}

	offset=Read_W5500_SOCK_2Byte(s,Sn_TX_WR);
	offset1=offset;
	offset&=(S_TX_SIZE-1);//?????????

	W5500_SCS_L;

	SPI_Send_Short(offset);//?16???
	SPI_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//?????,N???????,???,????s????

	if((offset+size)<S_TX_SIZE)//?????????W5500?????????????
	{
		for(i=0;i<size;i++)//????size?????
		{
			SPI_Send_Byte(*dat_ptr++);//?????????		
		}
	}
	else//????????W5500?????????????
	{
		offset=S_TX_SIZE-offset;
		for(i=0;i<offset;i++)//?????offset?????
		{
			SPI_Send_Byte(*dat_ptr++);//?????????
		}
		W5500_SCS_H;

		W5500_SCS_L;

		SPI_Send_Short(0x00);//?16???
		SPI_Send_Byte(VDM|RWB_WRITE|(s*0x20+0x10));//?????,N???????,???,????s????

		for(;i<size;i++)//????size-offset?????
		{
			SPI_Send_Byte(*dat_ptr++);//?????????
		}
	}
	W5500_SCS_H;

	offset1+=size; 
	Write_W5500_SOCK_2Byte(s, Sn_TX_WR, offset1);
	Write_W5500_SOCK_1Byte(s, Sn_CR, SEND);//????????				
}

/*******************************************************************************
* ???  : W5500_Hardware_Reset
* ??    : ????W5500
* ??    : ?
* ??    : ?
* ???  : ?
* ??    : W5500????????????500us??,????W5500
*******************************************************************************/
void W5500_Hardware_Reset(void)
{
	W5500_RESET_L;
	delay_ms(200);
	W5500_RESET_H;
	delay_ms(200);
	while((Read_W5500_1Byte(PHYCFGR)&LINK)==0); 
}

/*******************************************************************************
* ???  : W5500_Init
* ??    : ???W5500?????
* ??    : ?
* ??    : ?
* ???  : ?
* ??    : ???W5500??,??W5500???
*******************************************************************************/
void W5500_Init(void)
{
	unsigned char i=0;

	Write_W5500_1Byte(MR, RST);//????W5500,?1??,??????0
	delay_ms(10);//??10ms,???????

	//????(Gateway)?IP??,Gateway_IP?4??unsigned char??,???? 
	//????????????????,????????????????Internet
	Write_W5500_nByte(GAR, Gateway_IP, 4);
			
	//??????(MASK)?,SUB_MASK?4??unsigned char??,????
	//??????????
	Write_W5500_nByte(SUBR,Sub_Mask,4);		
	
	//??????,PHY_ADDR?6??unsigned char??,????,????????????????
	//???????IEEE??,??OUI???,?3????????,??????????
	//??????????,????????????
	Write_W5500_nByte(SHAR,Phy_Addr,6);		

	//?????IP??,IP_ADDR?4??unsigned char??,????
	//??,??IP?????IP???????,???????????
	Write_W5500_nByte(SIPR,IP_Addr,4);		
	
	//????????????????,??W5500????
	for(i=0;i<8;i++)
	{
		Write_W5500_SOCK_1Byte(i,Sn_RXBUF_SIZE, 0x02);//Socket Rx memory size=2k
		Write_W5500_SOCK_1Byte(i,Sn_TXBUF_SIZE, 0x02);//Socket Tx mempry size=2k
	}

	//??????,???2000(200ms) 
	//???????100??,???????2000(0x07D0),??200??
	
	Write_W5500_1Byte(0x0013, 0x01);
	Write_W5500_1Byte(0x0014, 0x01);
	//Write_W5500_1Byte(0x0016, 0xF0);
  Write_W5500_1Byte(0x0018, 0x01);
	
	Write_W5500_2Byte(RTR, 0x07d0);

	//??????,???8? 
	//????????????,???????(????????????Sn_IR ???(TIMEOUT)?�1�)
	Write_W5500_1Byte(RCR,8);
		
//	while(1)
//	{
//	 Read_W5500_1Byte(0X0002); 0xA8
//	}	
	
}

/*******************************************************************************
* ???  : Detect_Gateway
* ??    : ???????
* ??    : ?
* ??    : ?
* ???  : ????TRUE(0xFF),????FALSE(0x00)
* ??    : ?
*******************************************************************************/
unsigned char Detect_Gateway(void)
{
	unsigned char ip_adde[4];
	ip_adde[0]=IP_Addr[0]+1;
	ip_adde[1]=IP_Addr[1]+1;
	ip_adde[2]=IP_Addr[2]+1;
	ip_adde[3]=IP_Addr[3]+1;

	//??????????????
	Write_W5500_SOCK_4Byte(0,Sn_DIPR,ip_adde);//?????????????IP???IP?
	Write_W5500_SOCK_1Byte(0,Sn_MR,MR_TCP);//??socket?TCP??
	Write_W5500_SOCK_1Byte(0,Sn_CR,OPEN);//??Socket	
	delay_ms(5);//??5ms 	
	
	if(Read_W5500_SOCK_1Byte(0,Sn_SR) != SOCK_INIT)//??socket????
	{
		Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//?????,??Socket
		return FALSE;//??FALSE(0x00)
	}

	Write_W5500_SOCK_1Byte(0,Sn_CR,CONNECT);//??Socket?Connect??						

	do
	{
		unsigned char j=0;
		j=Read_W5500_SOCK_1Byte(0,Sn_IR);//??Socket0???????
		if(j!=0)
		Write_W5500_SOCK_1Byte(0,Sn_IR,j);
		delay_ms(5);//??5ms 
		if((j&IR_TIMEOUT) == IR_TIMEOUT)
		{
			return FALSE;	
		}
		else if(Read_W5500_SOCK_1Byte(0,Sn_DHAR) != 0xff)
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//??Socket
			return TRUE;							
		}
	}while(1);
}

/*******************************************************************************
* ???  : Socket_Init
* ??    : ??Socket(0~7)???
* ??    : s:???????
* ??    : ?
* ???  : ?
* ??    : ?
*******************************************************************************/
void Socket_Init(SOCKET s)
{
	//??????,??W5500????,???????	
	Write_W5500_SOCK_2Byte(0, Sn_MSSR, 30);//???????=30(0x001e)
	//??????
	switch(s)
	{
		case 0:
			//????0????
			Write_W5500_SOCK_2Byte(0, Sn_PORT, S0_Port[0]*256+S0_Port[1]);	
			
			break;

		case 1:
			break;

		case 2:
			break;

		case 3:
			break;

		case 4:
			break;

		case 5:
			break;

		case 6:
			break;

		case 7:
			break;

		default:
			break;
	}
}

/*******************************************************************************
* ???  : Socket_Connect
* ??    : ????Socket(0~7)????????????
* ??    : s:??????
* ??    : ?
* ???  : ????TRUE(0xFF),????FALSE(0x00)
* ??    : ???Socket?????????,?????,??????????
*			?????????????,?????????,???????????
*			????????,???????????
*******************************************************************************/
unsigned char Socket_Connect(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_TCP);//??socket?TCP??
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//??Socket
	delay_ms(5);//??5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_INIT)//??socket????
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//?????,??Socket
		return FALSE;//??FALSE(0x00)
	}
	Write_W5500_SOCK_1Byte(s,Sn_CR,CONNECT);//??Socket?Connect??
	return TRUE;//??TRUE,????
}

/*******************************************************************************
* ???  : Socket_Listen
* ??    : ????Socket(0~7)??????????????
* ??    : s:??????
* ??    : ?
* ???  : ????TRUE(0xFF),????FALSE(0x00)
* ??    : ???Socket?????????,?????,?????????
*			????????,??W5500????????
*******************************************************************************/
unsigned char Socket_Listen(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_TCP);//??socket?TCP?? 
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//??Socket	
	delay_ms(5);//??5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_INIT)//??socket????
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//?????,??Socket
		return FALSE;//??FALSE(0x00)
	}	
	Write_W5500_SOCK_1Byte(s,Sn_CR,LISTEN);//??Socket?????	
	delay_ms(5);//??5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_LISTEN)//??socket????
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//?????,??Socket
		return FALSE;//??FALSE(0x00)
	}

	return TRUE;

	//?????Socket??????????,???????????????,?????Socket??,
	//???Socket??????????W5500?????Socket????
	//???????????????IP??????
}

/*******************************************************************************
* ???  : Socket_UDP
* ??    : ????Socket(0~7)?UDP??
* ??    : s:??????
* ??    : ?
* ???  : ????TRUE(0xFF),????FALSE(0x00)
* ??    : ??Socket???UDP??,?????,?UDP???,Socket?????????
*			????????,??W5500???UDP??
*******************************************************************************/
unsigned char Socket_UDP(SOCKET s)
{
	Write_W5500_SOCK_1Byte(s,Sn_MR,MR_UDP);//??Socket?UDP??*/
	Write_W5500_SOCK_1Byte(s,Sn_CR,OPEN);//??Socket*/
	delay_ms(5);//??5ms
	if(Read_W5500_SOCK_1Byte(s,Sn_SR)!=SOCK_UDP)//??Socket????
	{
		Write_W5500_SOCK_1Byte(s,Sn_CR,CLOSE);//?????,??Socket
		return FALSE;//??FALSE(0x00)
	}
	else
		return TRUE;

	//?????Socket????UDP????,???????????????????
	//??Socket???????,?????????????????IP???Socket????
	//??????IP???Socket????????,??????????,??????????
}

/*******************************************************************************
* ???  : W5500_Interrupt_Process
* ??    : W5500????????
* ??    : ?
* ??    : ?
* ???  : ?
* ??    : ?
*******************************************************************************/
void W5500_Interrupt_Process(void)
{
	unsigned char i,j;

IntDispose:

	i=Read_W5500_1Byte(SIR);//???????????	
	if((i & S0_INT) == S0_INT)//Socket0???? 
	{
		j=Read_W5500_SOCK_1Byte(0,Sn_IR);//??Socket0???????
		Write_W5500_SOCK_1Byte(0,Sn_IR,j);
		if(j&IR_CON)//?TCP???,Socket0???? 
		{
			S0_State|=S_CONN;//??????0x02,??????,????????
		}
		if(j&IR_DISCON)//?TCP???Socket??????
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);//????,???????? 
			Socket_Init(0);		//??Socket(0~7)???,?????0
			S0_State=0;//??????0x00,??????
		}
		if(j&IR_SEND_OK)//Socket0??????,??????S_tx_process()?????? 
		{
			S0_Data|=S_TRANSMITOK;//??????????? 
			Write_SOCK_Data_Buffer(0, Rx_Buffer, ETH_Receive_Data_Number);
		}
		if(j&IR_RECV)//Socket?????,????S_rx_process()?? 
		{
			S0_Data|=S_RECEIVE;//??????????
			ETH_Receive_Data_Number=Receive_ETH_Data(0);
			Write_SOCK_Data_Buffer(0, Rx_Buffer, ETH_Receive_Data_Number);
			// As soon as STM32 has received information from the other termination.  
      // STM32 W5500 would send the whole received information to the other termination. 
      // By Sun Libo Tel and WeChat : 15889672958			
		}
		if(j&IR_TIMEOUT)//Socket??????????? 
		{
			Write_W5500_SOCK_1Byte(0,Sn_CR,CLOSE);// ????,???????? 			
			S0_State=0;//??????0x00,??????
		}
	}

	if(Read_W5500_1Byte(SIR) != 0) 
		goto IntDispose;
}


unsigned int Receive_ETH_Data(SOCKET s)
{
	unsigned int size=8;
	unsigned int DataShift_i=0;
	size=Read_SOCK_Data_Buffer(s, Rx_Buffer);
	UDP_DIPR[0] = Rx_Buffer[0];
	UDP_DIPR[1] = Rx_Buffer[1];
	UDP_DIPR[2] = Rx_Buffer[2];
	UDP_DIPR[3] = Rx_Buffer[3];

	UDP_DPORT[0] = Rx_Buffer[4];
	UDP_DPORT[1] = Rx_Buffer[5];  

	for(DataShift_i=0;DataShift_i<size-8;DataShift_i++)
	  {
			Rx_Buffer[DataShift_i]=Rx_Buffer[DataShift_i+8];	
		}
	return (size-8);	
}
