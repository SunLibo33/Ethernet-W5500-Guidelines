#include "sys.h"
#include "delay.h" 
#include "usart.h"
#include "math.h"
#include "W5500.h"

		
#include <string.h>


// Main function + Interrupt function + Register Configuration
// Main function + Register Configuration

// clksel      [7..0]   0:2m  1:4m  2:8m  3:0.5m  4:0.25m  OK
// setcntr     [23..0]  累加次数                           OK
// setaddr     [15..0]  采样点数                           OK
// setpls      [15..0]  脉冲宽度                           OK
// setlazersel [7..0]   激光器功率                         NA
// Gain        [7..0]   增益控制                           NA
// comarrival  Start One Test Circle


const unsigned int  SamplePoint    =  12000 ;
const unsigned char NeedCount      =     15 ;
unsigned char       SystemCount    =      0 ;
unsigned char       OutOfOTDRRange =      1 ;

unsigned char  ETH_Interrupt_Count      = 0x00;
unsigned int   ETH_Receive_Data_Number  = 0x0000;

unsigned char SPI_Command[10] = { 0x00, 
                                  0x00, 0x04, 0x00, 
							   	                SamplePoint%256, SamplePoint/256,
							                    0x00, 0x01, 
								                  0x0A,
								                  0x50
								                };

unsigned long Optical_Data[15000];
unsigned long ForAnalogSignalBase;
float         ForAnalogSignalBaseF;
unsigned long DataPoint_H3,DataPoint_H2,DataPoint_H1;
unsigned long DataPoint;
unsigned char dBShowForUSART[6];
unsigned char dBShowDataCnt;

unsigned char StartOneTestCircleFlag=0x00;
unsigned char ErrorFlag=0x00;

unsigned char UARTReceiveData  ;
void USARTSend(unsigned char SendData);
unsigned char SPI_WR(unsigned char SendData);
void ResetFPGA(void);
void StartOneTestCircle(void);


unsigned long DataCache[32];
unsigned long DataCacheFIR;
unsigned int  FIRCNT;



void Dealy(unsigned int Cnt)
{
    unsigned int m,n;
	  for(m=0;m<Cnt;m++)
	   {
	     for(n=0;n<4000;n++)
		  ;
	   }
}

void USARTSend(unsigned char SendData)
{
   while (	 ( USART1->SR  &	(1<<6) ) == 0x00000000	 ) ;
   USART1->DR = SendData  ;
   USART1->SR &= 0xDF;
}

void SPIInit()
{
	RCC->AHB1ENR|=1<<0;   	//使能PORTA口时钟  
	RCC->APB2ENR|=1<<12;
	GPIOA->MODER &= 0XFFFF00FF;
	GPIOA->MODER |= 0X0000AA00;
	GPIOA->OSPEEDR &= 0XFFFF00FF;
	GPIOA->OSPEEDR |= 0X0000AA00;
	GPIOA->AFR[0] &= 0X0000FFFF;
	GPIOA->AFR[0] |= 0X55550000;

	GPIOA->MODER &= 0XFFFFFFF3;	//PA1 CS功能
  GPIOA->MODER |= 0X00000004;	//PA1 CS功能
  GPIOA->OSPEEDR &= 0XFFFFFFF3;//PA1 speed=50MHz
	GPIOA->OSPEEDR |= 0X0000000B;//PA1 speed=50MHz
	
	GPIOA->MODER &= 0XFFFFFFCF;	//PA2 CRC Check功能
  GPIOA->MODER |= 0X00000000;	//PA2 CRC Check功能	
	GPIOA->MODER &= 0XFFFFFF3F;	//PA3 Reset FPGA功能
  GPIOA->MODER |= 0X00000040;	//PA3 Reset FPGA功能
	GPIOA->OSPEEDR &= 0XFFFFFF3F;//PA3 speed=50MHz
	GPIOA->OSPEEDR |= 0X00000080;//PA3 speed=50MHz
	GPIOA->ODR &= 0x0000fff7;//PA3 Reset FPGA Enable
	

	SPI1->CR1 = 0x431C; //0X4324 : fPCLK/32 MHz	 0x431C : fPCLK/16 MHz  0X4334 : fPCLK/128 MHz	0X432C : fPCLK/64 MHz	 [fPCLK=84MHz STM32F407]
	SPI1->CR1 |= 0X0040;  //enable SPI Function
 
}


void PortDInit()
{
	RCC->AHB1ENR|=1<<3;   	//使能PORTD口时钟  
  //PD15 TestDataGetReady功能For Check
	GPIOD->MODER &= 0XFFFFFF0F;
  GPIOD->MODER |= 0X00000050;  
  GPIOD->OSPEEDR |= 0XAAAAAAAA; // speed=50MHz
}

void PortEInit()
{
	RCC->AHB1ENR|=1<<4;   	//使能PORTE口时钟  
  GPIOE->MODER &= 0XFFFFFFF0;
  GPIOE->MODER |= 0X00000005; //PE0 PE1 Output , PE0 ETH_CS and PE1 ETH_CLK
	
	GPIOE->MODER &= 0XFFFFFF0F;
  GPIOE->MODER |= 0X00000010; //PE2 PE3 Output , PE2 ETH_MOSI and PE3 ETH_INT
	
	GPIOE->MODER &= 0XFFFFF0FF;
  GPIOE->MODER |= 0X00000400; //PE4 PE5 Input , PE4 ETH_MISO and PE5 ETH_Reset
	
	GPIOE->OSPEEDR |= 0XAAAAAAAA; // speed=50MHz
	
//************* ETH External Interrupt	
	RCC->APB2ENR |= 1 << 14;         //Enable SYSCFG Clock 
	GPIOE->PUPDR |= 0X00000000;      //PE3 Up pull to VCC3.3
	SYSCFG->EXTICR[0] = 0x00004000; //Configuration the relationship between IO and Interruption wire
	
	EXTI->IMR  |= 1 << 3;  
  EXTI->EMR  |= 1 << 3;
  EXTI->FTSR |= 1 << 3;  
	
	SCB->AIRCR = 0x05AF0000 | 0x500; //|= 0x5 << 8;  
  NVIC->IP[9] |= 0xe0;       
  NVIC->ISER[0] |= 1 << 9;  
//*************	ETH External Interrupt	
	
	W5500_SCS_H;
	W5500_SCLK_H;
	W5500_RESET_H;
	
	
}

unsigned char SPI_WR(unsigned char SendData)
{
	GPIOA->ODR |= 0x00000002; 
	GPIOA->ODR &= 0xffffffFD;
	SPI1->DR= SendData ;
	while((SPI1->SR&1<<7)==0x0080) ;
	while((SPI1->SR&1<<7)==0x0080) ;
	GPIOA->ODR &= 0xffffffFD;
	GPIOA->ODR |= 0x00000002; 
	while((SPI1->SR&1<<0)==0) ;
	return SPI1->DR ;
}

void ResetFPGA(void)
{
	GPIOA->ODR &= 0x0000fff7;//PA3 Reset FPGA Enable
	Dealy(10);
	GPIOA->ODR |= 0x00000008;//PA3 Reset FPGA Disable
}

void StartOneTestCircle(void)
{
   unsigned char m,n,CRCCheckData;
   ResetFPGA(); 
   for(n=0;n<7;n++)
   {
     {
       ResetFPGA(); 
       Dealy(20);
       SPI_WR(0xAA);Dealy(2);
       SPI_WR(0xBB);Dealy(2);
       SPI_WR(0xCC);Dealy(2);
       SPI_WR(0xDD);Dealy(2);
       CRCCheckData=0x00;
       for(m=0;m<10;m++)
         {
           SPI_WR(SPI_Command[m]);
	         Dealy(2);
	         CRCCheckData = CRCCheckData ^ SPI_Command[m] ;
         }
				 
			 SPI_WR(CRCCheckData);		 
       Dealy(2);
       if  ( ((GPIOA->IDR)&(0x0004)) == (0x0004) ) //判断测试命令是否发送成功
       break;
     }
     SPI_WR(0xBB);
   }
   if(n==0x07) //7次命令下发，全部失败！
   {
    USARTSend(0xee);
	  ErrorFlag = 0x01;
   }
   else      //7次命令下发，如果成功
   {
    USARTSend(0xa0+n);
	  ErrorFlag = 0x00;
   }

}


/*******************************************************************************
* ???  : W5500_Initialization
* ??    : W5500?????
* ??    : ?
* ??    : ?
* ???  : ?
* ??    : ?
*******************************************************************************/
void W5500_Initialization(void)
{
	W5500_Init();		//???W5500?????
	Detect_Gateway();	//??????? 
	Socket_Init(0);		//??Socket(0~7)???,?????0
}

/*******************************************************************************
* ???  : Load_Net_Parameters
* ??    : ??????
* ??    : ?
* ??    : ?
* ???  : ?
* ??    : ?????????????IP?????????IP???????????????
*******************************************************************************/
void Load_Net_Parameters(void)
{
	Gateway_IP[0] = 192;//??????
	Gateway_IP[1] = 168;
	Gateway_IP[2] = 1;
	Gateway_IP[3] = 1;

	Sub_Mask[0]=255;//??????
	Sub_Mask[1]=255;
	Sub_Mask[2]=255;
	Sub_Mask[3]=0;

	Phy_Addr[0]=0x0c;//??????
	Phy_Addr[1]=0x29;
	Phy_Addr[2]=0xab;
	Phy_Addr[3]=0x7c;
	Phy_Addr[4]=0x00;
	Phy_Addr[5]=0x01;

	IP_Addr[0]=192;//????IP??
	IP_Addr[1]=168;
	IP_Addr[2]=1;
	IP_Addr[3]=199;

	S0_Port[0] = 0x13;//????0????5000 
	S0_Port[1] = 0x88;

//	S0_DIP[0]=192;//????0???IP??
//	S0_DIP[1]=168;
//	S0_DIP[2]=1;
//	S0_DIP[3]=190;
//	
//	S0_DPort[0] = 0x17;//????0??????6000
//	S0_DPort[1] = 0x70;

//	UDP_DIPR[0] = 192;	//UDP(??)??,????IP??
//	UDP_DIPR[1] = 168;
//	UDP_DIPR[2] = 1;
//	UDP_DIPR[3] = 190;
//
//	UDP_DPORT[0] = 0x17;	//UDP(??)??,???????
//	UDP_DPORT[1] = 0x70;

	S0_Mode=UDP_MODE;//????0?????,UDP??
}

/*******************************************************************************
* ???  : W5500_Socket_Set
* ??    : W5500???????
* ??    : ?
* ??    : ?
* ???  : ?
* ??    : ????4???,????????,?????TCP????TCP????UDP??.
*			???????Socket_State???????????
*******************************************************************************/
void W5500_Socket_Set(void)
{
	if(S0_State==0)//??0?????
	{
		if(S0_Mode==TCP_SERVER)//TCP????? 
		{
			if(Socket_Listen(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else if(S0_Mode==TCP_CLIENT)//TCP????? 
		{
			if(Socket_Connect(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else//UDP?? 
		{
			if(Socket_UDP(0)==TRUE)
				S0_State=S_INIT|S_CONN;
			else
				S0_State=0;
		}
	}
}

/*******************************************************************************
* ???  : Process_Socket_Data
* ??    : W5500???????????
* ??    : s:???
* ??    : ?
* ???  : ?
* ??    : ??????S_rx_process()?W5500??????????????,
*			?????????Rx_Buffer???Temp_Buffer????????
*			????,????Temp_Buffer???Tx_Buffer??????S_tx_process()
*			?????
*******************************************************************************/
void Process_Socket_Data(SOCKET s)
{
	unsigned short size;
	size=Read_SOCK_Data_Buffer(s, Rx_Buffer);
	UDP_DIPR[0] = Rx_Buffer[0];
	UDP_DIPR[1] = Rx_Buffer[1];
	UDP_DIPR[2] = Rx_Buffer[2];
	UDP_DIPR[3] = Rx_Buffer[3];

	UDP_DPORT[0] = Rx_Buffer[4];
	UDP_DPORT[1] = Rx_Buffer[5];
	memcpy(Tx_Buffer, Rx_Buffer+8, size-8);			
	Write_SOCK_Data_Buffer(s, Tx_Buffer, size);
}


void Send_ETH_Data(SOCKET s, unsigned char *Tx_Buffer, unsigned int size)
{
 Write_SOCK_Data_Buffer(s, Tx_Buffer, size);
}



int main(void) //MAIN Function
{ 

	
	Stm32_Clock_Init(336,8,2,7);
	delay_init(168);		
	uart_init(84,115200);	
	SPIInit();
	PortDInit();
	ResetFPGA();
	PortEInit();
	
	Load_Net_Parameters();			
	W5500_Hardware_Reset();		
	W5500_Initialization();		
	W5500_Socket_Set();	
	
	while (1)
	{
		if((Rx_Buffer[0]==0xAB)&&(Rx_Buffer[1]==0xCD) )  
		  {
// When STM32 has received 0xAB for 1st byte and 0xCD for 2nd byte , 
// STM32 W5500 would send the whole received information to the other termination. 
// By Sun Libo Tel and WeChat : 15889672958
		  	Send_ETH_Data(0, Rx_Buffer, ETH_Receive_Data_Number);	
 		    Rx_Buffer[0]=0x00;
				Rx_Buffer[1]=0x00;
			}
	} 
	
}


void EXTI3_IRQHandler(void)
{
// Please notice that W5500 received data from the other termination or sent data to the other termination 
// both trigger the EXTI3_IRQHandler as negedge from W5500 Int Pin.   
// By Sun Libo Tel and WeChat : 15889672958
  ETH_Interrupt_Count++;
	USARTSend(ETH_Interrupt_Count);
  W5500_Interrupt_Process();
	
	EXTI->PR |= 1<<3;  	
}

