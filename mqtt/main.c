
#include "stm32f10x.h"
#include "w5500.h"
#include "dhcp.h"
#include "socket.h"
 #include "MQTTClient.h"
 #include "mqtt_interface.h"
 //#include "loopback.h"


/***************----- W5500 GPIO定义 -----***************/
#define W5500_SCS		GPIO_Pin_4	//定义W5500的CS引脚	 
#define W5500_SCS_PORT	GPIOA
	
#define W5500_RST		GPIO_Pin_3	//定义W5500的RST引脚
#define W5500_RST_PORT	GPIOA

#define W5500_INT		GPIO_Pin_4	//定义W5500的INT引脚
#define W5500_INT_PORT	GPIOC
extern wiz_NetInfo NetConf = {
  {0x0c,0x29,0xab,0x7c,0x04,0x02},  // mac地址
  {192,168,0,113},                  // 本地IP地址
  {255,255,255,0},                  // 子网掩码
  {192,168,0,1},                    // 网关地址
  {192,168,0,1},                        // DNS服务器地址
  NETINFO_STATIC                    // 使用静态IP
};

uint8_t myport = 12345;


/********** 禁用半主机模式 **********/
#pragma import(__use_no_semihosting)
 
struct __FILE
{
	int a;
};
 
FILE __stdout;
 
void _sys_exit(int x)
{
	
}

//Socket number defines
#define TCP_SOCKET	0

//Receive Buffer Size define
#define BUF_SIZE 2048

//Global variables
//unsigned char targetIP[4] = {139,196,135,135}; // mqtt server IP  
unsigned char targetIP[4] = {192,168,0,101}; // mqtt server IP 
unsigned int targetPort = 1883; // mqtt server port
unsigned char tempBuffer[BUF_SIZE];

uint8_t loopback_tcpc1();

struct opts_struct
{
	char* clientid;
	int nodelimiter;
	char* delimiter;
	enum QoS qos;
	char* username;
	char* password;
	char* host;
	int port;
	int showtopics;
} opts ={ (char*)"12345|securemode=3,signmethod=hmacsha1|", 0, (char*)"\n", QOS0, (char*)"mike_001&a17b2zgOxYj", (char*)"125947B2E8123264B5CEAEA1116665CD21B9B20F", NULL, NULL, 0 };


// @brief messageArrived callback function
void messageArrived(MessageData* md)
{
	unsigned char testbuffer[100];
	MQTTMessage* message = md->message;

	if (opts.showtopics)
	{
		memcpy(testbuffer,(char*)message->payload,(int)message->payloadlen);
		*(testbuffer + (int)message->payloadlen + 1) = '\n';
		printf("%s\r\n",testbuffer);
	}

	if (opts.nodelimiter)
		printf("%.*s", (int)message->payloadlen, (char*)message->payload);
	else
		printf("%.*s%s", (int)message->payloadlen, (char*)message->payload, opts.delimiter);
}

void delay_us(uint16_t time);

void f2()
{
	
}

/*****************************************************
*function:	写字符文件函数
*param1:	输出的字符
*param2:	文件指针
*return:	输出字符的ASCII码
******************************************************/
int fputc(int ch, FILE *f)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);		//等待上次发送结束
	USART_SendData(USART1, (unsigned char)ch);				//发送数据到串口
	return ch;
}


/*****************************************************
*function:	初始化串口1
*param:		串口波特率
*return:		
******************************************************/
void USART1_Init(unsigned int BaudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef	USART_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);			//使能USART1，GPIOA时钟
	
	/* TX - PA.9 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;							//PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;							//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* RX - PA.10 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;							//PA.10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;						//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = BaudRate;							//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;					//字长8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;						//停止位1位
	USART_InitStructure.USART_Parity = USART_Parity_No;						//无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl	= USART_HardwareFlowControl_None;		//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收/发模式
	
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);							//开启接收中断
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);							//开启空闲中断
	USART_Cmd(USART1, ENABLE);
}


/*****************************************************
*function:	串口1中断配置
*param:			
*return:		
******************************************************/
void NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);				//中断分组1：1位抢占优先级，3位响应优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			//中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断
	NVIC_Init(&NVIC_InitStructure);
}

// @brief 1 millisecond Tick Timer setting
void NVIC_configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	SysTick_Config(72000);
	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // Highest priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/*******************************************************************************
* 函数名  : delay_us
* 描述    : 延时函数
* 输入    : us
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void delay_us(uint16_t time)
{
    uint16_t i,j;
    for(i=0;i<time;i++)
    {
        j=32;
        while(j>1)j--;
    }
}

/*******************************************************************************
* 函数名  : SPI_Configuration
* 描述    : W5500 SPI初始化配置(STM32 SPI1)
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void SPI_Configuration(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	SPI_InitTypeDef   	SPI_InitStructure;	   

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE);	

	/* 初始化SCK、MISO、MOSI引脚 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // GPIO_Mode_Out_PP;   //GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);//
	GPIO_ResetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIO_ResetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);//
	
	/* 初始化RST引脚 */
	GPIO_InitStructure.GPIO_Pin = W5500_RST;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(W5500_RST_PORT, &GPIO_InitStructure);
	GPIO_SetBits(W5500_RST_PORT,W5500_RST);

	/* 初始化CS引脚 */
	GPIO_InitStructure.GPIO_Pin = W5500_SCS;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(W5500_SCS_PORT, &GPIO_InitStructure);
	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS);

	/* 初始化配置STM32 SPI1 */
	SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;	//SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode=SPI_Mode_Master;							//设置为主SPI
	SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b;						//SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_Low;							//时钟悬空低
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_1Edge;							//数据捕获于第1个时钟沿
	SPI_InitStructure.SPI_NSS=SPI_NSS_Soft;								//NSS由外部管脚管理
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_2;	//波特率预分频值为2
	SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;					//数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial=7;								//CRC多项式为7
	SPI_Init(SPI1,&SPI_InitStructure);									//根据SPI_InitStruct中指定的参数初始化外设SPI1寄存器

	SPI_Cmd(SPI1,ENABLE);	//STM32使能SPI1
}



/*******************************************************************************
* 函数名  : System_Initialization
* 描述    : STM32系统初始化函数(初始化STM32时钟及外设)
* 输入    : 无
* 输出    : 无
* 返回    : 无 
* 说明    : 无
*******************************************************************************/
void System_Initialization(void)
{
	SystemInit();	// 配置系统时钟为72M 
	SPI_Configuration();		//W5500 SPI初始化配置(STM32 SPI1)
}


/*******************************************************************************
* 函数名  : W5500_Hardware_Reset
* 描述    : 硬件复位W5500
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : W5500的复位引脚保持低电平至少500us以上,才能重围W5500
*******************************************************************************/
void W5500_Hardware_Reset(void)
{
	GPIO_ResetBits(W5500_RST_PORT, W5500_RST);//复位引脚拉低
	delay_us(50);
	GPIO_SetBits(W5500_RST_PORT, W5500_RST);//复位引脚拉高
	delay_us(200);
}
/*******************************************************************************
* 函数名  : cs_select
* 描述    : 片选选择
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 置W5500的SCS为低电平
*******************************************************************************/
void 	cs_select(void)            
{
	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为低电平
}
/*******************************************************************************
* 函数名  : cs_deselect
* 描述    : 片选取消
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 置W5500的SCS为高电平
*******************************************************************************/
void 	cs_deselect(void)          
{
	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS);//置W5500的SCS为高电平
}


/*******************************************************************************
* 函数名  : SPI1_Send_Byte
* 描述    : SPI1发送1个字节数据
* 输入    : dat:待发送的数据
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void SPI1_Send_Byte(uint8_t dat)
{
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1,dat);//写1个字节数据
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_I2S_ReceiveData(SPI1);
}

/*******************************************************************************
* 函数名  : SPI1_Read_Byte
* 描述    : SPI1读取1个字节数据
* 输入    : 无
* 输出    : 无
* 返回值  : 读取到的一个字节数据
* 说明    : 无
*******************************************************************************/
uint8_t SPI1_Read_Byte(void)
{
	uint8_t ret;
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1,0x00);//写1个字节数据
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	ret = SPI_I2S_ReceiveData(SPI1);
	return ret;
}


/*******************************************************************************
* 函数名  : SPI1_Send_nByte
* 描述    : SPI1发送n个字节数据
* 输入    : *dat_ptr:待写入数据缓冲区指针,len:待写入的数据长度
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void SPI1_Send_nByte(uint8_t *dat_ptr, uint16_t len)
{
	uint8_t i;
	for(i=0;i<len;i++)//循环将缓冲区的size个字节数据写入W5500
	{
		SPI1_Send_Byte(*dat_ptr++);//写一个字节数据
	}
}

/*******************************************************************************
* 函数名  : SPI1_Read_nByte
* 描述    :  SPI1接收n个字节数据
* 输入    : *dat_ptr:待写入数据缓冲区指针,len:待写入的数据长度
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void SPI1_Read_nByte(uint8_t *dat_ptr, uint16_t len)
{
	uint8_t i;
	for(i=0;i<len;i++)//循环将缓冲区的len个字节数据读入dat_ptr
	{
		SPI1_Send_Byte(0x00);  //发送一个哑数据
		*dat_ptr++ = SPI_I2S_ReceiveData(SPI1);
	}
}


/*******************************************************************************
* 函数名  : RegisterW5500Func
* 描述    : 注册5500函数
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 注册5500函数
*******************************************************************************/
void RegisterW5500Func(void)          
{
	// 注册驱动函数
  reg_wizchip_spi_cbfunc(SPI1_Read_Byte,SPI1_Send_Byte);
  //reg_wizchip_spiburst_cbfunc(SPI1_Read_nByte,SPI1_Send_nByte);
  reg_wizchip_cs_cbfunc(cs_select,cs_deselect);
}

void DefaultMesHand(MessageData* mesdata)
{
	printf("topic:%s\r\n",mesdata->topicName->cstring);
	printf("data:%s\r\n",mesdata->message->payload);
}

void f1()
{
	memset(tempBuffer, 0, BUF_SIZE); 
	int rc = 0;
	int con = 1;
	int sub = 1;
	unsigned char buf[200];
	Network n;
	MQTTClient c;
	NewNetwork(&n, TCP_SOCKET);
	c.defaultMessageHandler = DefaultMesHand;
	MQTTClientInit(&c,&n,1000,buf,200,tempBuffer,2048);

	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	data.willFlag = 0;
	data.MQTTVersion = 3;
	data.clientID.cstring = opts.clientid;
	data.username.cstring = opts.username;
	data.password.cstring = opts.password;
	data.keepAliveInterval = 300;
	data.cleansession = 1;
	opts.showtopics = 1;
	while(1)
	{
		//loopback_tcpc(0, buf, targetIP, targetPort);

		///*
		rc = loopback_tcpc1();
		if(con&&(rc&SOCK_ESTABLISHED))
		{
			rc = MQTTConnect(&c, &data);
			printf("Connected %d\r\n", rc);
			if(rc!=FAILURE)
				con = 0;
		}
		if(sub&&(con==0))
		{
			printf("Subscribing to %s\r\n", "/a17b2zgOxYj/mike_001/user/mytopic1");
			rc = MQTTSubscribe(&c, "/a17b2zgOxYj/mike_001/user/mytopic1", opts.qos, messageArrived);
			printf("Subscribed %d\r\n", rc);
			if(rc!=FAILURE)
				sub = 0;
		}
		if(!sub)
		{
			MQTTYield(&c, data.keepAliveInterval);
		}
		//*/
	}
	
}

/*******************************************************************************
* 函数名  : configNet
* 描述    : W5500芯片复位
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : W5500芯片复位
*******************************************************************************/
void configNet(){
  wiz_NetInfo conf;
	uint8_t buf[200];
	setSHAR(NetConf.mac); //set MAC
	
  // 配置网络地址
  ctlnetwork(CN_SET_NETINFO,(void *)&NetConf);
  // 回读
  ctlnetwork(CN_GET_NETINFO,(void *)&conf);
  if(memcmp(&conf,&NetConf,sizeof(wiz_NetInfo)) == 0){
    // 配置成功
	
		wiz_NetTimeout to;
		to.retry_cnt = 8;   // 重试次数，默认8次
		to.time_100us = 2000; // 超时时间，默认2000*100us = 200ms
		wizchip_settimeout(&to);
		DNS_init(0,buf);
		//uint8_t * dns_ip =  &(NetConf.dns);
		uint8_t * name = "m2m.eclipse.org";
		uint8_t ip_from_dns[4] = {0,0,0,0};
		

  }else{
    // 配置失败
  }
}



uint8_t loopback_tcpc1()
{
	 static uint16_t any_port =   50000;
   switch(getSn_SR(TCP_SOCKET))
   {
      case SOCK_ESTABLISHED :
         if(getSn_IR(TCP_SOCKET) & Sn_IR_CON)	// Socket n interrupt register mask; TCP CON interrupt = connection with peer is successful
         {
			printf("Connected");
			setSn_IR(TCP_SOCKET, Sn_IR_CON);  // this interrupt should be write the bit cleared to '1'
         }
		 return SOCK_ESTABLISHED;
		 //break;
      case SOCK_CLOSE_WAIT :
         disconnect(TCP_SOCKET);
         break;
      case SOCK_INIT :
    	 printf("Try to connect");
    	 connect(TCP_SOCKET, targetIP, targetPort);
         break;
      case SOCK_CLOSED:
    	 close(TCP_SOCKET);
    	 socket(TCP_SOCKET, Sn_MR_TCP, any_port, 0x00);
         break;
      default:
         break;
   }
   return 0;
}



int main(void)
{
	System_Initialization();	//STM32系统初始化函数(初始化STM32时钟及外设)
	USART1_Init(115200);
	NVIC_Config();
	NVIC_configuration();
	W5500_Hardware_Reset();
	RegisterW5500Func();
	uint8_t ar[16] = {2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2}; // 全部收发缓冲区设为2KB(默认)
	ctlwizchip(CW_INIT_WIZCHIP,ar);
	configNet();
	f1();
	
}
