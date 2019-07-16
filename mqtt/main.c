
#include "stm32f10x.h"
#include "w5500.h"
#include "dhcp.h"
#include "socket.h"
 #include "MQTTClient.h"
 #include "mqtt_interface.h"
 #include "FreeRTOS.h"
 #include "task.h"


/***************----- W5500 GPIO���� -----***************/
#define W5500_SCS		GPIO_Pin_4	//����W5500��CS����	 
#define W5500_SCS_PORT	GPIOA
	
#define W5500_RST		GPIO_Pin_3	//����W5500��RST����
#define W5500_RST_PORT	GPIOA

#define W5500_INT		GPIO_Pin_4	//����W5500��INT����
#define W5500_INT_PORT	GPIOC
extern wiz_NetInfo NetConf = {
  {0x0c,0x29,0xab,0x7c,0x04,0x02},  // mac��ַ
  {192,168,0,113},                  // ����IP��ַ
  {255,255,255,0},                  // ��������
  {192,168,0,1},                    // ���ص�ַ
  {192,168,0,1},                        // DNS��������ַ
  NETINFO_STATIC                    // ʹ�þ�̬IP
};

uint8_t myport = 12345;


/********** ���ð�����ģʽ **********/
#pragma import(__use_no_semihosting)
 
struct __FILE
{
	int a;
};
 
FILE __stdout;
 
void _sys_exit(int x)
{
	
}

/* ���������� */
static TaskHandle_t connect_Handle = NULL;
 /* ���������� */
static TaskHandle_t AppTask_Handle = NULL;


//Socket number defines
#define TCP_SOCKET	0


//Receive Buffer Size define
#define BUF_SIZE 2048

//Global variables
int tcp_state = 0;
int mqtt_state = 0;  //mqtt_state 1 is connect,0 is unconnect
//unsigned char targetIP[4] = {139,196,135,135}; // mqtt server IP  
unsigned char targetIP[4] = {192,168,0,101}; // mqtt server IP 
unsigned int targetPort = 1883; // mqtt server port
unsigned char tempBuffer[BUF_SIZE];
unsigned char sbuf[200];
Network n;
MQTTClient c;
char* subTopic = "/a17b2zgOxYj/mike_001/user/mytopic1";
char* pubTopic = "/a17b2zgOxYj/mike_001/user/mytopic1";

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
*function:	д�ַ��ļ�����
*param1:	������ַ�
*param2:	�ļ�ָ��
*return:	����ַ���ASCII��
******************************************************/
int fputc(int ch, FILE *f)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);		//�ȴ��ϴη��ͽ���
	USART_SendData(USART1, (unsigned char)ch);				//�������ݵ�����
	return ch;
}


/*****************************************************
*function:	��ʼ������1
*param:		���ڲ�����
*return:		
******************************************************/
void USART1_Init(unsigned int BaudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef	USART_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);			//ʹ��USART1��GPIOAʱ��
	
	/* TX - PA.9 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;							//PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;							//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* RX - PA.10 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;							//PA.10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;						//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = BaudRate;							//������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;					//�ֳ�8λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;						//ֹͣλ1λ
	USART_InitStructure.USART_Parity = USART_Parity_No;						//����żУ��
	USART_InitStructure.USART_HardwareFlowControl	= USART_HardwareFlowControl_None;		//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//��/��ģʽ
	
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);							//���������ж�
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);							//���������ж�
	USART_Cmd(USART1, ENABLE);
}


/*****************************************************
*function:	����1�ж�����
*param:			
*return:		
******************************************************/
void NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);				//�жϷ���1��1λ��ռ���ȼ���3λ��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			//�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;			//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);
}




void delay_us(uint16_t time)
{    
   uint16_t i=0;  
   while(time--)
   {
      i=10;  //�Լ�����
      while(i--) ;    
   }
}

//���뼶����ʱ
void delay_ms(uint16_t time)
{    
   uint16_t i=0;  
   while(time--)
   {
      i=12000;  //�Լ�����
      while(i--) ;    
   }
}


/*******************************************************************************
* ������  : SPI_Configuration
* ����    : W5500 SPI��ʼ������(STM32 SPI1)
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void SPI_Configuration(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	SPI_InitTypeDef   	SPI_InitStructure;	   

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE);	

	/* ��ʼ��SCK��MISO��MOSI���� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // GPIO_Mode_Out_PP;   //GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);//
	GPIO_ResetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIO_ResetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);//
	
	/* ��ʼ��RST���� */
	GPIO_InitStructure.GPIO_Pin = W5500_RST;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(W5500_RST_PORT, &GPIO_InitStructure);
	GPIO_SetBits(W5500_RST_PORT,W5500_RST);

	/* ��ʼ��CS���� */
	GPIO_InitStructure.GPIO_Pin = W5500_SCS;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(W5500_SCS_PORT, &GPIO_InitStructure);
	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS);

	/* ��ʼ������STM32 SPI1 */
	SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;	//SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode=SPI_Mode_Master;							//����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b;						//SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_Low;							//ʱ�����յ�
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_1Edge;							//���ݲ����ڵ�1��ʱ����
	SPI_InitStructure.SPI_NSS=SPI_NSS_Soft;								//NSS���ⲿ�ܽŹ���
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_4;	//������Ԥ��ƵֵΪ4
	SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;					//���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial=7;								//CRC����ʽΪ7
	SPI_Init(SPI1,&SPI_InitStructure);									//����SPI_InitStruct��ָ���Ĳ�����ʼ������SPI1�Ĵ���

	SPI_Cmd(SPI1,ENABLE);	//STM32ʹ��SPI1
}



/*******************************************************************************
* ������  : System_Initialization
* ����    : STM32ϵͳ��ʼ������(��ʼ��STM32ʱ�Ӽ�����)
* ����    : ��
* ���    : ��
* ����    : �� 
* ˵��    : ��
*******************************************************************************/
void System_Initialization(void)
{
	SystemInit();	// ����ϵͳʱ��Ϊ72M 
	SPI_Configuration();		//W5500 SPI��ʼ������(STM32 SPI1)
}


/*******************************************************************************
* ������  : W5500_Hardware_Reset
* ����    : Ӳ����λW5500
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : W5500�ĸ�λ���ű��ֵ͵�ƽ����500us����,������ΧW5500
*******************************************************************************/
void W5500_Hardware_Reset(void)
{
	GPIO_ResetBits(W5500_RST_PORT, W5500_RST);//��λ��������
	delay_ms(1);
	GPIO_SetBits(W5500_RST_PORT, W5500_RST);//��λ��������
	delay_ms(1);
}
/*******************************************************************************
* ������  : cs_select
* ����    : Ƭѡѡ��
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��W5500��SCSΪ�͵�ƽ
*******************************************************************************/
void 	cs_select(void)            
{
	GPIO_ResetBits(W5500_SCS_PORT, W5500_SCS);//��W5500��SCSΪ�͵�ƽ
}
/*******************************************************************************
* ������  : cs_deselect
* ����    : Ƭѡȡ��
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��W5500��SCSΪ�ߵ�ƽ
*******************************************************************************/
void 	cs_deselect(void)          
{
	GPIO_SetBits(W5500_SCS_PORT, W5500_SCS);//��W5500��SCSΪ�ߵ�ƽ
}


/*******************************************************************************
* ������  : SPI1_Send_Byte
* ����    : SPI1����1���ֽ�����
* ����    : dat:�����͵�����
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void SPI1_Send_Byte(uint8_t dat)
{
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1,dat);//д1���ֽ�����
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_I2S_ReceiveData(SPI1);
}

/*******************************************************************************
* ������  : SPI1_Read_Byte
* ����    : SPI1��ȡ1���ֽ�����
* ����    : ��
* ���    : ��
* ����ֵ  : ��ȡ����һ���ֽ�����
* ˵��    : ��
*******************************************************************************/
uint8_t SPI1_Read_Byte(void)
{
	uint8_t ret;
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1,0x00);//д1���ֽ�����
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	ret = SPI_I2S_ReceiveData(SPI1);
	return ret;
}


/*******************************************************************************
* ������  : SPI1_Send_nByte
* ����    : SPI1����n���ֽ�����
* ����    : *dat_ptr:��д�����ݻ�����ָ��,len:��д������ݳ���
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void SPI1_Send_nByte(uint8_t *dat_ptr, uint16_t len)
{
	uint8_t i;
	for(i=0;i<len;i++)//ѭ������������size���ֽ�����д��W5500
	{
		SPI1_Send_Byte(*dat_ptr++);//дһ���ֽ�����
	}
}

/*******************************************************************************
* ������  : SPI1_Read_nByte
* ����    :  SPI1����n���ֽ�����
* ����    : *dat_ptr:��д�����ݻ�����ָ��,len:��д������ݳ���
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void SPI1_Read_nByte(uint8_t *dat_ptr, uint16_t len)
{
	uint8_t i;
	for(i=0;i<len;i++)//ѭ������������len���ֽ����ݶ���dat_ptr
	{
		SPI1_Send_Byte(0x00);  //����һ��������
		*dat_ptr++ = SPI_I2S_ReceiveData(SPI1);
	}
}

extern void	vPortEnterCritical();
extern void	vPortExitCritical();

/*******************************************************************************
* ������  : RegisterW5500Func
* ����    : ע��5500����
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ע��5500����
*******************************************************************************/
void RegisterW5500Func(void)          
{
	// ע����������
  reg_wizchip_spi_cbfunc(SPI1_Read_Byte,SPI1_Send_Byte);
  
  reg_wizchip_cris_cbfunc(vPortEnterCritical, vPortExitCritical);
  
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
	
}

/*******************************************************************************
* ������  : configNet
* ����    : W5500оƬ��λ
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : W5500оƬ��λ
*******************************************************************************/
void configNet()
{
	W5500_Hardware_Reset();
	uint8_t ar[16] = {2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2}; // ȫ���շ���������Ϊ2KB(Ĭ��)
	ctlwizchip(CW_INIT_WIZCHIP,ar);
	uint8_t linkStatus;
	uint8_t ret;
	printf("check linkstatus*************************\r\n");
	do
	{
		ret = ctlwizchip(CW_GET_PHYLINK, (void*) linkStatus);
		if(ret!=0||linkStatus!=PHY_LINK_ON)
		{
			printf("linkstatus is :%x\r\n",linkStatus);
			delay_ms(1000);
		}
	}
	while(ret==0&&linkStatus==PHY_LINK_ON);
	printf("*************************\r\n");
	printf("config netinfo****************\r\n");
	wiz_NetInfo conf;
	do
	{
		setSHAR(NetConf.mac); //set MAC
		// ���������ַ
		ctlnetwork(CN_SET_NETINFO,(void *)&NetConf);
		// �ض�
		ctlnetwork(CN_GET_NETINFO,(void *)&conf);
		if(memcmp(&conf,&NetConf,sizeof(wiz_NetInfo)) != 0)
		{
			printf("config netinfo fail \r\n");
			delay_ms(1000);
		}
	}
	while(memcmp(&conf,&NetConf,sizeof(wiz_NetInfo)) == 0)
	printf("config netinfo success \r\n");nfo)) == 0){
	wiz_NetTimeout to;
	to.retry_cnt = 8;   // ���Դ�����Ĭ��8��
	to.time_100us = 2000; // ��ʱʱ�䣬Ĭ��2000*100us = 200ms
	wizchip_settimeout(&to);
}



uint8_t loopback_tcpc1()
{
   static uint16_t any_port =   50000;
   switch(getSn_SR(TCP_SOCKET))
   {
      case SOCK_ESTABLISHED :
         if(getSn_IR(TCP_SOCKET) & Sn_IR_CON)	// Socket n interrupt register mask; TCP CON interrupt = connection with peer is successful
         {
			setSn_IR(TCP_SOCKET, Sn_IR_CON);  // this interrupt should be write the bit cleared to '1'
         }
		 return SOCK_ESTABLISHED;
		 //break;
      case SOCK_CLOSE_WAIT :
         disconnect(TCP_SOCKET);
         break;
      case SOCK_INIT :
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



static void tcpTask(void* parameter)
{	
    while (1)
    {
		taskENTER_CRITICAL();
		tcp_state = loopback_tcpc1();
		if(tcp_state==SOCK_ESTABLISHED)
		{
			if(mqtt_state==0)
			{
				printf("tcp Connected %d\r\n", rc);
				if(MQTTConnect(&c, &data)!=FAILURE)
				{
					mqtt_state = 1;
					MQTTSubscribe(&c, subTopic, opts.qos, messageArrived);
				}
			}
			
		}
		else
		{
			mqtt_state = 0;
		}
		taskEXIT_CRITICAL();
		const portTickType xDelay = pdMS_TO_TICKS(200);
		vTaskDelay( xDelay );
    }
}


static void printInfoTask(void* parameter)
{	
    while (1)
    {
		taskENTER_CRITICAL();
		printf("tcp_state=%x, mqtt_state=%x\r\n",tcp_state,mqtt_state);
		taskEXIT_CRITICAL();
		const portTickType xDelay = pdMS_TO_TICKS(10000);
		vTaskDelay( xDelay );
    }
}

static void subTask(void* parameter)
{	
    while (1)
    {
		if(tcp_state==SOCK_ESTABLISHED&&mqtt_state==1)
		{
			MQTTYield(&c, 300);
		}
    }
}

static void pubTask(void* parameter)
{	
    while (1)
    {
		if(tcp_state==SOCK_ESTABLISHED&&mqtt_state==1)
		{
			MQTTPublish  ( &n,    pubTopic  ,    "ttcc\n\r" ); 
		}
		const portTickType xDelay = pdMS_TO_TICKS(1000);
		vTaskDelay( xDelay );
    }
}


int main(void)
{
	System_Initialization();	//STM32ϵͳ��ʼ������(��ʼ��STM32ʱ�Ӽ�����)
	USART1_Init(115200);
	NVIC_Config();
	RegisterW5500Func();
	//��ʼ������w5500
	configNet();
	/*��װmqtt��������*/
	memset(tempBuffer, 0, BUF_SIZE); 
	NewNetwork(&n, TCP_SOCKET);
	c.defaultMessageHandler = DefaultMesHand;
	MQTTClientInit(&c,&n,1000,sbuf,200,tempBuffer,2048);
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	data.willFlag = 0;
	data.MQTTVersion = 3;
	data.clientID.cstring = opts.clientid;
	data.username.cstring = opts.username;
	data.password.cstring = opts.password;
	data.keepAliveInterval = 30000;
	data.cleansession = 1;
	opts.showtopics = 1;
	xTaskCreate((TaskFunction_t )tcpTask,  /* ������ں��� */
                        (const char*    )"tcpTask",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )2, /* ��������ȼ� */
                        (TaskHandle_t*  )&AppTask_Handle);/* ������ƿ�ָ�� */ 
	xTaskCreate((TaskFunction_t )printInfoTask,  /* ������ں��� */
                        (const char*    )"printInfoTask",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )1, /* ��������ȼ� */
                        (TaskHandle_t*  )&AppTask_Handle);/* ������ƿ�ָ�� */
	xTaskCreate((TaskFunction_t )subTask,  /* ������ں��� */
                        (const char*    )"subTask",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )2, /* ��������ȼ� */
                        (TaskHandle_t*  )&AppTask_Handle);/* ������ƿ�ָ�� */ 
	xTaskCreate((TaskFunction_t )pubTask,  /* ������ں��� */
                        (const char*    )"pubTask",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )2, /* ��������ȼ� */
                        (TaskHandle_t*  )&AppTask_Handle);/* ������ƿ�ָ�� */
	printf("start\r\n");
	vTaskStartScheduler();
	while(1)
	{
		
	}
	
}
