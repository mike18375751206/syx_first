
#include "stm32f10x.h"
#include "w5500.h"
#include "dhcp.h"
#include "socket.h"
 #include "MQTTClient.h"
 #include "mqtt_interface.h"

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

//Socket number defines
#define TCP_SOCKET	0

//Receive Buffer Size define
#define BUF_SIZE 2048

//Global variables
//unsigned char targetIP[4] = {139,196,135,135}; // mqtt server IP  
unsigned char targetIP[4] = {192,168,0,101}; // mqtt server IP 
unsigned int targetPort = 1883; // mqtt server port
unsigned char tempBuffer[BUF_SIZE];

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
* ������  : delay_us
* ����    : ��ʱ����
* ����    : us
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
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
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_2;	//������Ԥ��ƵֵΪ2
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
	delay_us(50);
	GPIO_SetBits(W5500_RST_PORT, W5500_RST);//��λ��������
	delay_us(200);
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
  //reg_wizchip_spiburst_cbfunc(SPI1_Read_nByte,SPI1_Send_nByte);
  reg_wizchip_cs_cbfunc(cs_select,cs_deselect);
}


void f1()
{
	memset(tempBuffer, 0, BUF_SIZE); 
	int rc = 0;
  unsigned char buf[100];
  opts.host = targetIP;
	opts.port = targetPort;
	Network n;
	MQTTClient c;

	NewNetwork(&n, TCP_SOCKET);
	ConnectNetwork(&n, targetIP, targetPort);
	MQTTClientInit(&c,&n,1000,buf,100,tempBuffer,2048);

	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	data.willFlag = 0;
	data.MQTTVersion = 3;
	data.clientID.cstring = opts.clientid;
	data.username.cstring = opts.username;
	data.password.cstring = opts.password;

	data.keepAliveInterval = 300;
	data.cleansession = 1;

	rc = MQTTConnect(&c, &data);
	printf("Connected %d\r\n", rc);
	opts.showtopics = 1;

	printf("Subscribing to %s\r\n", "/a17b2zgOxYj/mike_001/user/mytopic1");
	rc = MQTTSubscribe(&c, "/a17b2zgOxYj/mike_001/user/mytopic1", opts.qos, messageArrived);
	printf("Subscribed %d\r\n", rc);

    while(1)
    {
    	MQTTYield(&c, data.keepAliveInterval);
    }
}

/*******************************************************************************
* ������  : configNet
* ����    : W5500оƬ��λ
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : W5500оƬ��λ
*******************************************************************************/
void configNet(){
  wiz_NetInfo conf;
	uint8_t buf[200];
	setSHAR(NetConf.mac); //set MAC
	
  // ���������ַ
  ctlnetwork(CN_SET_NETINFO,(void *)&NetConf);
  // �ض�
  ctlnetwork(CN_GET_NETINFO,(void *)&conf);
  if(memcmp(&conf,&NetConf,sizeof(wiz_NetInfo)) == 0){
    // ���óɹ�
	
		wiz_NetTimeout to;
		to.retry_cnt = 8;   // ���Դ�����Ĭ��8��
		to.time_100us = 2000; // ��ʱʱ�䣬Ĭ��2000*100us = 200ms
		wizchip_settimeout(&to);
		DNS_init(0,buf);
		//uint8_t * dns_ip =  &(NetConf.dns);
		uint8_t * name = "m2m.eclipse.org";
		uint8_t ip_from_dns[4] = {0,0,0,0};
		

  }else{
    // ����ʧ��
  }
}



void MQTT_CON_ALI( void )
{
	int	len;
	int	type;
	switch ( getSn_SR( TCP_SOCKET ) )                       /* ��ȡsocket0��״̬ */
	{
	case SOCK_INIT:                                         /* Socket���ڳ�ʼ�����(��)״̬ */
		connect( TCP_SOCKET, targetIP, targetPort );
		break;
	case SOCK_ESTABLISHED:                                  /* Socket�������ӽ���״̬ */
		if ( getSn_IR( TCP_SOCKET ) & Sn_IR_CON )
		{
			setSn_IR( TCP_SOCKET, Sn_IR_CON );      /* Sn_IR��CONλ��1��֪ͨW5500�����ѽ��� */
		}
		memset( msgbuf, 0, sizeof(msgbuf) );
		if ( (len = getSn_RX_RSR( 0 ) ) == 0 )
		{
			if ( 1 == CONNECT_FLAG )
			{
				printf( "send connect\r\n" );


				/*MQTTƴ�����ӱ���
				 * *���ݰ�����ƽ̨MQTT�豸�����ֲ�����
				 */

				/* void make_con_msg(char* clientID,int keepalive, uint8 cleansession,char*username,char* password,unsigned char*buf,int buflen) */
				make_con_msg( "192.168.207.115|securemode=3,signmethod=hmacsha1,timestamp=789|", 180, 1,
					      "MQTT1&TKKMt4nMF8U", "9076b0ebc04dba8a8ebba1f0003552dbc862c9b9", msgbuf, sizeof(msgbuf) );
				/*
				 * hamacsha1();//hamacsha1�ַ�������
				 * passwor = hamacsha1("secret","clientId192.168.207.115deviceNameMQTT1productKeyTKKMt4nMF8Utimestap789");
				 * printf(" server_ip: %d.%d.%d.%d\r\n", server_ip[0],server_ip[1],server_ip[2],server_ip[3]);
				 * printf("connect ALY\r\n");
				 */
				CONNECT_FLAG = 0;
				send( 0, msgbuf, sizeof(msgbuf) );
				Delay_s( 2 );
				while ( (len = getSn_RX_RSR( 0 ) ) == 0 )
				{
					Delay_s( 2 );
					send( 0, msgbuf, sizeof(msgbuf) );
				}
				;
				recv( 0, msgbuf, len );
				while ( mqtt_decode_msg( msgbuf ) != CONNACK ) /* �ж��ǲ���CONNACK */
				{
					printf( "wait ack\r\n" );
				}
			}else if ( SUB_FLAG == 1 )
			{
				memset( msgbuf, 0, sizeof(msgbuf) );
				make_sub_msg( topic, msgbuf, sizeof(msgbuf) );
				/* make_pub_msg(topic,msgbuf,sizeof(msgbuf),"hello"); */
				send( 0, msgbuf, sizeof(msgbuf) ); /* ���յ����ݺ��ٻظ���������������ݻػ� */
				SUB_FLAG = 0;
				Delay_s( 2 );
				while ( (len = getSn_RX_RSR( 0 ) ) == 0 )
				{
					Delay_s( 2 );
					send( 0, msgbuf, sizeof(msgbuf) );
				}
				;
				recv( 0, msgbuf, len );
				while ( mqtt_decode_msg( msgbuf ) != SUBACK ) /* �ж��ǲ���SUBACK */
				{
					printf( "wait suback\r\n" );
				}
				TIM_Cmd( TIM2, ENABLE );
				printf( "send sub\r\n" );
			}
#if 1
			else{
				/*
				 * count++;
				 * Delay_s(2);
				 */
				if ( count > 10000 )
				{
					count = 0;
					make_ping_msg( msgbuf, sizeof(msgbuf) );
					send( 0, msgbuf, sizeof(msgbuf) );

					while ( (len = getSn_RX_RSR( 0 ) ) == 0 )
					{
						/*
						 * Delay_s(2);
						 * send(0,msgbuf,sizeof(msgbuf));
						 */
						printf( "wait pingresponse" );
					}
					;
					recv( 0, msgbuf, len );
					printf( "ping len : %d\r\n", len );
					if ( len > 2 )
					{
						if ( PUBLISH == mqtt_decode_msg( msgbuf + 2 ) )
						{
							printf( "publish\r\n" );
							MQTTDeserialize_publish( &dup, &qos, &retained, &mssageid, &receivedTopic, &payload_in, &payloadlen_in, msgbuf + 2, len - 2 );
							/* printf("message arrived %d: %s\n\r", payloadlen_in, payload_in); */
							memset( topic, 0, sizeof(topic) );
							memset( ser_cmd, 0, sizeof(ser_cmd) );
							memcpy( topic, receivedTopic.lenstring.data, receivedTopic.lenstring.len );
							replace_string( new_topic, topic, "request", "response" );
							printf( "topic:%s\r\n", topic );
							strcpy( ser_cmd, (const char *) payload_in );
							/*
							 * parse_topic(ser_cmd);
							 * printf("message is %s\r\n",ser_cmd);
							 */
							memset( msgbuf, 0, sizeof(msgbuf) );
							make_pub_msg( new_topic, msgbuf, sizeof(msgbuf), "hello" );
							send( 0, msgbuf, sizeof(msgbuf) );
						}
					}
				}
			}
#endif
#if 0
			if ( PUB_FLAG == 1 )
			{
				memset( msgbuf, 0, sizeof(msgbuf) );
				/* make_sub_msg(topic,msgbuf,sizeof(msgbuf)); */
				make_pub_msg( topic, msgbuf, sizeof(msgbuf), "hello" );
				if ( count == 10000 )
				{
PUB:
					send( 0, msgbuf, sizeof(msgbuf) ); /* ���յ����ݺ��ٻظ���������������ݻػ� */

					Delay_s( 2 );
					/*
					 * while((len=getSn_RX_RSR(0))==0)
					 *  {
					 * Delay_s(2);
					 * send(0,msgbuf,sizeof(msgbuf));
					 *      printf("puback\r\n");
					 *  };
					 * recv(0,msgbuf,len);
					 * if(mqtt_decode_msg(msgbuf)!=PUBACK)//�ж��ǲ���SUBACK
					 *  {
					 *              goto PUB;
					 *     printf("wait Puback\r\n");
					 * }
					 */
					printf( "send Pub\r\n" );
				}
			}
#endif
		}
#if 1
		if ( (len = getSn_RX_RSR( 0 ) ) > 0 )
		{
			recv( 0, msgbuf, len );
			if ( PUBLISH == mqtt_decode_msg( msgbuf ) )
			{
				printf( "publish\r\n" );
				MQTTDeserialize_publish( &dup, &qos, &retained, &mssageid, &receivedTopic, &payload_in, &payloadlen_in, msgbuf, len );
				/* printf("message arrived %d: %s\n\r", payloadlen_in, payload_in); */
				memset( topic, 0, sizeof(topic) );
				memcpy( topic, receivedTopic.lenstring.data, receivedTopic.lenstring.len );
				replace_string( new_topic, topic, "request", "response" );

				printf( "topic:%s\r\n", topic );

				memset( ser_cmd, 0, sizeof(ser_cmd) );
				memcpy( ser_cmd, (const char *) payload_in, strlen( (char *) payload_in ) );
				memset( msgbuf, 0, sizeof(msgbuf) );
				make_pub_msg( new_topic, msgbuf, sizeof(msgbuf), rebuf );
				send( 0, msgbuf, sizeof(msgbuf) );
				/* printf("%s\n",msgbuf); */
			}else if ( PINGRESP == mqtt_decode_msg( msgbuf ) )
			{
				if ( len > 2 )
				{
					if ( PUBLISH == mqtt_decode_msg( msgbuf + 2 ) )
					{
						printf( "publish\r\n" );
						MQTTDeserialize_publish( &dup, &qos, &retained, &mssageid, &receivedTopic, &payload_in, &payloadlen_in, msgbuf + 2, len - 2 );
						/* printf("message arrived %d: %s\n\r", payloadlen_in, payload_in); */
						memset( topic, 0, sizeof(topic) );
						memcpy( topic, receivedTopic.lenstring.data, receivedTopic.lenstring.len );
						replace_string( new_topic, topic, "request", "response" );
						printf( "topic:%s\r\n", topic );
						memset( ser_cmd, 0, sizeof(ser_cmd) );
						strcpy( ser_cmd, (const char *) payload_in );
						/*
						 * printf("message is %s\r\n",ser_cmd);
						 * parse_topic(ser_cmd);
						 */
						memset( msgbuf, 0, sizeof(msgbuf) );
						make_pub_msg( new_topic, msgbuf, sizeof(msgbuf), "hello" );
						send( 0, msgbuf, sizeof(msgbuf) );
					}
				}
			}else{
				printf( "wait publish\r\n" );
			}
		}
		/*	printf("send ping\r\n"); */

#endif
		break;
	case SOCK_CLOSE_WAIT:           /* Socket���ڵȴ��ر�״̬ */
		close( TCP_SOCKET );    /* �ر�Socket0 */
		break;
	case SOCK_CLOSED:               /* Socket���ڹر�״̬ */
		socket( TCP_SOCKET, Sn_MR_TCP, myport, 0 );
		break;
	}
}


int main(void)
{
	System_Initialization();	//STM32ϵͳ��ʼ������(��ʼ��STM32ʱ�Ӽ�����)
	USART1_Init(115200);
	NVIC_Config();
	NVIC_configuration();
	W5500_Hardware_Reset();
	RegisterW5500Func();
	uint8_t ar[16] = {2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2}; // ȫ���շ���������Ϊ2KB(Ĭ��)
	ctlwizchip(CW_INIT_WIZCHIP,ar);
	configNet();
	f1();
	
}
