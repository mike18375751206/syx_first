
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

void f2()
{
	loopback_tcpc(0x0, dataBuffer, dest_ip, dest_port);
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

/*****************************************************
*function:	串口1中断服务函数，打印接收到的字节
*param:			
*return:		
******************************************************/
void USART1_IRQHandler(void)
{
	static unsigned char buff[64];
	static unsigned char n = 0;
	unsigned char i;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)		//判断是否为接收中断
	{
		buff[n++] = USART1->DR;																//读取接收到的字节数据
 
		if(n == 64)
		{
			n = 0;
		}
	}
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)		//判断是否为空闲中断
	{
		USART1->DR;						//读DR，清标志
		
		printf("%d characters:\r\n", n);
		for(i=0; i<n; i++)
		{
			printf("buff[%d] = 0x%02hhx\r\n", i, buff[i]);	//输出十六进制，保留最低两位，不够补0
		}
		n = 0;
	}
}

int main()
{
	USART1_Init(115200);
	NVIC_Config();
	printf("Hello, world!\r\n");
	printf("Please enter any character:\r\n");
	f2();
	while(1);
}
