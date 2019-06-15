
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

void f2()
{
	loopback_tcpc(0x0, dataBuffer, dest_ip, dest_port);
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

/*****************************************************
*function:	����1�жϷ���������ӡ���յ����ֽ�
*param:			
*return:		
******************************************************/
void USART1_IRQHandler(void)
{
	static unsigned char buff[64];
	static unsigned char n = 0;
	unsigned char i;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)		//�ж��Ƿ�Ϊ�����ж�
	{
		buff[n++] = USART1->DR;																//��ȡ���յ����ֽ�����
 
		if(n == 64)
		{
			n = 0;
		}
	}
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)		//�ж��Ƿ�Ϊ�����ж�
	{
		USART1->DR;						//��DR�����־
		
		printf("%d characters:\r\n", n);
		for(i=0; i<n; i++)
		{
			printf("buff[%d] = 0x%02hhx\r\n", i, buff[i]);	//���ʮ�����ƣ����������λ��������0
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
