#include "sys.h"
#include "usart.h"	
#include "delay.h"
#include "dma.h"
#include "malloc.h"
#include "led.h"


////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos ʹ��	  
#endif

////////////////////////////////////////////////////////////////////////////////// 	 
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 
//end

u8 USART1_TX_BUF[USART_LEN]={0};
u8 USART1_RX_BUF[USART_LEN]={0};
u8 USART2_TX_BUF[USART_LEN]={0};
u8 USART2_RX_BUF[USART_LEN]={0};
u8 USART3_TX_BUF[USART_LEN]={0};
u8 USART3_RX_BUF[USART_LEN]={0};
u8 UART4_TX_BUF[USART_LEN]={0};
u8 UART4_RX_BUF[USART_LEN]={0};
u8 UART5_TX_BUF[USART_LEN]={0};
u8 UART5_RX_BUF[USART_LEN]={0};
u8 USART6_TX_BUF[USART_LEN]={0};
u8 USART6_RX_BUF[USART_LEN]={0};


int16_t real_current;
int16_t real_velocity;
int32_t real_position;

//=======================================================================	USART1

/*------------------------------------------------
* ��������void Init_USART1(u32 pclk2,u32 bound)
* ��  �ܣ���ʼ��IO ����1
* ��  ����pclk2: PCLK2ʱ��Ƶ��(Mhz)
	      bound: ������ 
* ����ֵ�� ��
------------------------------------------------*/
void Init_USART1(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV@OVER8=0
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������@OVER8=0 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->AHB1ENR|=1<<0;   	//ʹ��PORTA��ʱ��  
	RCC->APB2ENR|=1<<4;  	//ʹ�ܴ���1ʱ�� 
	GPIO_Set(GPIOA,PIN9|PIN10,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PA9,PA10,���ù���,�������
 	GPIO_AF_Set(GPIOA,9,7);	//PA9,AF7
	GPIO_AF_Set(GPIOA,10,7);//PA10,AF7  	   
	//����������
 	USART1->BRR=mantissa; 	//����������	 
	USART1->CR1&=~(1<<15); 	//����OVER8=0 
	
	USART1->CR1|=1<<3;  	//���ڷ���ʹ�� 
	USART1->CR3|=1<<7;      //ʹ�ܴ���1��DMA����
	
#if EN_USART1_RX		  	//���ʹ���˽���	
	USART1->CR1|=1<<2;  	//���ڽ���ʹ��
	USART1->CR3|=1<<6;      //ʹ�ܴ���1��DMA����  
	USART1->CR1|=1<<4;    	//ʹ�ܿ����ж�	    	
	MY_NVIC_Init(3,3,USART1_IRQn,2);//��2��������ȼ� 
#endif
	USART1->CR1|=1<<13;  	//����ʹ��
}								 

#if EN_USART1_RX

void USART1_IRQHandler(void)
{
	u8 temp;
	u16 len;
	
	if(USART1->SR&(1<<4))//��⵽��·����
	{
//����������IDLE��־λ
		temp = USART1->SR;
		temp = USART1->DR;
		
		DMA2_Stream5->CR &=~(1<<0); //�ر�DMA����,׼����������
		DMA2->HIFCR|=1<<11;			//���DMA2_Steam5������ɱ�־
		DMA2->HIFCR|=1<<9;			//���DMA2_Steam5��������־
	
		len = USART_LEN - (uint16_t)(DMA2_Stream5->NDTR);
		myDMAprintf(USART1,"len = %d,data: %s",len,USART1_RX_BUF);
	}
	mymemset(USART1_RX_BUF,0,(u32)len);
	MYDMA_Enable(DMA2_Stream5,(u32)USART1_RX_BUF,USART_LEN);//��ʼһ��DMA���䣡
}
#endif

//=======================================================================	USART2
//��ʼ��IO ����1
//pclk2:PCLK2ʱ��Ƶ��(Mhz)
//bound:������ 
void Init_USART2(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV@OVER8=0
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������@OVER8=0 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->AHB1ENR|=1<<0;   	//ʹ��PORTA��ʱ��  
	RCC->APB1ENR|=1<<17;  	//ʹ�ܴ���2ʱ�� 
	GPIO_Set(GPIOA,PIN2|PIN3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PA9,PA10,���ù���,�������
 	GPIO_AF_Set(GPIOA,2,7);	//PA2,AF7		TXD
	GPIO_AF_Set(GPIOA,3,7); //PA3,AF7		RXD  	   
	//����������
 	USART2->BRR=mantissa; 	//����������	 
	USART2->CR1&=~(1<<15); 	//����OVER8=0 

	USART2->CR1|=1<<3;  	//���ڷ���ʹ�� 
	USART2->CR3|=1<<7;      //ʹ�ܴ���1��DMA����
	
#if EN_USART2_RX		  	//���ʹ���˽���	
	USART2->CR1|=1<<2;  	//���ڽ���ʹ��
	USART2->CR3|=1<<6;      //ʹ�ܴ���1��DMA����  
	USART2->CR1|=1<<4;    	//ʹ�ܿ����ж�	    	
	MY_NVIC_Init(3,2,USART2_IRQn,2);//��2��������ȼ� 
#endif
	USART2->CR1|=1<<13;  	//����ʹ��
}

#if EN_USART2_RX   //���ʹ���˽���  
  
void USART2_IRQHandler(void)
{
	u8 temp;
	u16 len;
	
	if(USART2->SR&(1<<4))//��⵽��·����
	{
//����������IDLE��־λ
		temp = USART2->SR;
		temp = USART2->DR;
		
		DMA1_Stream5->CR &=~(1<<0); //�ر�DMA����,׼����������
		DMA1->HIFCR|=1<<11;			//���DMA1_Stream5������ɱ�־
		DMA1->HIFCR|=1<<9;			//���DMA1_Stream5��������־
	
		len = USART_LEN - (uint16_t)(DMA1_Stream5->NDTR);
		myDMAprintf(USART2,"len = %d,data: %s",len,USART2_RX_BUF);
	}
	mymemset(USART2_RX_BUF,0,(u32)len);
	MYDMA_Enable(DMA1_Stream5,(u32)USART2_RX_BUF,USART_LEN);//��ʼһ��DMA���䣡
} 
#endif										 


//=======================================================================	USART3

void Init_USART3(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV@OVER8=0
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������@OVER8=0 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->AHB1ENR|=1<<1;   	//ʹ��PORTB��ʱ��  
	RCC->APB1ENR|=1<<18;  	//ʹ�ܴ���3ʱ�� 
	GPIO_Set(GPIOB,PIN10|PIN11,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PB10,PB11,���ù���,�������
 	GPIO_AF_Set(GPIOB,10,7);	//PB10,AF7		TXD
	GPIO_AF_Set(GPIOB,11,7);	//PB11,AF7  	RXD   
	//����������
 	USART3->BRR=mantissa; 	//����������	 
	USART3->CR1&=~(1<<15); 	//����OVER8=0 
	
	USART3->CR1|=1<<3;  	//���ڷ���ʹ�� 
	USART3->CR3|=1<<7;      //ʹ�ܴ���3��DMA����
	
#if EN_USART3_RX		  	//���ʹ���˽���	
	USART3->CR1|=1<<2;  	//���ڽ���ʹ��
	USART3->CR3|=1<<6;      //ʹ�ܴ���3��DMA����  
	USART3->CR1|=1<<4;    	//ʹ�ܿ����ж�	    	
	MY_NVIC_Init(3,1,USART3_IRQn,2);//��2��������ȼ� 
#endif
	USART3->CR1|=1<<13;  	//����ʹ��
}

#if EN_USART3_RX   //���ʹ���˽���	  
  
void USART3_IRQHandler(void)
{
	u8 temp;
	u16 len;
	
	if(USART3->SR&(1<<4))//��⵽��·����
	{
//����������IDLE��־λ
		temp = USART3->SR;
		temp = USART3->DR;
		
		DMA1_Stream1->CR &=~(1<<0); //�ر�DMA����,׼����������
		DMA1->LIFCR|=1<<11;			//���DMA1_Stream1������ɱ�־
		DMA1->LIFCR|=1<<9;			//���DMA1_Stream1��������־
	
		len = USART_LEN - (uint16_t)(DMA1_Stream1->NDTR);
		myDMAprintf(USART3,"len = %d,data: %s",len,USART3_RX_BUF);
	}
	mymemset(USART3_RX_BUF,0,(u32)len);
	MYDMA_Enable(DMA1_Stream1,(u32)USART3_RX_BUF,USART_LEN);//��ʼһ��DMA���䣡
} 
#endif


//=======================================================================	UART4

void Init_UART4(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV@OVER8=0
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������@OVER8=0 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->AHB1ENR|=1<<0;   	//ʹ��PORTA��ʱ��  
	RCC->APB1ENR|=1<<19;  	//ʹ�ܴ���4ʱ�� 
	GPIO_Set(GPIOA,PIN0|PIN1,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PB10,PB11,���ù���,�������
 	GPIO_AF_Set(GPIOA,0,8);	//PA0,AF8	TXD
	GPIO_AF_Set(GPIOA,1,8);	//PA1,AF8  	RXD   
	//����������
 	UART4->BRR=mantissa; 	//����������	 
	UART4->CR1&=~(1<<15); 	//����OVER8=0 
	
	UART4->CR1|=1<<3;  	//���ڷ���ʹ�� 
	UART4->CR3|=1<<7;      //ʹ�ܴ���3��DMA����
	
#if EN_UART4_RX		  	//���ʹ���˽���	
	UART4->CR1|=1<<2;  	//���ڽ���ʹ��
	UART4->CR3|=1<<6;   //ʹ�ܴ���4��DMA����  
	UART4->CR1|=1<<4;   //ʹ�ܿ����ж�	    	
	MY_NVIC_Init(3,0,UART4_IRQn,2);//��2��������ȼ� 
#endif
	UART4->CR1|=1<<13;  	//����ʹ��
}

#if EN_UART4_RX   //���ʹ���˽���	  
  
void UART4_IRQHandler(void)
{
	u8 temp;
	u16 len;
	
	if(UART4->SR&(1<<4))//��⵽��·����
	{
//����������IDLE��־λ
		temp = UART4->SR;
		temp = UART4->DR;
		
		DMA1_Stream2->CR &=~(1<<0); //�ر�DMA����,׼����������
		DMA1->LIFCR|=1<<21;			//���DMA1_Stream2������ɱ�־
		DMA1->LIFCR|=1<<19;			//���DMA1_Stream2��������־
	
		len = USART_LEN - (uint16_t)(DMA1_Stream2->NDTR);
		myDMAprintf(UART4,"len = %d,data: %s",len,UART4_RX_BUF);
	}
	mymemset(UART4_RX_BUF,0,(u32)len);
	MYDMA_Enable(DMA1_Stream2,(u32)UART4_RX_BUF,USART_LEN);//��ʼһ��DMA���䣡
} 
#endif


//=======================================================================	UART5

void Init_UART5(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV@OVER8=0
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������@OVER8=0 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->AHB1ENR|=1<<2;   	//ʹ��PORTC��ʱ�� 
	RCC->AHB1ENR|=1<<3;   	//ʹ��PORTD��ʱ��	
	RCC->APB1ENR|=1<<20;  	//ʹ�ܴ���5ʱ�� 
	GPIO_Set(GPIOC,PIN12,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PB10,PB11,���ù���,�������
	GPIO_Set(GPIOD,PIN2,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PB10,PB11,���ù���,�������
 	GPIO_AF_Set(GPIOC,12,8);	//PC12,AF8	TXD
	GPIO_AF_Set(GPIOD,2,8);		//PD2, AF8  RXD   
	//����������
 	UART5->BRR=mantissa; 	//����������	 
	UART5->CR1&=~(1<<15); 	//����OVER8=0 
	
	UART5->CR1|=1<<3;  	//���ڷ���ʹ�� 
	UART5->CR3|=1<<7;      //ʹ�ܴ���3��DMA����
	
#if EN_UART5_RX		  	//���ʹ���˽���	
	UART5->CR1|=1<<2;  	//���ڽ���ʹ��
	UART5->CR3|=1<<6;   //ʹ�ܴ���4��DMA����  
	UART5->CR1|=1<<4;   //ʹ�ܿ����ж�	    	
	MY_NVIC_Init(2,3,UART5_IRQn,2);//��2��������ȼ� 
#endif
	UART5->CR1|=1<<13;  	//����ʹ��
}

#if EN_UART5_RX   //���ʹ���˽���	  
  
void UART5_IRQHandler(void)
{
	u8 temp;
	u16 len;
	
	if(UART5->SR&(1<<4))//��⵽��·����
	{
//����������IDLE��־λ
		temp = UART5->SR;
		temp = UART5->DR;
		
		DMA1_Stream0->CR &=~(1<<0); //�ر�DMA����,׼����������
		DMA1->LIFCR|=1<<5;			//���DMA1_Stream0������ɱ�־
		DMA1->LIFCR|=1<<3;			//���DMA1_Stream0��������־
	
		len = USART_LEN - (uint16_t)(DMA1_Stream0->NDTR);
		myDMAprintf(UART5,"len = %d,data: %s",len,UART5_RX_BUF);
	}
	mymemset(UART5_RX_BUF,0,(u32)len);
	MYDMA_Enable(DMA1_Stream0,(u32)UART5_RX_BUF,USART_LEN);//��ʼһ��DMA���䣡
} 
#endif

//=======================================================================	USART6

void Init_USART6(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV@OVER8=0
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������@OVER8=0 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->AHB1ENR|=1<<2;   	//ʹ��PORTC��ʱ��  
	RCC->APB2ENR|=1<<5;  	//ʹ�ܴ���6ʱ�� 
	GPIO_Set(GPIOC,PIN6|PIN7,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PB10,PB11,���ù���,�������
 	GPIO_AF_Set(GPIOC,6,8);	//PC6,AF8	TXD
	GPIO_AF_Set(GPIOC,7,8);	//PC7,AF8  	RXD   
	//����������
 	USART6->BRR=mantissa; 	//����������	 
	USART6->CR1&=~(1<<15); 	//����OVER8=0 
	
	USART6->CR1|=1<<3;  	//���ڷ���ʹ�� 
	USART6->CR3|=1<<7;      //ʹ�ܴ���6��DMA����
	
#if EN_USART6_RX		  	//���ʹ���˽���	
	USART6->CR1|=1<<2;  	//���ڽ���ʹ��
	USART6->CR3|=1<<6;      //ʹ�ܴ���3��DMA����  
	USART6->CR1|=1<<4;    	//ʹ�ܿ����ж�	    	
	MY_NVIC_Init(2,2,USART6_IRQn,2);//��2��������ȼ� 
#endif
	USART6->CR1|=1<<13;  	//����ʹ��
}

#if EN_USART6_RX   //���ʹ���˽���	  
  
void USART6_IRQHandler(void)
{
	u8 temp;
	u16 len;
	
	if(USART6->SR&(1<<4))//��⵽��·����
	{
//����������IDLE��־λ
		temp = USART6->SR;
		temp = USART6->DR;
		
		DMA2_Stream1->CR &=~(1<<0); //�ر�DMA����,׼����������
		DMA2->LIFCR|=1<<11;			//���DMA2_Stream1������ɱ�־
		DMA2->LIFCR|=1<<9;			//���DMA2_Stream1��������־
	
		len = USART_LEN - (uint16_t)(DMA2_Stream1->NDTR);
		myDMAprintf(USART6,"len = %d,data: %s",len,USART6_RX_BUF);
	}
	mymemset(USART6_RX_BUF,0,(u32)len);
	MYDMA_Enable(DMA2_Stream1,(u32)USART6_RX_BUF,USART_LEN);//��ʼһ��DMA���䣡
} 
#endif


/*------------------------------------------------
* ��������Status myDMAprintf(USART_TypeDef *USARTx, const char *format, ...)
* ��  �ܣ��� printf ���� 
* ��  ����*USARTx: ���ں�
	      *pString: ��ӡ����
	      ... : ����
* ����ֵ�� ״̬
------------------------------------------------*/
Status myDMAprintf(USART_TypeDef *USARTx, const char *format, ...)
{
	va_list args; 	
	u16 len;
	
	if(format == NULL)
		return 1;
	va_start(args, format);
	
	if(USARTx == USART1)
	{
		len = vsnprintf((char *)USART1_TX_BUF, USART_LEN, format, args);
		while(DMA2_Stream7_Flag != IDLE);
		MYDMA_Enable(DMA2_Stream7,(u32)USART1_TX_BUF,len);
		DMA2_Stream7_Flag = BUSY;
	}
	else if(USARTx == USART2)
	{
		len = vsnprintf((char *)USART2_TX_BUF, USART_LEN, format, args);
		while(DMA1_Stream6_Flag != IDLE);
		MYDMA_Enable(DMA1_Stream6,(u32)USART2_TX_BUF,len);
		DMA1_Stream6_Flag = BUSY;
	}	
	else if(USARTx == USART3)
	{
		len = vsnprintf((char *)USART3_TX_BUF, USART_LEN, format, args);
		while(DMA1_Stream3_Flag != IDLE);
		MYDMA_Enable(DMA1_Stream3,(u32)USART3_TX_BUF,len);
		DMA1_Stream3_Flag = BUSY;
	}	
	else if(USARTx == UART4)
	{
		len = vsnprintf((char *)UART4_TX_BUF, USART_LEN, format, args);
		while(DMA1_Stream4_Flag != IDLE);
		MYDMA_Enable(DMA1_Stream4,(u32)UART4_TX_BUF,len);
		DMA1_Stream4_Flag = BUSY;
	}	
	else if(USARTx == UART5)
	{
		len = vsnprintf((char *)UART5_TX_BUF, USART_LEN, format, args);
		while(DMA1_Stream7_Flag != IDLE);
		MYDMA_Enable(DMA1_Stream7,(u32)UART5_TX_BUF,len);
		DMA1_Stream7_Flag = BUSY;
	}	
	else if(USARTx == USART6)
	{
		len = vsnprintf((char *)USART6_TX_BUF, USART_LEN, format, args);
		while(DMA2_Stream6_Flag != IDLE);
		MYDMA_Enable(DMA2_Stream6,(u32)USART6_TX_BUF,len);
		DMA2_Stream6_Flag = BUSY;
	}
	else
		return 2;
	
	va_end(args);
	
	return 0;
}

/*------------------------------------------------
* ��������Status myDMAarray(USART_TypeDef *USARTx, u32 mar, u16 ndtr)
* ��  �ܣ�����DMA��ʽ��������
* ��  ����*USARTx: ���ں�
	      mar: �洢����ַ
		  ndtr: ���ݴ�����
* ����ֵ�� ״̬
------------------------------------------------*/
Status myDMAarray(USART_TypeDef *USARTx, u32 mar, u16 ndtr)
{
	if(ndtr == 0)
		return 1;
	
	if(USARTx == USART1)
	{	
		while(DMA2_Stream7_Flag != IDLE);
		MYDMA_Enable(DMA2_Stream7,mar,ndtr);
		DMA2_Stream7_Flag = BUSY;
	}
	else if(USARTx == USART2)
	{
		while(DMA1_Stream6_Flag != IDLE);
		MYDMA_Enable(DMA1_Stream6,mar,ndtr);
		DMA1_Stream6_Flag = BUSY;
	}	
	else if(USARTx == USART3)
	{
		while(DMA1_Stream3_Flag != IDLE);
		MYDMA_Enable(DMA1_Stream3,mar,ndtr);
		DMA1_Stream3_Flag = BUSY;
	}	
	else if(USARTx == UART4)
	{
		while(DMA1_Stream4_Flag != IDLE);
		MYDMA_Enable(DMA1_Stream4,mar,ndtr);
		DMA1_Stream4_Flag = BUSY;
	}	
	else if(USARTx == UART5)
	{
		while(DMA1_Stream7_Flag != IDLE);
		MYDMA_Enable(DMA1_Stream7,mar,ndtr);
		DMA1_Stream7_Flag = BUSY;
	}	
	else if(USARTx == USART6)
	{
		while(DMA2_Stream6_Flag != IDLE);
		MYDMA_Enable(DMA2_Stream6,mar,ndtr);
		DMA2_Stream6_Flag = BUSY;
	}
	else
		return 2;
	
	return 0;
}






