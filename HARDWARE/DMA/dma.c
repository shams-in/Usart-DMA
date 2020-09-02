#include "dma.h"
#include "usart.h"
#include "delay.h"

volatile DMA_Flag DMA2_Stream7_Flag = IDLE;		//USART1
volatile DMA_Flag DMA1_Stream6_Flag = IDLE;		//USART2
volatile DMA_Flag DMA1_Stream3_Flag = IDLE;		//USART3
volatile DMA_Flag DMA1_Stream4_Flag = IDLE;		//UART4
volatile DMA_Flag DMA1_Stream7_Flag = IDLE;		//UART5
volatile DMA_Flag DMA2_Stream6_Flag = IDLE;		//USART6

/*------------------------------------------------
* ��������void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u8 chx,u32 par,u32 mar,u16 ndtr,u8 dir)
* ��  �ܣ�����DMA
* ��  ����DMA_Streamx: DMA��������DMA1_Stream0~7/DMA2_Stream0~7��
	     chx: DMAͨ��ѡ�񣨷�Χ:0~7��
	     par: �����ַ
		 mar: �洢����ַ
		 ndtr: ���ݴ�����
		 dir�� ���ݴ��䷽��DMA_DIR_PeripheralToMemory / DMA_DIR_MemoryToPeripheral / DMA_DIR_MemoryToMemory��
* ����ֵ�� ��
------------------------------------------------*/
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u8 chx,u32 par,u32 mar,u16 ndtr,u8 dir)
{ 
	DMA_TypeDef *DMAx;
	u8 streamx;
	if((u32)DMA_Streamx>(u32)DMA2)//�õ���ǰstream������DMA2����DMA1
	{
		DMAx=DMA2;
		RCC->AHB1ENR|=1<<22;//DMA2ʱ��ʹ�� 
	}else 
	{
		DMAx=DMA1; 
 		RCC->AHB1ENR|=1<<21;//DMA1ʱ��ʹ�� 
	}
	while(DMA_Streamx->CR&0X01);//�ȴ�DMA������ 
	streamx=(((u32)DMA_Streamx-(u32)DMAx)-0X10)/0X18;		//�õ�streamͨ����
 	if(streamx>=6)DMAx->HIFCR|=0X3D<<(6*(streamx-6)+16);	//���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx>=4)DMAx->HIFCR|=0X3D<<6*(streamx-4);    //���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx>=2)DMAx->LIFCR|=0X3D<<(6*(streamx-2)+16);//���֮ǰ��stream�ϵ������жϱ�־
	else DMAx->LIFCR|=0X3D<<6*streamx;						//���֮ǰ��stream�ϵ������жϱ�־
	
	DMA_Streamx->PAR=par;		//DMA�����ַ
	DMA_Streamx->M0AR=mar;		//DMA�洢����ַ
	DMA_Streamx->NDTR=ndtr;		//n��������
	DMA_Streamx->CR=0;			//��ȫ����λCR�Ĵ���ֵ 

	switch(dir)
	{
		case DMA_DIR_PeripheralToMemory: //���赽�洢��ģʽ
			DMA_Streamx->CR&=~(1<<6);
			DMA_Streamx->CR&=~(1<<7);
			break;
		case DMA_DIR_MemoryToPeripheral: 
			DMA_Streamx->CR|=1<<6;
			DMA_Streamx->CR&=~(1<<7);
			break;
		case DMA_DIR_MemoryToMemory: 
			DMA_Streamx->CR&=~(1<<6);
			DMA_Streamx->CR|=1<<7;
			break;
		default: break;
	}
	DMA_Streamx->CR|=0<<8;		//��ѭ��ģʽ(��ʹ����ͨģʽ)
	DMA_Streamx->CR|=0<<9;		//���������ģʽ
	DMA_Streamx->CR|=1<<10;		//�洢������ģʽ
	DMA_Streamx->CR|=0<<11;		//�������ݳ���:8λ
	DMA_Streamx->CR|=0<<13;		//�洢�����ݳ���:8λ
	DMA_Streamx->CR|=1<<16;		//�е����ȼ�
	DMA_Streamx->CR|=0<<21;		//����ͻ�����δ���
	DMA_Streamx->CR|=0<<23;		//�洢��ͻ�����δ���
	DMA_Streamx->CR|=(u32)chx<<25;//ͨ��ѡ��
	//DMA_Streamx->FCR=0x21;	//FIFO���ƼĴ���

	if(DMA_Streamx == DMA2_Stream7)	//USART1����
	{
		DMA2_Stream7->CR|=1<<4;		//ʹ�ܴ�������ж�
		MY_NVIC_Init(2,1,DMA2_Stream7_IRQn,2);
	}
	else if(DMA_Streamx == DMA1_Stream6)//USART2����
	{
		DMA1_Stream6->CR|=1<<4;
		MY_NVIC_Init(2,0,DMA1_Stream6_IRQn,2);
	}
	else if(DMA_Streamx == DMA1_Stream3)//USART3����
	{
		DMA1_Stream3->CR|=1<<4;
		MY_NVIC_Init(1,3,DMA1_Stream3_IRQn,2);
	}
	else if(DMA_Streamx == DMA1_Stream4)//USART4����
	{
		DMA1_Stream4->CR|=1<<4;
		MY_NVIC_Init(1,2,DMA1_Stream4_IRQn,2);
	}
	else if(DMA_Streamx == DMA1_Stream7)//USART5����
	{
		DMA1_Stream7->CR|=1<<4;
		MY_NVIC_Init(1,1,DMA1_Stream7_IRQn,2);
	}
	else if(DMA_Streamx == DMA2_Stream6)//USART6����
	{
		DMA2_Stream6->CR|=1<<4;
		MY_NVIC_Init(1,0,DMA2_Stream6_IRQn,2);
	}
} 


/*------------------------------------------------
* ��������void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx, u32 mar, u16 ndtr)
* ��  �ܣ�����DMA����
* ��  ����DMA_Streamx: DMA��������DMA1_Stream0~7 / DMA2_Stream0~7��
		 mar: �洢����ַ
		 ndtr: ���ݴ�����
* ����ֵ�� ��
------------------------------------------------*/
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx, u32 mar, u16 ndtr)
{
	DMA_Streamx->CR&=~(1<<0); 	//�ر�DMA���� 
	while(DMA_Streamx->CR&0X1);	//ȷ��DMA���Ա�����  
	DMA_Streamx->M0AR=mar;		//DMA�洢����ַ
	DMA_Streamx->NDTR=ndtr;		//DMA���������� 
	DMA_Streamx->CR|=1<<0;		//����DMA����
}	  

//��ӦUSART1����
void DMA2_Stream7_IRQHandler(void)
{
	if((DMA2->HISR&(1<<27)))
	{
		DMA2->HIFCR|=1<<27;
		DMA2_Stream7_Flag = IDLE;
	}
}

void DMA1_Stream6_IRQHandler(void)
{
	if((DMA1->HISR&(1<<21)))
	{
		DMA1->HIFCR|=1<<21;
		DMA1_Stream6_Flag = IDLE;
	}
}

void DMA1_Stream3_IRQHandler(void)
{
	if((DMA1->LISR&(1<<27)))
	{
		DMA1->LIFCR|=1<<27;
		DMA1_Stream3_Flag = IDLE;
	}
}

void DMA1_Stream4_IRQHandler(void)
{
	if((DMA1->HISR&(1<<5)))
	{
		DMA1->HIFCR|=1<<5;
		DMA1_Stream4_Flag = IDLE;
	}
}

void DMA1_Stream7_IRQHandler(void)
{
	if((DMA1->HISR&(1<<27)))
	{
		DMA1->HIFCR|=1<<27;
		DMA1_Stream7_Flag = IDLE;
	}
}

void DMA2_Stream6_IRQHandler(void)
{
	if((DMA2->HISR&(1<<21)))
	{
		DMA2->HIFCR|=1<<21;
		DMA2_Stream6_Flag = IDLE;
	}
}


