#include "led.h" 
	 

//��ʼ��PF9��PF10Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{    	 
	RCC->AHB1ENR|=1<<0;//ʹ��PORTAʱ�� 
	GPIO_Set(GPIOA,PIN6|PIN7,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU); //PF9,PF10����
	LED0=1;//LED0�ر�
	LED1=1;//LED1�ر�
}






