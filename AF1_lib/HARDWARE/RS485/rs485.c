#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//RS485���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////




 		  
  
										 
//��ʼ��IO ����2
//pclk1:PCLK1ʱ��Ƶ��(Mhz)
//bound:������	  
void RS485_Init(u32 bound)
{  
    GPIO_InitTypeDef GPIO_InitStructure;
  	USART_InitTypeDef USART_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOG, ENABLE);//ʹ��GPIOA,Gʱ��	 ���������� 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);	 //����������
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				 //PG9�˿�����,����������
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				 //����������
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_Init(GPIOG, &GPIO_InitStructure);		  //����������
 		GPIO_Init(GPIOB, &GPIO_InitStructure);	   //������ʹ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	//PA2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);  

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,ENABLE);//��λ����2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,DISABLE);//ֹͣ��λ
 
	
 #ifdef EN_USART2_RX		  	//���ʹ���˽���
	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ���ݳ���
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;///��żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�ģʽ

    USART_Init(USART2, &USART_InitStructure); ; //��ʼ������
  
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //ʹ�ܴ���2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure); //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
 
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�
   
    USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ��� 

 #endif

 RS485_TX_EN=0;			//Ĭ��Ϊ����ģʽ

  
 
}






















