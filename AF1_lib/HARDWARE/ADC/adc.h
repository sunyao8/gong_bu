#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//ADC ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/7
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

#define ADC_CH1  1 //ͨ��0 ��ѹ����
#define ADC_CH4  4 //ͨ��1 ��������
#define ADC_CH5  5 //ͨ��2 ����1�¶Ȳ���
#define ADC_CH6  6 //ͨ��3 ����2�¶Ȳ��� 	
void Adc_Init(void);
u16  Get_Adc(u8 ch); 
u16 Get_Adc_Average(u8 ch,u8 times); 
u16 Get_Adc_13_5(u8 ch);   
u16 Get_Adc_Average_13_5(u8 ch,u8 times);
u16 Get_Adc2(u8 ch);
u16 Get_Adc2_Average(u8 ch,u8 times);

#endif 
