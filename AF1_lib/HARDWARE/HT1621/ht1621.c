//#include <stm32f10x_lib.h>
#include "ht1621.h"
#include "delay.h"
#include "key.h"
#include "led.h"
	   
//u8 num456Seg[]={0x0d,0x07,0x00,0x06,0x0e,0x03,0x0a,0x07,0x03,0x06,0x0b,0x05,0x0f,0x05,0x00,0x07,0x0f,0x07,0x0b,0x07 };
//u8 num1237Seg[]={0x07,0x0d,0x06,0x00,0x03,0x0e,0x07,0x0a,0x06,0x03,0x05,0x0b,0x05,0x0f,0x07,0x00,0x07,0x0f,0x07,0x0b};
u8 num12345Seg[]={0x0F,0x0A,0x00,0x0A,0x0B,0x0C,0x09,0x0E,0x04,0x0E,0x0D,0x06,0x0F,0x06,0x08,0x0A,0x0F,0x0E,0x0D,0x0E};
u8 num67Seg[]={0x0A,0x0F,0x0A,0x00,0x0C,0x0B,0x0E,0x09,0x0E,0x04,0x06,0x0D,0x06,0x0F,0x0A,0x08,0x0E,0x0F,0x0E,0x0D};
u8 dotnum12345Seg[]={0x0F,0x0B,0x00,0x0B,0x0B,0x0D,0x09,0x0F,0x04,0x0F,0x0D,0x07,0x0F,0x07,0x08,0x0B,0x0F,0x0F,0x0D,0x0F};
u8 dotnum67Seg[]={0x0B,0x0F,0x0B,0x00,0x0D,0x0B,0x0F,0x09,0x0F,0x04,0x07,0x0D,0x07,0x0F,0x0B,0x08,0x0F,0x0F,0x0F,0x0D};	   

extern s8 L_C_flag;//�������Ա�׼����
extern u8 auto_on;
u8 ht1621_595=1;

//////////////////////////////////////////////////////////////////////////////////	 
//������Ϊ��������ƣ�δ�����ɣ����ø����⴫
//ʵ��嶰�����V3.0-1
//LCD����HT1621���� PB3Ϊ595RCLK;PB4Ϊ1621WR;PB5Ϊ1621CS;PB6Ϊ1621DATA��595SCLK	   
//�޸�����:2013/3/13
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �����ж�����ӿƼ����޹�˾ 2013-2023
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   

//��ʼ��PB4 PB5��PB6Ϊ�����.��ʹ����PEʱ��		    
//HT1621 IO��ʼ��
void HT1621_Init(void)
{		
	RCC->APB2ENR|=1<<3;    //ʹ��PORTBʱ��
											  

      	GPIOB->CRH&=0XFFFFFF0F;
	GPIOB->CRH|=0X00000030;//PB9

	GPIOB->CRL&=0X000FFFFF;
	GPIOB->CRL|=0X33300000;//PB 4 5 6�������

	GPIOB->ODR|=1<<9;      //PB9����� 
	GPIOB->ODR|=1<<5;      //PB5����� 
	GPIOB->ODR|=1<<6;      //PB6�����
	GPIOB->ODR|=1<<7;      //PB7����� 


	SendCmd(LCDOFF);
	SendCmd(BIAS);			
	SendCmd(SYSEN);
	SendCmd(LCDON);
}

void SendBit_1621(u8 data,u8 cnt)	 //data��cntλд��HT1621,��λ��ǰ
{
  u8 i;
  if(ht1621_595==1)
  	{
  	ht1621_595=0;
  for(i=0;i<cnt;i++)
  {
   if((data&0x80)==0)DATA=0;
   else DATA=1;
   WR=0;
   delay_us(1);
   WR=1;
   data<<=1;
  }
   ht1621_595=1;

}
}
void SendDataBit_1621(u8 data,u8 cnt)	 //data��cntλд��HT1621,��λ��ǰ
{
  u8 i;
  if(ht1621_595==1)
  	{
  	  	ht1621_595=0;
  for(i=0;i<cnt;i++)
  {
   if((data&0x01)==0)DATA=0;
   else DATA=1;
   WR=0;
   delay_us(1);
   WR=1;
   data>>=1;
  }
    	ht1621_595=1;

  	}
}


void SendCmd(u8 command)
{
   CS=0;
   SendBit_1621(0x80,3);
   SendBit_1621(command,9);
   CS=1;
}

void Write_1621(u8 addr,u8 data)
{
//	SendCmd(LCDOFF);
SendCmd(BIAS);			
	SendCmd(SYSEN);
	SendCmd(LCDON);
   CS=0;
   SendBit_1621(0xa0,3);
   SendBit_1621(addr<<2,6);
   SendDataBit_1621(data,4);
   CS=1;
}

void WriteAll_1621(u8 addr,u8 *p,u8 cnt)
{
   u8 i;
   //	SendCmd(LCDOFF);
   SendCmd(BIAS);			
	SendCmd(SYSEN);
	SendCmd(LCDON);
   CS=0;
   SendBit_1621(0xa0,3);
   SendBit_1621(addr<<2,6);
   for(i=0;i<cnt;i++,p++)
   {
   	   SendDataBit_1621(*p,4);
   }
   CS=1;
}

void Clera_lcd(void)
{
	 u8 t;
	// 	SendCmd(LCDOFF);
	 SendCmd(BIAS);			
	SendCmd(SYSEN);
	SendCmd(LCDON);
	 for(t=0;t<22;t++)
	 {
	   u8 i;
	   for(i=0;i<4;i++)
	   {		  
	   	Write_1621(t,0x00<<i);
	   }
	 }
}



void Graf_con_u(u8 cos,u16 volt)
{
	u8 temp,coszhengshu,cosshifen,cosbaifen;
	u16 voltbaiwei,voltshiwei,voltgewei;

		Write_1621(12,0x04);   //��ʾU(v)
	Write_1621(14,0x01);   //��ʾcos
	Write_1621(17,0x01);   //��ʾdot.
	Write_1621(15,0x01);   //logo
	   if(auto_on==1)
   Write_1621(6,0x01);	//��ʾauto
   if(auto_on==0)
   	   Write_1621(5,0x01);	//��ʾhand
	temp=cos;
if( L_C_flag==1)
{
	coszhengshu=temp/100;
	WriteAll_1621(16,dotnum12345Seg+2*coszhengshu,2);	//��ʾcos��������

}
if( L_C_flag==0)

{
	//Write_1621(17,0x04);	//��ʾcos��������
		Write_1621(17,0x05);   //��ʾdot.

}
	cosshifen=(temp%100)/10;
	WriteAll_1621(18,num12345Seg+2*cosshifen,2);	 	//��ʾcosʮ��λ����

	cosbaifen=(temp%10)%10;
	WriteAll_1621(20,num12345Seg+2*cosbaifen,2);	   //��ʾcos�ٷ�λ����
	


	voltbaiwei=volt/100;
	WriteAll_1621(2,num12345Seg+2*voltbaiwei,2);	  //��ʾvolt��λ����

	
	voltshiwei=(volt%100)/10;
	WriteAll_1621(10,num67Seg+2*voltshiwei,2);	  //��ʾvoltʮλ����

	voltgewei=volt%10;
	WriteAll_1621(8,num67Seg+2*voltgewei,2);	  //��ʾvolt��λ����


//	Write_1621(7,0x08);   //��ʾauto
//	Write_1621(16,0x08);   //BLOG
//	WriteAll_1621(4,num67Seg+8,2);	  //
//	WriteAll_1621(19,num67Seg+8,4);	  //��ʾvolt��λ����
}
void Graf_cuirrent(u32 current)
{
   u32 temp;
   u16 currshiwei,currgewei,currshifenwei,currbaifenwei,currqianfenwei;

	Write_1621(12,0x01);	//��ʾC-I(A)
//	Write_1621(13,0x08);	//��ʾdot4 .
	   if(auto_on==1)
   Write_1621(6,0x01);	//��ʾauto
   if(auto_on==0)
   	   Write_1621(5,0x01);	//��ʾhand
	Write_1621(15,0x01);   //logo

   temp=current;

   currshiwei=temp/10000;
   WriteAll_1621(16,num12345Seg+2*currshiwei,2);	//��ʾC-I(A)��������ʮλ

   currgewei=(temp%10000)/1000;
   WriteAll_1621(18,num12345Seg+2*currgewei,2);	//��ʾC-I(A)�������ָ�λ

   currshifenwei=(temp%1000)/100;
   WriteAll_1621(20,num12345Seg+2*currshifenwei,2);	//��ʾC-I(A)С������ʮ��λ

   currbaifenwei=(temp%100)/10;
   WriteAll_1621(0,num12345Seg+2*currbaifenwei,2);	//��ʾC-I(A)С�����ְٷ�λ

   currqianfenwei=temp%10;
   WriteAll_1621(2,num12345Seg+2*currqianfenwei,2);	//��ʾC-I(A)С������ǧ��λ

}

void Graf_qkvar(u16 qkvar)
{
   u32 temp;
   u16 qkvarshiwei,qkvargewei,qkvarshifenwei,qkvarbaifenwei,qkvarqianfenwei;

	Write_1621(15,0x05);	//��ʾQ(Kvar),logo
//	Write_1621(11,0x08);	//��ʾdot8 .
	   if(auto_on==1)
   Write_1621(6,0x01);	//��ʾauto
   if(auto_on==0)
   	   Write_1621(5,0x01);	//��ʾhand
	
   temp=qkvar;

   qkvarshiwei=temp/10000;
   WriteAll_1621(20,num12345Seg+2*qkvarshiwei,2);	//��ʾQ(Kvar)��������ʮλ

   qkvargewei=(temp%10000)/1000;
   WriteAll_1621(0,num12345Seg+2*qkvargewei,2);	//��ʾQ(Kvar)�������ָ�λ

   qkvarshifenwei=(temp%1000)/100;
   WriteAll_1621(2,num12345Seg+2*qkvarshifenwei,2);	//��ʾQ(Kvar)С������ʮ��λ

   qkvarbaifenwei=(temp%100)/10;
   WriteAll_1621(10,num67Seg+2*qkvarbaifenwei,2);	//��ʾQ(Kvar)С�����ְٷ�λ

   qkvarqianfenwei=temp%10;
   WriteAll_1621(8,num67Seg+2*qkvarqianfenwei,2);	//��ʾQ(Kvar)С������ǧ��λ

}

void Graf_temp(u8 temp)
{
	u8 tempbaiwei,tempshiwei,tempgewei;

	Write_1621(14,0x04);	//��ʾTEMP
	   if(auto_on==1)
   Write_1621(6,0x01);	//��ʾauto
   if(auto_on==0)
   	   Write_1621(5,0x01);	//��ʾhand
	Write_1621(15,0x01);   //logo

	tempbaiwei=temp/100;
	WriteAll_1621(2,num12345Seg+2*tempbaiwei,2);	//��ʾTEMP����λ

	tempshiwei=(temp%100)/10;
	WriteAll_1621(10,num67Seg+2*tempshiwei,2);	//��ʾTEMPʮ��λ

	tempgewei=temp%10;
	WriteAll_1621(8,num67Seg+2*tempgewei,2);	//��ʾTEMP����λ

}

void Graf_id(u8 hostguest,u8 id)
{
	u8 h_gbaiwei,h_gshiwei,h_ggewei,idbaiwei,idshiwei,idgewei;

	Write_1621(7,0x08);	//��ʾID
	   if(auto_on==1)
   Write_1621(6,0x01);	//��ʾauto
   if(auto_on==0)
   	   Write_1621(5,0x01);	//��ʾhand
	Write_1621(15,0x01);   //logo

	h_gbaiwei=hostguest/100;
	WriteAll_1621(16,num12345Seg+2*h_gbaiwei,2);	//��ʾ���Ű�λ��

	h_gshiwei=(hostguest%100)/10;
	WriteAll_1621(18,num12345Seg+2*h_gshiwei,2);	//��ʾ����ʮλ��

	h_ggewei=hostguest%10;
	WriteAll_1621(20,num12345Seg+2*h_ggewei,2);	//��ʾ���Ÿ�λ��

	idbaiwei=id/100;
	WriteAll_1621(2,num12345Seg+2*idbaiwei,2);	//��ʾID����λ

	idshiwei=(id%100)/10;
	WriteAll_1621(10,num67Seg+2*idshiwei,2);	//��ʾIDʮ��λ

	idgewei=id%10;
	WriteAll_1621(8,num67Seg+2*idgewei,2);	//��ʾID����λ

}

void Graf_ver(u8 ver)
{
   u8 verbaiwei,vershiwei,vergewei;
   
   //Write_1621(4,0x04);	//��ʾdot10.
   Write_1621(13,0x08);	//��ʾver
      if(auto_on==1)
   Write_1621(6,0x01);	//��ʾauto
   if(auto_on==0)
   	   Write_1621(5,0x01);	//��ʾhand
	Write_1621(15,0x01);   //logo

   verbaiwei=ver/100;
   WriteAll_1621(2,num12345Seg+2*verbaiwei,2);	//��ʾVER����λ

   vershiwei=(ver%100)/10;
   WriteAll_1621(10,num67Seg+2*vershiwei,2);	//��ʾVERʮ��λ

   vergewei=ver%10;
   WriteAll_1621(8,num67Seg+2*vergewei,2);	//��ʾVER����λ
}

void Graf_setid(u8 idnum)
{   
   u8 idnumbaiwei,idnumshiwei,idnumgewei;
  
   Write_1621(6,0x08);	//��ʾset
   Write_1621(7,0x08);	//��ʾID
   	Write_1621(15,0x01);   //logo

   idnumbaiwei=idnum/100;
   WriteAll_1621(2,num12345Seg+2*idnumbaiwei,2);	//��ʾidnum����λ

   idnumshiwei=(idnum%100)/10;
   WriteAll_1621(10,num67Seg+2*idnumshiwei,2);	//��ʾidnumʮ��λ

   idnumgewei=idnum%10;
   WriteAll_1621(8,num67Seg+2*idnumgewei,2);	//��ʾidnum����λ

}


/***********************************************/
/*
void Clera_lcd(void)
{
	 u8 t;
	 for(t=0;t<22;t++)
	 {
	   u8 i;
	   for(i=0;i<4;i++)
	   {		  
	   	Write_1621(t,0x00<<i);
	   }
	 }
}

void Graf_con_u(u8 cos,u16 volt)
{
	u8 temp,coszhengshu,cosshifen,cosbaifen;
	u16 voltbaiwei,voltshiwei,voltgewei;

	Write_1621(11,0x01);   //��ʾU(v)
	Write_1621(13,0x01);   //��ʾcos
	Write_1621(14,0x08);   //��ʾdot.
	Write_1621(17,0x08);   //��ʾauto
	
	temp=cos;
if( L_C_flag==1)
{
	coszhengshu=temp/100;
	WriteAll_1621(15,num1237Seg+2*coszhengshu,2);	//��ʾcos��������
}
if( L_C_flag==0)

{
	Write_1621(16,0x02);	//��ʾcos��������
}
	cosshifen=(temp%100)/10;
	WriteAll_1621(19,num1237Seg+2*cosshifen,2);	 	//��ʾcosʮ��λ����

	cosbaifen=(temp%10)%10;
	WriteAll_1621(17,num1237Seg+2*cosbaifen,2);	   //��ʾcos�ٷ�λ����

	voltbaiwei=volt/100;
	WriteAll_1621(2,num456Seg+2*voltbaiwei,2);	  //��ʾvolt��λ����

	voltshiwei=(volt%100)/10;
	WriteAll_1621(5,num456Seg+2*voltshiwei,2);	  //��ʾvoltʮλ����

	voltgewei=volt%10;
	WriteAll_1621(8,num1237Seg+2*voltgewei,2);	  //��ʾvolt��λ����

}

void Graf_cuirrent(u32 current)
{
   u32 temp;
   u16 currshiwei,currgewei,currshifenwei,currbaifenwei,currqianfenwei;

	Write_1621(12,0x01);	//��ʾC-I(A)
//	Write_1621(13,0x08);	//��ʾdot4 .
	Write_1621(17,0x08);	//��ʾauto

   temp=current;

   currshiwei=temp/10000;
   WriteAll_1621(15,num1237Seg+2*currshiwei,2);	//��ʾC-I(A)��������ʮλ

   currgewei=(temp%10000)/1000;
   WriteAll_1621(19,num1237Seg+2*currgewei,2);	//��ʾC-I(A)�������ָ�λ

   currshifenwei=(temp%1000)/100;
   WriteAll_1621(17,num1237Seg+2*currshifenwei,2);	//��ʾC-I(A)С������ʮ��λ

   currbaifenwei=(temp%100)/10;
   WriteAll_1621(0,num456Seg+2*currbaifenwei,2);	//��ʾC-I(A)С�����ְٷ�λ

   currqianfenwei=temp%10;
   WriteAll_1621(2,num456Seg+2*currqianfenwei,2);	//��ʾC-I(A)С������ǧ��λ

}

void Graf_qkvar(u16 qkvar)
{
   u32 temp;
   u16 qkvarshiwei,qkvargewei,qkvarshifenwei,qkvarbaifenwei,qkvarqianfenwei;

	Write_1621(10,0x01);	//��ʾQ(Kvar)
//	Write_1621(11,0x08);	//��ʾdot8 .
	Write_1621(17,0x08);	//��ʾauto

   temp=qkvar;

   qkvarshiwei=temp/10000;
   WriteAll_1621(17,num1237Seg+2*qkvarshiwei,2);	//��ʾQ(Kvar)��������ʮλ

   qkvargewei=(temp%10000)/1000;
   WriteAll_1621(0,num456Seg+2*qkvargewei,2);	//��ʾQ(Kvar)�������ָ�λ

   qkvarshifenwei=(temp%1000)/100;
   WriteAll_1621(2,num456Seg+2*qkvarshifenwei,2);	//��ʾQ(Kvar)С������ʮ��λ

   qkvarbaifenwei=(temp%100)/10;
   WriteAll_1621(5,num456Seg+2*qkvarbaifenwei,2);	//��ʾQ(Kvar)С�����ְٷ�λ

   qkvarqianfenwei=temp%10;
   WriteAll_1621(8,num1237Seg+2*qkvarqianfenwei,2);	//��ʾQ(Kvar)С������ǧ��λ

}

void Graf_temp(u8 temp)
{
	u8 tempbaiwei,tempshiwei,tempgewei;

	Write_1621(4,0x01);	//��ʾTEMP
	Write_1621(17,0x08);	//��ʾauto

	tempbaiwei=temp/100;
	WriteAll_1621(2,num456Seg+2*tempbaiwei,2);	//��ʾTEMP����λ

	tempshiwei=(temp%100)/10;
	WriteAll_1621(5,num456Seg+2*tempshiwei,2);	//��ʾTEMPʮ��λ

	tempgewei=temp%10;
	WriteAll_1621(8,num1237Seg+2*tempgewei,2);	//��ʾTEMP����λ

}

void Graf_id(u8 hostguest,u8 id)
{
	u8 h_gbaiwei,h_gshiwei,h_ggewei,idbaiwei,idshiwei,idgewei;

	Write_1621(7,0x04);	//��ʾID
	Write_1621(17,0x08);	//��ʾauto

	h_gbaiwei=hostguest/100;
	WriteAll_1621(15,num1237Seg+2*h_gbaiwei,2);	//��ʾ���Ű�λ��

	h_gshiwei=(hostguest%100)/10;
	WriteAll_1621(19,num1237Seg+2*h_gshiwei,2);	//��ʾ����ʮλ��

	h_ggewei=hostguest%10;
	WriteAll_1621(17,num1237Seg+2*h_ggewei,2);	//��ʾ���Ÿ�λ��

	idbaiwei=id/100;
	WriteAll_1621(2,num456Seg+2*idbaiwei,2);	//��ʾID����λ

	idshiwei=(id%100)/10;
	WriteAll_1621(5,num456Seg+2*idshiwei,2);	//��ʾIDʮ��λ

	idgewei=id%10;
	WriteAll_1621(8,num1237Seg+2*idgewei,2);	//��ʾID����λ

}

void Graf_ver(u8 ver)
{
   u8 verbaiwei,vershiwei,vergewei;
   
   Write_1621(4,0x04);	//��ʾdot10.
   Write_1621(10,0x02);	//��ʾver
   Write_1621(17,0x08);	//��ʾauto

   verbaiwei=ver/100;
   WriteAll_1621(2,num456Seg+2*verbaiwei,2);	//��ʾVER����λ

   vershiwei=(ver%100)/10;
   WriteAll_1621(5,num456Seg+2*vershiwei,2);	//��ʾVERʮ��λ

   vergewei=ver%10;
   WriteAll_1621(8,num1237Seg+2*vergewei,2);	//��ʾVER����λ
}

void Graf_setid(u8 idnum)
{   
   u8 idnumbaiwei,idnumshiwei,idnumgewei;
  
   Write_1621(1,0x08);	//��ʾset
   Write_1621(7,0x04);	//��ʾID
   
   idnumbaiwei=idnum/100;
   WriteAll_1621(2,num456Seg+2*idnumbaiwei,2);	//��ʾidnum����λ

   idnumshiwei=(idnum%100)/10;
   WriteAll_1621(5,num456Seg+2*idnumshiwei,2);	//��ʾidnumʮ��λ

   idnumgewei=idnum%10;
   WriteAll_1621(8,num1237Seg+2*idnumgewei,2);	//��ʾidnum����λ

}
*/
void HT595_Send_Byte(u8 state)
{                        
    u8 t; 
	if(ht1621_595==1)
		{
	ht1621_595=0;
		RCLK_595=0;		    
    for(t=0;t<8;t++)
    {    
		DATA=0;          
        if((state&0x80)==0)
		WR=0;
		else WR=1;
		delay_us(3);
		DATA=1;
        state<<=1; 	  

    }

	delay_us(10);
	RCLK_595=1;
		ht1621_595=1;
		}
}
