//#include <stm32f10x_lib.h>
#include "key.h"
#include "delay.h"
#include "ht1621.h"
#include "led.h"
//#include "exti.h"
#include "24cxx.h" 	
static u8 m=1;
u8 zhongduan_flag=1;
u8 zhongduan_flag_BT=1;
u8 zhongduan_flag_warn_volt=1;
vu8 id_num=1,BT_num=1;
vu8	warn_volt_onlimt=25;//��ѹֵ +400
vu8 grafnum=1,tempshuzhi,vernum=105,hguestnum=222,gonglvshishu=0;
vu16 dianya_zhi=0,wugongkvar=0;
vu32	dianliuzhi=0;
 vu8 ligt_time=16;
 u8 TR[]={1,2,3,4,5,6,8,10,12,16,20,24,30,40,50,60,80,100,120};

//////////////////////////////////////////////////////////////////////////////////	 
//������Ϊ��������ƣ�δ����ɣ����ø����⴫
//ʵ��嶰�����V3.0-1
//KEY ���� PA11Ϊ��ʾ�����ð�����PA12Ϊ�ֶ�Ͷ�п���	   
//�޸�����:2013/3/13
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �����ж�����ӿƼ����޹�˾ 2013-2023
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////  
								    
//������ʼ������
void KEY_Init(void)
{
	RCC->APB2ENR|=1<<2;     //ʹ��PORTAʱ��
	GPIOA->CRH&=0XFFF00FFF;	//PA11 PA12���ó�����	  
	GPIOA->CRH|=0X00088000; 
	GPIOA->ODR|=1<11;		// ����
	GPIOA->ODR|=1<12;		  //����
} 
extern status_box mystatus;

void key_idset(void)
{
	
	u8 h=0;
	if((KEY0==0)&&m)
	{
		{
	LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);
           ligt_time=16;	  // delay_us(10000);
	   }
	m=0;
		if(grafnum==1)/*�ڵ�ѹ ��������  ��ѹ����       */
		{
	 while(KEY0==0)
	   	{
	   	   delay_us(3000);//3000
		   h++;
		   if(h>=200)break;
	   
	   	}
			   if(h>=200)//180
			   {		
					zhongduan_flag_warn_volt=0;

						
				{

					Clera_lcd();
		
			   		Graf_set_warn_volt(warn_volt_onlimt+400);

						}	

			   }
		}
	if(grafnum==2)/*�ڵ��� �������ñ��ֵ*/
		{
	 while(KEY0==0)
	   	{
	   	   delay_us(3000);//3000
		   h++;
		   if(h>=200)break;
	   
	   	}
			   if(h>=200)//180
			   {		
					zhongduan_flag_BT=0;

						
				{

					Clera_lcd();
		
			   		Graf_setBT(TR[BT_num]*50);

						}	

			   }
		}
	if(grafnum==5)/*��id ��������id��*/
		{
	 while(KEY0==0)
	   	{
	   	   delay_us(3000);//3000
		   h++;
		   if(h>=200)break;
	   
	   	}
			   if(h>=200)//180
			   {		
					zhongduan_flag=0;						

						{

					Clera_lcd();
		
			   		Graf_setid(id_num);

						}
					

			   }
		}
			     if(h<200)
				   {  
				     if(zhongduan_flag==1&&zhongduan_flag_BT==1&&zhongduan_flag_warn_volt==1)
				      	{
					  		grafnum++;
					  		if(grafnum>6)grafnum=1;
					  
					    	  switch(grafnum)
								{				 
									case 1:	//��ʾ���������͵�ѹֵ
										Clera_lcd();
										Graf_con_u(gonglvshishu,dianya_zhi);
										break;
									case 2:	//��ʾ����
										Clera_lcd();
										Graf_cuirrent(dianliuzhi);
										break;
									case 3:	//��ʾ�޹�����	 
										Clera_lcd();
										Graf_qkvar(wugongkvar);
										break;
									case 4:	//��ʾ�¶� 
										Clera_lcd();
										Graf_temp(tempshuzhi);
										break;
					
									case 5:	//��ʾID 
										Clera_lcd();
										Graf_id(hguestnum,id_num);
										break;
					
									case 6:	//��ʾVER 
										Clera_lcd();
										Graf_ver(vernum);
										break;
					
								}
					 	}
					 if(grafnum==1&&zhongduan_flag_warn_volt==0)/*��id ��������id��*/
						{
					             ligt_time=16;
							warn_volt_onlimt=warn_volt_onlimt+5;
					  		if(warn_volt_onlimt>50)warn_volt_onlimt=0;
							Clera_lcd();
			   		   Graf_set_warn_volt(warn_volt_onlimt+400);
							AT24CXX_WriteOneByte(0x1000,warn_volt_onlimt);
						}

						if(grafnum==2&&zhongduan_flag_BT==0)
				      	{

                                              
					             ligt_time=16;
							BT_num++;
					  		if(BT_num>18)BT_num=0;
							Clera_lcd();
	   						Graf_setBT(TR[BT_num]*50);
							AT24CXX_WriteOneByte(0x0100,BT_num);					


						}

						if(grafnum==5&&zhongduan_flag==0)/*��id ��������id��*/
						{
					             ligt_time=16;
							id_num++;
					  		if(id_num>32)id_num=1;
							Clera_lcd();
	   						Graf_setid(id_num);
							AT24CXX_WriteOneByte(0xa000,id_num);
						}
				   }
	
	}
	else if(KEY0==1)
		{
                  	  // delay_us(10000);
			m=1;

	if(grafnum==1)/*�ڵ�ѹ �������ù�ѹֵ  */
		{
			 while(KEY0==1)
			 {
		   	   delay_us(3000);//2500
			   h++;
			   if(h>=250)break;
	   
	   		 } 
			   if(h>=250)//200
				 {
						  zhongduan_flag_warn_volt=1;
			   	}
			   
		}

	if(grafnum==2)/*�ڵ��� �������ñ��ֵ*/
		{
			 while(KEY0==1)
			 {
		   	   delay_us(3000);//2500
			   h++;
			   if(h>=250)break;
	   
	   		 } 
			   if(h>=250)//200
				 {
						  zhongduan_flag_BT=1;
			   	}
			   
		}
	if(grafnum==5)/*�ڵ��� �������ñ��ֵ*/
		{
			 while(KEY0==1)
			 {
		   	   delay_us(3000);//2500
			   h++;
			   if(h>=250)break;
	   
	   		 } 
			   if(h>=250)//200
				 {
						  zhongduan_flag=1;
			   	}
			   
		}
				     if(zhongduan_flag==1&&zhongduan_flag_BT==1&&zhongduan_flag_warn_volt==1)
	                              switch(grafnum)
							{				 
								case 1:	//��ʾ���������͵�ѹֵ
									Clera_lcd();
									Graf_con_u(gonglvshishu,dianya_zhi);
									break;
								case 2:	//��ʾ����
									Clera_lcd();
									Graf_cuirrent(dianliuzhi);
									break;
								case 3:	//��ʾ�޹�����	 
									Clera_lcd();
									Graf_qkvar(wugongkvar);
									break;
								case 4:	//��ʾ�¶� 
									Clera_lcd();
									Graf_temp(tempshuzhi);
									break;
				
								case 5:	//��ʾID 
									Clera_lcd();
									Graf_id(hguestnum,id_num);
									break;
				
								case 6:	//��ʾVER 
									Clera_lcd();
									Graf_ver(vernum);
									break;
				
							}	

		}
}




















