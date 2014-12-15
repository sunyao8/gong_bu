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
vu8	warn_volt_onlimt=25;//过压值 +400
vu8 grafnum=1,tempshuzhi,vernum=105,hguestnum=222,gonglvshishu=0;
vu16 dianya_zhi=0,wugongkvar=0;
vu32	dianliuzhi=0;
 vu8 ligt_time=16;
 u8 TR[]={1,2,3,4,5,6,8,10,12,16,20,24,30,40,50,60,80,100,120};

//////////////////////////////////////////////////////////////////////////////////	 
//本程序为控制器设计，未经许可，不得复制外传
//实验板栋达电子V3.0-1
//KEY 代码 PA11为显示板设置按键；PA12为手动投切开关	   
//修改日期:2013/3/13
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 济宁市栋达电子科技有限公司 2013-2023
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////  
								    
//按键初始化函数
void KEY_Init(void)
{
	RCC->APB2ENR|=1<<2;     //使能PORTA时钟
	GPIOA->CRH&=0XFFF00FFF;	//PA11 PA12设置成输入	  
	GPIOA->CRH|=0X00088000; 
	GPIOA->ODR|=1<11;		// 上拉
	GPIOA->ODR|=1<12;		  //上拉
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
		if(grafnum==1)/*在电压 界面设置  过压上限       */
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
	if(grafnum==2)/*在电流 界面设置变比值*/
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
	if(grafnum==5)/*在id 界面设置id号*/
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
									case 1:	//显示功率因数和电压值
										Clera_lcd();
										Graf_con_u(gonglvshishu,dianya_zhi);
										break;
									case 2:	//显示电流
										Clera_lcd();
										Graf_cuirrent(dianliuzhi);
										break;
									case 3:	//显示无功功率	 
										Clera_lcd();
										Graf_qkvar(wugongkvar);
										break;
									case 4:	//显示温度 
										Clera_lcd();
										Graf_temp(tempshuzhi);
										break;
					
									case 5:	//显示ID 
										Clera_lcd();
										Graf_id(hguestnum,id_num);
										break;
					
									case 6:	//显示VER 
										Clera_lcd();
										Graf_ver(vernum);
										break;
					
								}
					 	}
					 if(grafnum==1&&zhongduan_flag_warn_volt==0)/*在id 界面设置id号*/
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

						if(grafnum==5&&zhongduan_flag==0)/*在id 界面设置id号*/
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

	if(grafnum==1)/*在电压 界面设置过压值  */
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

	if(grafnum==2)/*在电流 界面设置变比值*/
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
	if(grafnum==5)/*在电流 界面设置变比值*/
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
								case 1:	//显示功率因数和电压值
									Clera_lcd();
									Graf_con_u(gonglvshishu,dianya_zhi);
									break;
								case 2:	//显示电流
									Clera_lcd();
									Graf_cuirrent(dianliuzhi);
									break;
								case 3:	//显示无功功率	 
									Clera_lcd();
									Graf_qkvar(wugongkvar);
									break;
								case 4:	//显示温度 
									Clera_lcd();
									Graf_temp(tempshuzhi);
									break;
				
								case 5:	//显示ID 
									Clera_lcd();
									Graf_id(hguestnum,id_num);
									break;
				
								case 6:	//显示VER 
									Clera_lcd();
									Graf_ver(vernum);
									break;
				
							}	

		}
}




















