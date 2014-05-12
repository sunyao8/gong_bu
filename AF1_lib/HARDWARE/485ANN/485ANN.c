#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"
#include "485ANN.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include "key.h"
#include "math.h"
#include "stm32_dsp.h"
#include "stm32f10x_iwdg.h"
#include "wdg.h"

/***********************************************************************/
#define CPT_LL                                                    '^'
#define CONTROL                                                '/'

//u16 RS485_RX_BUF[64]; 		//½ÓÊÕ»º³å,×î´ó64¸ö×Ö½Ú
//½ÓÊÕµ½µÄÊý¾Ý³¤¶È
u32 RS485_RX_CNT=0;  
//Ä£Ê½¿ØÖÆ
 u16  dog_clock=20;
		 u16 K_BT=10;//µçÁ÷ÏµÊý

 OS_EVENT * RS485_MBOX,* RS485_STUTAS_MBOX,* RS485_RT;			//	rs485ÓÊÏäÐÅºÅÁ¿
 OS_EVENT *Heartbeat;			 //ÐÄÌøÐÅºÅÁ¿
OS_EVENT *master_led_task;
//OS_EVENT * sub_machine1_open;		//ÏÂÎ»»úÃüÁîÐÅºÅ
//OS_EVENT * sub_machine1_close;		//ÏÂÎ»»úÃüÁîÐÅºÅ
//OS_EVENT * sub_machine2_open;		//ÏÂÎ»»úÃüÁîÐÅºÅ
//OS_EVENT * sub_machine2_close;		//ÏÂÎ»»úÃüÁîÐÅºÅ

OS_EVENT *scan_slave;

u8 cont=0;//ÓÃÓÚ¸ü¸ÄÖ÷»úºÅµÄ¼Ç´ÎÊýÆ÷
u32 life_time_1=0;//´Ó»ú1¹¤×÷Ê±¼äµÄÖÐ¼ä±äÁ¿
u32 life_time_2=0;//´Ó»ú2¹¤×÷Ê±¼äµÄÖÐ¼ä±äÁ¿
u32 idle_time=0;//Ö÷»úÓÃÓÚÂÖÐÝµÄÊ±¼ä
u32 scan_time=0;//Ö÷»úÓÃÓÚ´ÓÐÂ±éÀú´Ó»úµÄµÄÊ±¼ä
u16 m1_opentime,m2_opentime,m1_closetime,m2_closetime;//´Ó»úÊ¹ÓÃ±äÁ¿
u8 true_worktime1_flag=0,true_worktime2_flag=0;//´Ó»úÊ¹ÓÃ±äÁ¿£¬ÓÃÓÚ±êÊ¶ÏÂÎ»»úÕæÕýÍ¸ÇÐµÄÊ±¼äµã
u8 turn_flag=1;//ÂÖÐÝÊ¹ÓÃ±äÁ¿
s8 turn_label_idle=0;//ÂÖÐÝÊ¹ÓÃ±äÁ¿
 u8 RT_FLAG=3;

box mybox;
status_box mystatus;
idle_list sort_idle_list_1[33];
idle_list sort_idle_list_2[33];
turn_node turn_idle_list[65];
busy_list sort_busy_list_1[33];
busy_list sort_busy_list_2[33];


status_list_node system_status_list[33];

u8 idle_done_nodelist_1[33];
u8 idle_done_nodelist_2[33];
u8 done_list1_flag=0,done_list2_flag=0;
u8 done_count_1=0,done_count_2=0;

u8 slave[30];

//u8 rs485buf[LEN_control];//·¢ËÍ¿ØÖÆÐÅÏ¢
u8 rs485buf[LEN_control];//·¢ËÍ¿ØÖÆÐÅÏ¢
u8 statusbuf[LEN_status];//·¢ËÍ×´Ì¬ÐÅÏ¢


u8 alarm_lock=0;
u8 phase_zhi=0;
/****************************************************************/
#define NPT 512            /* NPT = No of FFT point*/
#define FFT_NM NPT/2
#define PI2  6.28318530717959
#define TIME_TQ 1

long lBUFIN_V[NPT];         /* Complex input vector */
long lBUFOUT_V[FFT_NM];        /* Complex output vector */
long lBUFIN_I[NPT];         /* Complex input vector */
long lBUFOUT_I[FFT_NM];        /* Complex output vector */
double angle[4]; 



/**********************²âÊÔÎÞ¹¦¹¦ÂÊ Êý¾Ý*************************************/


/************************************************************/
u16 wugong_95,wugong_computer;

extern u8 id_num;
extern u8 grafnum,tempshuzhi,gonglvshishu;
extern u16 dianya_zhi,	wugongkvar,k;
extern u32	dianliuzhi;

s8 L_C_flag=1;//¸ÐÐÔÈÝÐÔ±ê×¼±äÁ¿
/*****************************************************/
extern status_box mystatus;
extern u8 ligt_time;
u8 rework_time[2];
extern u8 hguestnum;
 void TIM4_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //Ê±ÖÓÊ¹ÄÜ
	
	//¶¨Ê±Æ÷TIM4³õÊ¼»¯
	TIM_TimeBaseStructure.TIM_Period = arr; //ÉèÖÃÔÚÏÂÒ»¸ö¸üÐÂÊÂ¼þ×°Èë»î¶¯µÄ×Ô¶¯ÖØ×°ÔØ¼Ä´æÆ÷ÖÜÆÚµÄÖµ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //ÉèÖÃÓÃÀ´×÷ÎªTIMxÊ±ÖÓÆµÂÊ³ýÊýµÄÔ¤·ÖÆµÖµ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ÉèÖÃÊ±ÖÓ·Ö¸î:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIMÏòÉÏ¼ÆÊýÄ£Ê½
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //¸ù¾ÝÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯TIMxµÄÊ±¼ä»ùÊýµ¥Î»
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //Ê¹ÄÜÖ¸¶¨µÄTIM4ÖÐ¶Ï,ÔÊÐí¸üÐÂÖÐ¶Ï

	//ÖÐ¶ÏÓÅÏÈ¼¶NVICÉèÖÃ
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4ÖÐ¶Ï
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //ÏÈÕ¼ÓÅÏÈ¼¶0¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //´ÓÓÅÏÈ¼¶3¼¶
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQÍ¨µÀ±»Ê¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);  //³õÊ¼»¯NVIC¼Ä´æÆ÷


	TIM_Cmd(TIM4, ENABLE);  //Ê¹ÄÜTIMx					 
}
 
 void TIM4_IRQHandler(void)   //TIM4ÖÐ¶Ï
{	 
	OSIntEnter();   
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //¼ì²éTIM4¸üÐÂÖÐ¶Ï·¢ÉúÓë·ñ
		{	  
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //Çå³ýTIMx¸üÐÂÖÐ¶Ï±êÖ¾
	if(mybox.master==0)	
		{
		
		if(dog_clock==0)
		   { 
		   // LED0=!LED0;
			turn_master_id(mybox.myid);
			  cont++;
			}
			if(dog_clock>0){dog_clock--;cont=1;}
		 }
	if(ligt_time>0)ligt_time--;

	if(ligt_time==0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);

/*
{
	if(mybox.master==1)	
				{
                                   scan_time++;
					if(scan_time==65530)scan_time=0;			   

			       }
	
	
		 		 if (mystatus.work_status[0]==1)  //¹¤×÷Ê±¼äµÄ¼ÆÊ±
		    {  
		        life_time_1++;
		       if(life_time_1==5)
			   { mystatus.work_time[0]++;
			     life_time_1=0;
				  }
			        if(mystatus.work_time[0]==37)mystatus.work_time[0]=39;
				  if(mystatus.work_time[0]==254)mystatus.work_time[0]=0;
			   }
		 	
		 //if(mystatus.work_status[1]==1&&true_worktime2_flag==1)
		 if(mystatus.work_status[1]==1)
		 	{ 
		 	life_time_2++;
              if(life_time_2==5)
			  	{  mystatus.work_time[1]++;
			       life_time_2=0;
              	         }   
                   if(mystatus.work_time[1]==37)mystatus.work_time[1]=39;
			 if(mystatus.work_time[1]==254)mystatus.work_time[1]=0;	   
			    
		     }
		  // if (mystatus.work_status[0]==0&&true_worktime1_flag==0)  //¹¤×÷Ê±¼äÇåÁã
		   		   if (mystatus.work_status[0]==0)  //¹¤×÷Ê±¼äÇåÁã
		    {   mystatus.work_time[0]=0;
		   }
		   // if (mystatus.work_status[1]==0&&true_worktime2_flag==0)  //¹¤×÷Ê±¼äÇåÁã
		    		    if (mystatus.work_status[1]==0)  //¹¤×÷Ê±¼äÇåÁã
		    {   mystatus.work_time[1]=0;
			}

			if(mybox.master==1)
		       {  idle_time++;
			if(idle_time==65535)idle_time=0;
			}

}	
*/

	}
   	OSIntExit();  
}

 void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //Ê±ÖÓÊ¹ÄÜ
	
	//¶¨Ê±Æ÷TIM4³õÊ¼»¯
	TIM_TimeBaseStructure.TIM_Period = arr; //ÉèÖÃÔÚÏÂÒ»¸ö¸üÐÂÊÂ¼þ×°Èë»î¶¯µÄ×Ô¶¯ÖØ×°ÔØ¼Ä´æÆ÷ÖÜÆÚµÄÖµ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //ÉèÖÃÓÃÀ´×÷ÎªTIMxÊ±ÖÓÆµÂÊ³ýÊýµÄÔ¤·ÖÆµÖµ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ÉèÖÃÊ±ÖÓ·Ö¸î:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIMÏòÉÏ¼ÆÊýÄ£Ê½
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //¸ù¾ÝÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯TIMxµÄÊ±¼ä»ùÊýµ¥Î»
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //Ê¹ÄÜÖ¸¶¨µÄTIM4ÖÐ¶Ï,ÔÊÐí¸üÐÂÖÐ¶Ï

	//ÖÐ¶ÏÓÅÏÈ¼¶NVICÉèÖÃ
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM4ÖÐ¶Ï
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //ÏÈÕ¼ÓÅÏÈ¼¶0¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //´ÓÓÅÏÈ¼¶3¼¶
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQÍ¨µÀ±»Ê¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);  //³õÊ¼»¯NVIC¼Ä´æÆ÷


	TIM_Cmd(TIM3, ENABLE);  //Ê¹ÄÜTIMx					 
}
 
 void TIM3_IRQHandler(void)   //TIM4ÖÐ¶Ï
{	 static u8 count_rework[2];
	OSIntEnter();   
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //¼ì²éTIM4¸üÐÂÖÐ¶Ï·¢ÉúÓë·ñ
		{	  
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //Çå³ýTIMx¸üÐÂÖÐ¶Ï±êÖ¾
                    // IWDG_Feed();
 if(rework_time[0]==1)
 	{
 	count_rework[0]++;
	if(count_rework[0]==10)
		{
count_rework[0]=0;
rework_time[0]=0;
	  }
	}
  if(rework_time[1]==1)
 	{
 	count_rework[1]++;
	if(count_rework[1]==10)
		{
count_rework[1]=0;
rework_time[1]=0;
	  }
	}

	}
   	OSIntExit();  
}
		void USART2_IRQHandler(void)
{
 
	u8 RS485_RX_BUF[512];
	#ifdef OS_TICKS_PER_SEC	 	//Èç¹ûÊ±ÖÓ½ÚÅÄÊý¶¨ÒåÁË,ËµÃ÷ÒªÊ¹ÓÃucosIIÁË.
	OSIntEnter();    
      #endif
 		
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //½ÓÊÕµ½Êý¾Ý
	{	 
	 			 
		 RS485_RX_BUF[RS485_RX_CNT++]=USART_ReceiveData(USART2); 	//¶ÁÈ¡½ÓÊÕµ½µÄÊý¾Ý

/***************************************************************/			
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='&'){RS485_RX_BUF[0]='&'; RS485_RX_CNT=1;}
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='*')
		{				
		           RS485_RX_CNT=0;
			if(RS485_RX_BUF[1]=='!'){OSMboxPost(RS485_RT,(void*)&RS485_RX_BUF);}

		if(RS485_RX_BUF[1]=='#')
			{
/*
				if(mybox.master==1)//Èç¹ûÊÇÖ÷»ú£¬¼ì²é´Ó»úÖÐÓÐÎÞidºÅ±ÈÖ÷»úÐ¡µÄ£¬Èç¹ûÓÐ½»»»Ö÷´Ó»ú
					{

				if(mybox.myid>RS485_RX_BUF[2])
						{
					mybox.master=0;
					     hguestnum=222;
					dog_clock=20;	 
					OSTaskResume(Receive_TASK_PRIO );//Æô¶¯´Ó»úÈÎÎñ×´Ì¬	 
                      		OSTaskSuspend(MASTER_TASK_PRIO );//¹ÒÆðÖ÷»úÈÎ×´Ì¬.
					}
				}			

				else
					*/
					OSMboxPost(RS485_STUTAS_MBOX,(void*)&RS485_RX_BUF);

		     }
               else 
			   	{
			   	/*
			   	if(mybox.master==1)//Èç¹ûÊÇÖ÷»ú£¬¼ì²éÆäËûÓÐÎÞÖ÷»úÖÐidºÅ±È±¾Ö÷»úÐ¡µÄ£¬Èç¹ûÓÐ½»»»Ö÷´Ó»ú
					{

				if(mybox.myid>RS485_RX_BUF[1])
						{
					mybox.master=0;
					     hguestnum=222;
					dog_clock=20;	 
					OSTaskResume(Receive_TASK_PRIO );//Æô¶¯´Ó»úÈÎÎñ×´Ì¬	 
                      		OSTaskSuspend(MASTER_TASK_PRIO );//¹ÒÆðÖ÷»úÈÎ×´Ì¬.
					}
				}
				else 
					*/
					OSMboxPost(RS485_MBOX,(void*)&RS485_RX_BUF);

			       }

		} 
/************************·Ö²¹ÊÕ¼¯×´Ì¬Ð­Òé£¬¹²²¹ÏµÍ³ ÊÕµ½ºóÉáÆú ²¢Çå¿ÕRS485_RX_CNT ***********************************/		
			if(RS485_RX_BUF[RS485_RX_CNT-1]=='%'){RS485_RX_BUF[0]='%'; RS485_RX_CNT=1;}
		if(RS485_RX_BUF[RS485_RX_CNT-1]==')')
	{
				RS485_RX_CNT=0;

		}
/*************************·Ö²¹ÏµÍ³ÏÔÊ¾Êý¾ÝÐ­Òé£¬¹²²¹ÏµÍ³ ÊÕµ½ºóÉáÆú ²¢Çå¿ÕRS485_RX_CNT **********************************/		
		
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='$'){RS485_RX_BUF[0]='$'; RS485_RX_CNT=1;}
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='?')
		{
				RS485_RX_CNT=0;				
		}
			if(RS485_RX_BUF[RS485_RX_CNT-1]=='-'){RS485_RX_BUF[0]='-'; RS485_RX_CNT=1;}
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='=')
		{			RS485_RX_CNT=0;
/***********************Èç¹ûÓÐ·Ö²¹»úÆ÷£¬¹²²¹Èç¹ûÊÇÖ÷»ú£¬¾ÍÒª½øÐÐÇÐ»»Ö÷»ú************************/
				if(mybox.master==1)
					{
					mybox.master=0;
					     hguestnum=222;
						 dog_clock=20;
					OSTaskResume(Receive_TASK_PRIO );//Æô¶¯´Ó»úÈÎÎñ×´Ì¬	 
                      		OSTaskSuspend(MASTER_TASK_PRIO );//¹ÒÆðÖ÷»úÈÎ×´Ì¬.
				}
/*****************************Èç¹ûÓÐ·Ö²¹»úÆ÷£¬¹²²¹Èç¹ûÊÇÖ÷»ú£¬¾ÍÒª½øÐÐÇÐ»»Ö÷»úend******************************************/

		}	
/***********************************************************/		
		if(RS485_RX_CNT>=512)RS485_RX_CNT=0;
		 }  	
	#ifdef OS_TICKS_PER_SEC	 	//Èç¹ûÊ±ÖÓ½ÚÅÄÊý¶¨ÒåÁË,ËµÃ÷ÒªÊ¹ÓÃucosIIÁË.
	OSIntExit();  											 
#endif

} 

void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	RS485_TX_EN=1;			//ÉèÖÃÎª·¢ËÍÄ£Ê½
  	for(t=0;t<len;t++)		//Ñ­»··¢ËÍÊý¾Ý
	{		   
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	  
		USART_SendData(USART2,buf[t]);
		delay_us(100);
	}	 
 
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);		
	RS485_RX_CNT=0;	  
	RS485_TX_EN=0;				//ÉèÖÃÎª½ÓÊÕÄ£Ê½	

}

void initmybox()//³õÊ¼»¯×ÔÉíÐÅÏ¢
{  	 
  
  mybox.master=0;
 mybox.start='&';
mybox.myid=AT24CXX_ReadOneByte(0x0010);
///mybox.myid=1;
 mybox.source=0;
 mybox.destination=0;
 mybox.send=0;
 mybox.relay=0;
 mybox.message=0;
 mybox.end='*';	
						
}

void turn_master_id(u8 id)//¸Ä±äµ±Ç°Õû¸öÏµÍ³ÖÐÖ÷»úµÄIDºÅ
{
   u8 flag=0;
	{ 
	  flag=cont;
      if(id==(flag)){
	//delay_time(2);
         mybox.master=1;
	    OSTaskResume(MASTER_TASK_PRIO);
		cont=1;
	  }
	 //  LED1=!LED1;
	  
      }
   }






 int rs485_trans_order(u8 *tx_r485)//½âÎöÓÉÖ÷»ú·¢ËÍ¹ýÀ´µÄÐÅºÅ£¬²¢·¢ËÍ¸øÏÂÎ»»ú
{
 	         
 
if(tx_r485[8]==CPT_LL)
{
    dianya_zhi=comp_16(tx_r485[1],tx_r485[2]);
  dianliuzhi=comp_16(tx_r485[3],tx_r485[4]);
  wugongkvar=comp_16(tx_r485[5],tx_r485[6]);
  gonglvshishu=tx_r485[7];
  L_C_flag=tx_r485[9];
return 0;

}



if(tx_r485[8]==CONTROL)

{
   if(mybox.myid==tx_r485[2]||tx_r485[2]==0)//ÅÐ¶ÏÊÇ·ñÊÇ·¢¸ø±¾»úµÄÐÅÏ¢»òÊÇ¹ã²¥ÐÅÏ¢
   	{
   	 mybox.source=tx_r485[1];
   	 mybox.send=tx_r485[3];
     mybox.relay=tx_r485[4];
     mybox.message=tx_r485[5];
     return 1;
   	}
   
}

return 0;

}

 void order_trans_rs485(u8 source,u8 destination, u8 send,u8 relay,u8 message,u8 ctr)//Ö÷»ú³ÌÐò£¬Ö÷»úÃüÁî½âÎö³ÉRS485ÐÅÏ¢£¬·¢ËÍ¸øÄ¿µÄ´Ó»ú
{   OS_CPU_SR cpu_sr=0;
   OS_ENTER_CRITICAL();
    
   {  
  if(ctr==CONTROL)
    {
      rs485buf[0]='&';//Ð­ÒéÍ·
	rs485buf[1]=source;
	rs485buf[2]=destination;
	rs485buf[3]=send;
	rs485buf[4]=relay;
	rs485buf[5]=message;
	rs485buf[6]=0;
	rs485buf[7]=gonglvshishu;	
	rs485buf[8]=ctr;
	rs485buf[9]='*';//Ð­ÒéÎ²
	RS485_Send_Data(rs485buf,10);//·¢ËÍ5¸ö×Ö½Ú
	   	if(destination==source){mybox.send=send;slave_control(relay, message);}//Èç¹ûÐÅÏ¢·¢¸øµÄ×Ô¼º

  }
	
if(ctr==CPT_LL )

		{
      rs485buf[0]='&';//Ð­ÒéÍ·	
	rs485buf[1]=(dianya_zhi & (uint16_t)0x00FF);
	rs485buf[2]=((dianya_zhi & (uint16_t)0xFF00)>>8);
	rs485buf[3]=(dianliuzhi& (uint16_t)0x00FF);
	rs485buf[4]=((dianliuzhi& (uint16_t)0xFF00)>>8);
	rs485buf[5]=(wugongkvar& (uint16_t)0x00FF);
	rs485buf[6]=((wugongkvar& (uint16_t)0xFF00)>>8);
	rs485buf[7]=gonglvshishu;	
	rs485buf[8]=ctr;
	rs485buf[9]=L_C_flag;	
	rs485buf[10]='*';//Ð­ÒéÎ²
	RS485_Send_Data(rs485buf,11);//·¢ËÍ5¸ö×Ö½Ú
	  // 	if(destination==source){mybox.send=send;slave_control(relay, message);}//Èç¹ûÐÅÏ¢·¢¸øµÄ×Ô¼º

    	}
//#endif
}
	OS_EXIT_CRITICAL();	
}

u16 comp_16(u16 a,u16 b)
{
u16 value=0;
value=((a&0x00FF)+((b<<8)&0xFF00));
return value;
}

void heartbeat(u8 t)
{	u8 i;
for(i=0;i<=t;i++)
		{	
	       order_trans_rs485(mybox.myid,0,0,0,0,CPT_LL);
		    delay_us(10000);
		  // LCD_ShowxNum(60+i*32,190,0,7,16,0X80);
		}	
}










void led_on_off(u8 on_off,u8 j) //²ÎÊýÖµ1 Îª´ò¿ªled £¬0Îª¹Ø±Õled
{
}


/*****************************»ØÀ¡ÐÅÏ¢º¯Êý********************************************/


void init_mystatus(u8 size_1,u8 size_2,u8 work_status_1,u8 work_status_2,u8 work_time_1,u8 work_time_2)
{
mystatus.myid= mybox.myid;
mystatus.size[0]=size_1;
mystatus.size[1]=size_2;
mystatus.work_status[0]=work_status_1;
mystatus.work_status[1]=work_status_2;
mystatus.work_time[0]=work_time_1;
mystatus.work_time[1]=work_time_2;
}



void set_now_mystatus(u8 myid,u8 size_1,u8 size_2,u8 work_status_1,u8 work_status_2,u8 work_time_1,u8 work_time_2)
{
mystatus.myid=myid;
mystatus.size[0]=size_1;
mystatus.size[1]=size_2;
mystatus.work_status[0]=work_status_1;
mystatus.work_status[1]=work_status_2;
mystatus.work_time[0]=work_time_1;
mystatus.work_time[1]=work_time_2;

}



 void status_trans_rs485(status_box *mystatus)//´Ó»ú³ÌÐò
{  	 OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
    statusbuf[0]='&';
	statusbuf[1]='#';
	statusbuf[2]=mystatus->myid;
	statusbuf[3]=mystatus->size[0];
	statusbuf[4]=mystatus->size[1];
	statusbuf[5]=mystatus->work_status[0];
	statusbuf[6]=mystatus->work_status[1];
	statusbuf[7]=mystatus->work_time[0];
	statusbuf[8]=mystatus->work_time[1];
	statusbuf[9]='*';
	RS485_Send_Data(statusbuf,10);//·¢ËÍ10¸ö×Ö½Ú
	OS_EXIT_CRITICAL();	
}
 void status_trans_rs485_scantask(status_box *mystatus)//´Ó»ú³ÌÐò
{  	 OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
    statusbuf[0]='&';
	statusbuf[1]='#';
	statusbuf[2]=mystatus->myid;
	statusbuf[3]=mystatus->work_status[0];
	statusbuf[4]=mystatus->work_status[1];
	statusbuf[5]='*';
	RS485_Send_Data(statusbuf,6);//·¢ËÍ10¸ö×Ö½Ú
	OS_EXIT_CRITICAL();	
}
 void status_trans_rs485_dis(status_box *mystatus)//´Ó»ú³ÌÐò
{  	 OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
    statusbuf[0]='&';
	statusbuf[1]='#';
	statusbuf[2]=mystatus->myid;
	statusbuf[3]=mystatus->size[0];
	statusbuf[4]=mystatus->size[1];
	statusbuf[5]=mystatus->work_status[0];
	statusbuf[6]=mystatus->work_status[1];

	statusbuf[7]='*';
	RS485_Send_Data(statusbuf,8);//·¢ËÍ10¸ö×Ö½Ú
	OS_EXIT_CRITICAL();	
}

 void status_trans_rs485_RT()//´Ó»ú³ÌÐò
{  	 OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
    statusbuf[0]='&';
	statusbuf[1]='+';

	statusbuf[2]='*';
	RS485_Send_Data(statusbuf,3);//·¢ËÍ10¸ö×Ö½Ú
	OS_EXIT_CRITICAL();	
}

 void status_trans_rs485_comm_RT()//´Ó»ú³ÌÐò
{  	 OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
    statusbuf[0]='&';
	statusbuf[1]='!';

	statusbuf[2]='*';
	RS485_Send_Data(statusbuf,3);//·¢ËÍ10¸ö×Ö½Ú
	OS_EXIT_CRITICAL();	
}

















u8 sort_idlenode_list(idle_list *sort_idle_list,status_list_node *list)//¿ÕÏÐÓÐÐò¶ÓÁÐ(°´ÈÝÁ¿´óÐ¡ÓÉ´óµ½Ð¡ÅÅÁÐ£¬·µ»Ø¿ÕÏÐ½Úµã¸öÊý)
{
   u8 i,j=1,k,t,flag=0;
   u8 count=0;
   for(i=1;i<=slave[0];i++){sort_idle_list[i].myid=0;sort_idle_list[i].size=0;}
   for(i=1;i<=slave[0];i++){
   	              if(list[slave[i]].work_status==0&&list[slave[i]].size!=0)
                      { sort_idle_list[j].myid=list[slave[i]].myid;
				        sort_idle_list[j].size=list[slave[i]].size;
					    j++;
						count++;
						if(flag==0)flag=1;
   	              	  }
                    }
   if(flag==1)
   	{
   for(i=2;i<=count;i++)
   	 {
       t=sort_idle_list[i].size;
	   k=sort_idle_list[i].myid;
	   for(j=i-1;j>=1&&t>sort_idle_list[j].size;j--)
	   	{sort_idle_list[j+1].myid=sort_idle_list[j].myid;
         sort_idle_list[j+1].size=sort_idle_list[j].size;
	    }
	   sort_idle_list[j+1].myid=k;
	   sort_idle_list[j+1].size=t;

      }
for(i=1;i<count;i++)
   {if(sort_idle_list[i].myid==mybox.myid)
           {   t=sort_idle_list[i].size;
	        k=sort_idle_list[i].myid;
             for(j=i;j<count;j++)
              	{sort_idle_list[j].size=sort_idle_list[j+1].size;
                       sort_idle_list[j].myid=sort_idle_list[j+1].myid;
			 }
			 sort_idle_list[count].size=t;
			 sort_idle_list[count].myid=k;
			 break;
           }
    }
   	}
sort_idle_list[0].size=count;
	return count;
}


//Î´Íê³É







/**************½«µÚÒ»×é¿ÕÏÐ¶ÓÁÐºÍµÚ¶þ×é¿ÕÏÐ¶ÓÁÐ×é³ÉÒ»×é½øÐÐÅÅÐòÂÖ ÐÝ***************************/


u8 turn_idlenode_list(turn_node *turn_idle_list,status_list_node *list_1,status_list_node *list_2)//¿ÕÏÐÓÐÐò¶ÓÁÐ(°´ÈÝÁ¿´óÐ¡ÓÉ´óµ½Ð¡ÅÅÁÐ£¬·µ»Ø¿ÕÏÐ½Úµã¸öÊý)
{
   u8 i,j=1,k,t,g,flag=0;
   u8 count=0;
   for(i=1;i<=(2*slave[0]);i++){turn_idle_list[i].myid=0;turn_idle_list[i].size=0;turn_idle_list[i].group=0;}
   for(i=1;i<=slave[0];i++){
   	              if(list_1[slave[i]].work_status==0&&list_1[slave[i]].size!=0)
                      { turn_idle_list[j].myid=list_1[slave[i]].myid;
				        turn_idle_list[j].size=list_1[slave[i]].size;
						turn_idle_list[j].group=1;
					          j++;
						count++;
						if(flag==0)flag=1;//Èç¹ûÃ»ÓÐ¿ÕÏÐ½Úµã
   	              	  }
                    }

for(i=1;i<=slave[0];i++)
		{
   	              if(list_2[slave[i]].work_status==0&&list_2[slave[i]].size!=0)
                      { turn_idle_list[j].myid=list_2[slave[i]].myid;
				        turn_idle_list[j].size=list_2[slave[i]].size;
						turn_idle_list[j].group=2;
					          j++;
						count++;
						if(flag==0)flag=1;//Èç¹ûÃ»ÓÐ¿ÕÏÐ½Úµã
   	              	  }
                    }
   
   if(flag==1)
   	{
   for(i=2;i<=count;i++)
   	 {
       t=turn_idle_list[i].size;
	   k=turn_idle_list[i].myid;
	   g=turn_idle_list[i].group;
	   for(j=i-1;j>=1&&t>turn_idle_list[j].size;j--)
	   	{turn_idle_list[j+1].myid=turn_idle_list[j].myid;
                turn_idle_list[j+1].size=turn_idle_list[j].size;
		  turn_idle_list[j+1].group=turn_idle_list[j].group;
	    }
	            turn_idle_list[j+1].myid=k;
	           turn_idle_list[j+1].size=t;
			turn_idle_list[j+1].group=g;

      }
for(i=1;i<count;i++)
   {if(turn_idle_list[i].myid==mybox.myid)
           {   t=turn_idle_list[i].size;
	        k=turn_idle_list[i].myid;
		g=turn_idle_list[i].group;
             for(j=i;j<count;j++)
              	{turn_idle_list[j].size=turn_idle_list[j+1].size;
                       turn_idle_list[j].myid=turn_idle_list[j+1].myid;
			turn_idle_list[j].group=turn_idle_list[j+1].group;
			 }
			 turn_idle_list[count].size=t;
			 turn_idle_list[count].myid=k;
			 turn_idle_list[count].group=g;
			 break;
           }
    }
   	}
   //turn_idle_list[0]=count;
    return count;
}








/***************************************************************************************************************************/







/*
void sort_timenode_list(time_list *sort_time_list,status_list_node *list)//Ê±¼äÓÐÐò¶ÓÁÐ(°´¹¤×÷Ê±¼äÓÉ´óµ½Ð¡ÅÅÁÐ)
{
u8 i,j=1,k,t,s;
for(i=1;i<33;i++){sort_time_list[i].myid=0;sort_time_list[i].work_time=0;sort_time_list[i].size=0;}
 for(i=1;i<33;i++){
   	              if(list[i].myid!=0)
                     {sort_time_list[j].myid=list[i].myid;
				      sort_time_list[j].work_time=list[i].work_time;
					  sort_time_list[j].size=list[i].size;
					  j++;
   	              	 }
                    }
   for(i=2;i<33;i++)
   	{
       t=sort_time_list[i].work_time;
	   k=sort_time_list[i].myid;
	   s=sort_time_list[i].size;
	   for(j=i-1;j>=1&&t>sort_time_list[j].work_time;j--)
	   	{sort_time_list[j+1].myid=sort_time_list[j].myid;
         sort_time_list[j+1].work_time=sort_time_list[j].work_time;
		 sort_time_list[j+1].size=sort_time_list[j].size;
	    }
	   sort_time_list[j+1].myid=k;
	   sort_time_list[j+1].work_time=t;
       sort_time_list[j+1].size=s;
   }
}
*/
u8 sort_busynode_list(busy_list *sort_busy_list,status_list_node *list)//Ã¦ÂµÓÐÐò¶ÓÁÐ(°´¹¤×÷Ê±¼ä´óÐ¡ÓÉ´óµ½Ð¡ÅÅÁÐ)
{
   u8 i,j=1,g,f,k,t,w,flag=0;
   u8 count=0,count_time=0;
   for(i=1;i<=slave[0];i++){sort_busy_list[i].myid=0;sort_busy_list[i].size=0;}
   for(i=1;i<=slave[0];i++){
   	              if(list[slave[i]].work_status==1&&list[slave[i]].size!=0)
                      { sort_busy_list[j].myid=list[slave[i]].myid;
				        sort_busy_list[j].size=list[slave[i]].size;
						sort_busy_list[j].work_time=list[slave[i]].work_time;
					    j++;
						if(flag==0)flag=1;
						count++;
   	              	  }
                    }
   if(flag==1)
   	{
   for(i=2;i<=count;i++)
   	 {
       t=sort_busy_list[i].size;
	   k=sort_busy_list[i].myid;
	   w=sort_busy_list[i].work_time;
	   for(j=i-1;j>=1&&t>sort_busy_list[j].size;j--)
	   	{sort_busy_list[j+1].myid=sort_busy_list[j].myid;
         sort_busy_list[j+1].size=sort_busy_list[j].size;
		 sort_busy_list[j+1].work_time=sort_busy_list[j].work_time;
	    }
	   sort_busy_list[j+1].myid=k;
	   sort_busy_list[j+1].size=t;
       sort_busy_list[j+1].work_time=w;
     }
   /**********************************************/
   for(i=1;i<=count;i=i+count_time)
      { count_time=0;
        for(j=i;j<=count;j++)
        	{ if(sort_busy_list[i].size==sort_busy_list[j].size){count_time++;}
                     else break;
		}
		
		for(g=i+1;g<=i+count_time-1;g++)
			{
                               t=sort_busy_list[g].size;
	                        k=sort_busy_list[g].myid;
	                        w=sort_busy_list[g].work_time;
                 	   for(f=g-1;f>=i&&w>sort_busy_list[f].work_time;f--)
	   	{
	   	 sort_busy_list[f+1].myid=sort_busy_list[f].myid;
               sort_busy_list[f+1].size=sort_busy_list[f].size;
		 sort_busy_list[f+1].work_time=sort_busy_list[f].work_time;
	       }
	      sort_busy_list[f+1].myid=k;
	      sort_busy_list[f+1].size=t;
             sort_busy_list[f+1].work_time=w;
		       }

      }
           
   
   	}
   //sort_busy_list[0]=count;
  return count;
}


u8 sort_busynode_list_asc(busy_list *sort_busy_list,status_list_node *list)//Ã¦ÂµÓÐÐò¶ÓÁÐ(°´¹¤×÷ÈÝÁ¿´óÐ¡ÓÉ´óµ½Ð¡ÅÅÁÐ)
{
   u8 i,j=1,g,f,k,t,w,flag=0;
   u8 count=0,count_time=0;
   for(i=1;i<=slave[0];i++){sort_busy_list[i].myid=0;sort_busy_list[i].size=0;}
   for(i=1;i<=slave[0];i++){
   	              if(list[slave[i]].work_status==1&&list[slave[i]].size!=0)
                      { sort_busy_list[j].myid=list[slave[i]].myid;
				        sort_busy_list[j].size=list[slave[i]].size;
						sort_busy_list[j].work_time=list[slave[i]].work_time;
					    j++;
						if(flag==0)flag=1;
						count++;
   	              	  }
                    }
   if(flag==1)
   	{
   for(i=2;i<=count;i++)
   	 {
       t=sort_busy_list[i].size;
	   k=sort_busy_list[i].myid;
	   w=sort_busy_list[i].work_time;
	   for(j=i-1;j>=1&&t<sort_busy_list[j].size;j--)
	   	{sort_busy_list[j+1].myid=sort_busy_list[j].myid;
         sort_busy_list[j+1].size=sort_busy_list[j].size;
		 sort_busy_list[j+1].work_time=sort_busy_list[j].work_time;
	    }
	   sort_busy_list[j+1].myid=k;
	   sort_busy_list[j+1].size=t;
       sort_busy_list[j+1].work_time=w;
     }
   /**********************************************/
   for(i=1;i<=count;i=i+count_time)
      { count_time=0;
        for(j=i;j<=count;j++)
        	{ if(sort_busy_list[i].size==sort_busy_list[j].size){count_time++;}
                     else break;
		}
		
		for(g=i+1;g<=i+count_time-1;g++)
			{
                               t=sort_busy_list[g].size;
	                        k=sort_busy_list[g].myid;
	                        w=sort_busy_list[g].work_time;
                 	   for(f=g-1;f>=i&&w>sort_busy_list[f].work_time;f--)
	   	{
	   	 sort_busy_list[f+1].myid=sort_busy_list[f].myid;
               sort_busy_list[f+1].size=sort_busy_list[f].size;
		 sort_busy_list[f+1].work_time=sort_busy_list[f].work_time;
	       }
	      sort_busy_list[f+1].myid=k;
	      sort_busy_list[f+1].size=t;
             sort_busy_list[f+1].work_time=w;
		       }

      }
           
   
   	}
     // sort_busy_list[0]=count;
  return count;
}







void delay_time(u32 time)
{ heartbeat(time);
}  //±¾ÏµÍ³µÄÑÓÊ±º¯Êý£¬time*10ms

u8 inquiry_slave_status(u8 id,status_list_node *comm_list ,u8 *slave_comm)   
  {  u8 *msg;
        u8 err;
	

   order_trans_rs485(mybox.myid,id,2,0,0,CONTROL);
  // delay_us(10000);
   msg=(u8 *)OSMboxPend(RS485_STUTAS_MBOX,OS_TICKS_PER_SEC/50,&err);
   if(err==OS_ERR_TIMEOUT){return 0;}//(u8 id, u8 size, u8 work_status, u8 work_time) 
	else 
{ init_Queue(id,msg[3],msg[4],msg[5],msg[6],slave_comm,comm_list);return 1;}

} //²éÑ¯´Ó»ú×´Ì¬²¢±£´æµ½´Ó»ú×´Ì¬±íÖÐ£¬²ÎÊýidÊÇÒª²éÑ¯µÄ´Ó»úºÅ

/*******************¹¦ÂÊÒòËØÏà¹Øº¯Êý*****************************/
 s8 flag_phase=1;
void gonglvyinshu()
{
		 float adcv,adci,X,Y,Mag_v,Mag_i;
	        u16 i=0,flag_v=5,flag_i=5;
	         int32_t lX=0,lY=0;



		id_num=AT24CXX_ReadOneByte(0x0010);

/*	 for(i=0;i<600;i++)
	  {
		 adc_ix=Get_Adc_Average(ADC_Channel_4,2);
		 if(adc_ix>adc_imax)
		 adc_imax=adc_ix;
	  }	
*/
//if(adc_imax>=1750)

        	{
        for(i=0;i<NPT;i++)
        	{
		adci=(Get_Adc_Average(ADC_Channel_4,4)-1520)/10;
		adcv=(Get_Adc_Average(ADC_Channel_1,4)-1520)/10;                //10
		adcv=(Get_Adc_Average(ADC_Channel_1,4)-1520)/10;
		adci=(Get_Adc_Average(ADC_Channel_4,4)-1520)/10;
		lBUFIN_V[i]=((short)adcv) << 16;
		lBUFIN_I[i]=((short)adci) << 16;		
		
        	}

}

			 allphase(lBUFIN_V,lBUFIN_I);

		
          cr4_fft_256_stm32(lBUFOUT_V, lBUFIN_V, NPT);
	   cr4_fft_256_stm32(lBUFOUT_I, lBUFIN_I, NPT);

			 
		 lX  = (lBUFOUT_V[flag_v] << 16) >> 16;
                 lY  = (lBUFOUT_V[flag_v] >> 16);
				angle[0]=atan2(lY,lX);
				   X    = FFT_NM* ((float)lX) /32768;
                   Y    = FFT_NM* ((float)lY) /32768;
	Mag_v= sqrt(X*X + Y*Y)/FFT_NM/1.414;
                   dianya_zhi= (float)((Mag_v* 65536)/10.9);
	
		lX  = (lBUFOUT_I[flag_i] << 16) >> 16;
                 lY  = (lBUFOUT_I[flag_i] >> 16);
				   {
                   X    = FFT_NM* ((float)lX) /32768;
                   Y    = FFT_NM* ((float)lY) /32768;
            //        Mag_i = sqrt(X*X + Y*Y)/FFT_NM/1.414;
                    Mag_i = sqrt(X*X + Y*Y)/FFT_NM/1.414/2.9;					
                 dianliuzhi= (u32)(Mag_i* 65536)*K_BT/10;//(u32)(Mag_i* 65536)*K_BT/100;ÓÃÓÚ²âÊÔ±ä±È
                   }
				angle[1]=atan2(lY,lX);
/************************************phase******************/
				angle[3]=((angle[0]-angle[1])*360)/PI2;
                         	{
			if((angle[3]>3.0&&angle[3]<178.0))flag_phase=1;
				
			if((angle[3]>-178.0&&angle[3]<-3.0))flag_phase=0;
			//	if((angle[3]>0.0&&angle[3]<180.0)||(angle[3]>-360.0&&angle[3]<-180.0))flag_phase=1;
			//	if((angle[3]>180.0&&angle[3]<360.0)||(angle[3]>-180.0&&angle[3]<-0.0))flag_phase=0;
				}
/************************************phase_end*******************/
				
				angle[2]=((angle[1]-angle[0])*360)/PI2-90;


if((angle[2]>0.0)&&(angle[2]<180)){if(flag_phase==0)L_C_flag=0;else L_C_flag=1;}
if((angle[2]<-0.0)&&(angle[2]>-180.0)){if(flag_phase==0)L_C_flag=1;else L_C_flag=0;}
gonglvshishu=(u8)abs(sin((angle[1]-angle[0]))*100);

if(dianliuzhi<1)gonglvshishu=99;

			 wugongkvar=(uint16_t)((1.732*dianliuzhi*dianya_zhi*abs(cos((angle[1]-angle[0]))*100))/100000);
			wugong_95= (uint16_t)((17.32*dianliuzhi*dianya_zhi*31)/100000);//¹¦ÂÊÒòËØÔÚ0.95Ê±µÄ£¬ÎÞ¹¦¹¦Â
			wugong_computer=(uint16_t)((17.32*dianliuzhi*dianya_zhi*abs(cos((angle[1]-angle[0]))*100))/100000);
                    //wugongkvar=wugong_computer;
			
		
		   delay_time(1);
	

//ÎÞ¹¦¹¦ÂÊ
}



void allphase(long *V,long *I)
{
int i=0;
for(i=0;i<=NPT/2-1;i++)
{
V[i]=(i+1)*V[i];
I[i]=(i+1)*I[i];
}
for(i=NPT/2;i<NPT-1;i++)
{
V[i]=(NPT-(i+1))*V[i];
I[i]=(NPT-(i+1))*I[i];

}

for(i=0;i<NPT/2-1;i++)
{
V[i+NPT/2]=V[i]+V[i+NPT/2];
I[i+NPT/2]=I[i]+I[i+NPT/2];

}

for(i=0;i<=NPT/2-1;i++)
{
V[i]=V[NPT/2-1+i];
I[i]=I[NPT/2-1+i];

}
}


void temperature()   //µçÈÝÆ÷ÎÂ¶È¼ì²â
{
 u16 adc_tmp1=0;
 //u16 adc_tmp2=0;
       adc_tmp1=Get_Adc2(ADC_Channel_5);
//	  adc_tmp2=Get_Adc_Average(ADC_Channel_6,10);
	  tempshuzhi=(u8)(258-((adc_tmp1*255)/4096));

}

void LIGHT(u8 status_1,u8 status_2,u8 background )
{
if(background==1)
{
if(status_1==0&&status_2==1)HT595_Send_Byte((GREEN_RED)|background_light_on);
if(status_1==1&&status_2==0)HT595_Send_Byte((RED_GREEN)|background_light_on);
if(status_1==0&&status_2==0)HT595_Send_Byte((GREEN_GREEN)|background_light_on);
if(status_1==1&&status_2==1)HT595_Send_Byte((RED_RED)|background_light_on);
if(status_1==2&&status_2==2)HT595_Send_Byte((YELLOW_YELLOW)|background_light_on);
}
if(background==0)
{
if(status_1==0&&status_2==1)HT595_Send_Byte((GREEN_RED));
if(status_1==1&&status_2==0)HT595_Send_Byte((RED_GREEN));
if(status_1==0&&status_2==0)HT595_Send_Byte((GREEN_GREEN));
if(status_1==1&&status_2==1)HT595_Send_Byte((RED_RED));
if(status_1==2&&status_2==2)HT595_Send_Byte((YELLOW_YELLOW));
}

}

void myled()
  {
gonglvyinshu();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª
temperature();
key_lcd();
delay_ms(50);//Ã»ÓÐÑÓÊ±£¬ÆÁ»áËÀ»ú
}

void Alarm(void)
{}
	 
void key_lcd()
{
key_idset();//°´¼üÓëÏÔÊ¾¹¦ÄÜ
Alarm();//ÊÇ·ñÐèÒª±¨¾¯
// LIGHT(mystatus.work_status[0],mystatus.work_status[1]);//Ë¢Ö¸Ê¾µÆ£¬Èç¹ûÏÔÊ¾Æ÷ÓÐÅÔÂ·µçÈÝÂË²¨ ¿ÉÒÔÉ¾³ý
}


void scanf_slave_machine()
{
u8 i,g;
u8 flag_comm=0;
u8 *msg;
  u8 err;
    static u8 comm_err[11];
static u8 first_init=1;
if(first_init==1)
{
init_listindex(slave);
first_init=0;
}

{
for(i=1;i<=10;i++)
	{  

for(g=0;g<=slave[9]-1;g++)
{
if(i==system_status_list[g].myid){flag_comm=1;break;}

else flag_comm=0;
}
if(flag_comm==0)
		{
		{
	inquiry_slave_status(i,system_status_list,slave);
		}
			}
if(flag_comm==1)

{
{order_trans_rs485(mybox.myid,i,5,0,0,CONTROL); 
  msg=(u8 *)OSMboxPend(RS485_STUTAS_MBOX,OS_TICKS_PER_SEC/10,&err);
     if(err==OS_ERR_TIMEOUT)
	 	{
	  	comm_err[i-1]++; 
if(comm_err[i-1]==3)
	  {
	  	comm_err[i-1]=0; 

del_comm_listnode(i,1,slave,system_status_list);
 del_comm_listnode(i,2,slave,system_status_list);
}


	 }

else  if(msg[2]==i)
	{
			comm_err[i-1]=0; 
	if(flag_comm==1)
		{
		 flash_comm_list(i,msg[3] ,1,slave,system_status_list);
		 flash_comm_list(i,msg[4] ,2,slave,system_status_list);
		}
		
       }

}
	


}
	flag_comm=0;
    }
}	  
}


void init_Queue(u8 id,u8 size_1,u8 size_2,u8 work_status_1,u8 work_status_2,u8 *slave_comm,status_list_node *comm_list)
{
u8 i=0;
{
  if(size_1==2)
  	{
  	for(i=slave_comm[9];i>slave_comm[8];i--)//20µÄ¶ÓÁÐÒÆ¶¯
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;

	  	for(i=slave_comm[7];i>slave_comm[6];i--)//10µÄ¶ÓÁÐÒÆ¶¯
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
		slave_comm[7]++;
			
	  	for(i=slave_comm[5];i>slave_comm[4];i--)//5µÄ¶ÓÁÐÒÆ¶¯
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}	
slave_comm[5]++;
		
	   comm_list[slave_comm[3]].myid=id;
   	   comm_list[slave_comm[3]].size=size_1;
   	   comm_list[slave_comm[3]].work_status=work_status_1;
	   comm_list[slave_comm[3]].group=1;

		 slave_comm[3]++;
		slave_comm[4]=slave_comm[3]+1;
		slave_comm[6]=slave_comm[5]+1;
		slave_comm[8]=slave_comm[7]+1;

		 slave_comm[0]++;

  }
  
    if(size_1==5)
  	{
  		for(i=slave_comm[9];i>slave_comm[8];i--)//20µÄ¶ÓÁÐÒÆ¶¯
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;

	  	for(i=slave_comm[7];i>slave_comm[6];i--)//10µÄ¶ÓÁÐÒÆ¶¯
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
		slave_comm[7]++;
		
	   comm_list[slave_comm[5]].myid=id;
   	   comm_list[slave_comm[5]].size=size_1;
   	   comm_list[slave_comm[5]].work_status=work_status_1;
	   comm_list[slave_comm[5]].group=1;

	    slave_comm[5]++;
		slave_comm[6]=slave_comm[5]+1;
		slave_comm[8]=slave_comm[7]+1;
		 slave_comm[0]++;

  	} 
	if(size_1==10)
  	{
  		for(i=slave_comm[9];i>slave_comm[8];i--)//20µÄ¶ÓÁÐÒÆ¶¯
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;
	
	   comm_list[slave_comm[7]].myid=id;
   	   comm_list[slave_comm[7]].size=size_1;
   	   comm_list[slave_comm[7]].work_status=work_status_1;
	   comm_list[slave_comm[7]].group=1;

	       slave_comm[7]++;
		slave_comm[8]=slave_comm[7]+1;
		 slave_comm[0]++;

  	} 
	if(size_1==20)
  	{
	   comm_list[slave_comm[9]].myid=id;
   	   comm_list[slave_comm[9]].size=size_1;
   	   comm_list[slave_comm[9]].work_status=work_status_1;
	   comm_list[slave_comm[9]].group=1;
	   	    slave_comm[9]++;
		 slave_comm[0]++;

  	}
}



{
  if(size_2==2)
  	{
  	for(i=slave_comm[9];i>slave_comm[8];i--)//20µÄ¶ÓÁÐÒÆ¶¯
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;

	  	for(i=slave_comm[7];i>slave_comm[6];i--)//10µÄ¶ÓÁÐÒÆ¶¯
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
		slave_comm[7]++;
			
	  	for(i=slave_comm[5];i>slave_comm[4];i--)//5µÄ¶ÓÁÐÒÆ¶¯
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}	
slave_comm[5]++;
		
	   comm_list[slave_comm[3]].myid=id;
   	   comm_list[slave_comm[3]].size=size_2;
   	   comm_list[slave_comm[3]].work_status=work_status_2;
	   comm_list[slave_comm[3]].group=2;

		 slave_comm[3]++;
		slave_comm[4]=slave_comm[3]+1;
		slave_comm[6]=slave_comm[5]+1;
		slave_comm[8]=slave_comm[7]+1;

		 slave_comm[0]++;

  }
  
    if(size_2==5)
  	{
  		for(i=slave_comm[9];i>slave_comm[8];i--)//20µÄ¶ÓÁÐÒÆ¶¯
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;

	  	for(i=slave_comm[7];i>slave_comm[6];i--)//10µÄ¶ÓÁÐÒÆ¶¯
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
		slave_comm[7]++;
		
	   comm_list[slave_comm[5]].myid=id;
   	   comm_list[slave_comm[5]].size=size_2;
   	   comm_list[slave_comm[5]].work_status=work_status_2;
	   comm_list[slave_comm[5]].group=2;

	    slave_comm[5]++;
		slave_comm[6]=slave_comm[5]+1;
		slave_comm[8]=slave_comm[7]+1;
		 slave_comm[0]++;

  	} 
	if(size_2==10)
  	{
  		for(i=slave_comm[9];i>slave_comm[8];i--)//20µÄ¶ÓÁÐÒÆ¶¯
  		{
	   comm_list[i].myid=comm_list[i-1].myid;
   	   comm_list[i].size=comm_list[i-1].size;
   	   comm_list[i].work_status=comm_list[i-1].work_status;
	   comm_list[i].group=comm_list[i-1].group;
  		}
	slave_comm[9]++;
	
	   comm_list[slave_comm[7]].myid=id;
   	   comm_list[slave_comm[7]].size=size_2;
   	   comm_list[slave_comm[7]].work_status=work_status_2;
	   comm_list[slave_comm[7]].group=2;

	       slave_comm[7]++;
		slave_comm[8]=slave_comm[7]+1;
		 slave_comm[0]++;

  	} 
	if(size_2==20)
  	{
	   comm_list[slave_comm[9]].myid=id;
   	   comm_list[slave_comm[9]].size=size_2;
   	   comm_list[slave_comm[9]].work_status=work_status_2;
	   comm_list[slave_comm[9]].group=2;
	   	    slave_comm[9]++;
		 slave_comm[0]++;

  	}
}

}
void init_listindex(u8 *slave_comm)
{
slave_comm[2]=0;//¶ÓÁÐ2v ±êÊ¾³õÊ¼»¯
slave_comm[3]=0;

slave_comm[4]=1;//¶ÓÁÐ5v ±êÊ¾³õÊ¼»¯
slave_comm[5]=1;

slave_comm[6]=2;//¶ÓÁÐ10v ±êÊ¾³õÊ¼»¯
slave_comm[7]=2;

slave_comm[8]=3;//¶ÓÁÐ20v ±êÊ¾³õÊ¼»¯
slave_comm[9]=3;

}
void del_comm_listnode(u8 id,u8 group,u8 *slave_comm,status_list_node *comm_list)
{
u8 i=0;
u8 j=0;
  	
{
  	  		for(i=slave_comm[2];i<=slave_comm[9]-1;i++)//2
	if(id==comm_list[i].myid&&group==comm_list[i].group)//Ëø¶¨½Úµã½øÐÐ¸üÐÂ
		{ 

	
		for(j=i;j<slave_comm[9]-1;j++)
		      {
		         comm_list[j].myid=comm_list[j+1].myid;
   	   comm_list[j].size=comm_list[j+1].size;
   	   comm_list[j].work_status=comm_list[j+1].work_status;
	   comm_list[j].group=comm_list[j+1].group;
		      }
	       if(i<slave_comm[3])
		   	{
                         {
			slave_comm[3]--;
	             comm_list[slave_comm[3]].myid=0;
   	               comm_list[slave_comm[3]].size=0;
   	                comm_list[slave_comm[3]].work_status=0;
	              comm_list[slave_comm[3]].group=0;

		          }	
		   
                      {
			slave_comm[4]--;					  	
			slave_comm[5]--;
	             comm_list[slave_comm[5]].myid=0;
   	               comm_list[slave_comm[5]].size=0;
   	                comm_list[slave_comm[5]].work_status=0;
	              comm_list[slave_comm[5]].group=0;

		          }	
				  
                      {
			slave_comm[6]--;					  	
			slave_comm[7]--;
	             comm_list[slave_comm[7]].myid=0;
   	               comm_list[slave_comm[7]].size=0;
   	                comm_list[slave_comm[7]].work_status=0;
	              comm_list[slave_comm[7]].group=0;

		          }			
				  
	             {
			slave_comm[8]--;				 	
			slave_comm[9]--;
	             comm_list[slave_comm[9]].myid=0;
   	               comm_list[slave_comm[9]].size=0;
   	                comm_list[slave_comm[9]].work_status=0;
	              comm_list[slave_comm[9]].group=0;

		          }
		   	}
		else if(i<slave_comm[5]&&i>slave_comm[3])
			{
                         	   
                      {
			slave_comm[5]--;
	             comm_list[slave_comm[5]].myid=0;
   	               comm_list[slave_comm[5]].size=0;
   	                comm_list[slave_comm[5]].work_status=0;
	              comm_list[slave_comm[5]].group=0;

		          }	
				  
                      {
			slave_comm[6]--;					  	
			slave_comm[7]--;
	             comm_list[slave_comm[7]].myid=0;
   	               comm_list[slave_comm[7]].size=0;
   	                comm_list[slave_comm[7]].work_status=0;
	              comm_list[slave_comm[7]].group=0;

		          }			
				  
	             {
			slave_comm[8]--;				 	
			slave_comm[9]--;
	             comm_list[slave_comm[9]].myid=0;
   	               comm_list[slave_comm[9]].size=0;
   	                comm_list[slave_comm[9]].work_status=0;
	              comm_list[slave_comm[9]].group=0;

		          }
		   	}
		else if(i<slave_comm[7]&&i>slave_comm[5])
			{
                         		  
                      {
			slave_comm[7]--;
	             comm_list[slave_comm[7]].myid=0;
   	               comm_list[slave_comm[7]].size=0;
   	                comm_list[slave_comm[7]].work_status=0;
	              comm_list[slave_comm[7]].group=0;

		          }			
				  
	             {
			slave_comm[8]--;				 	
			slave_comm[9]--;
	             comm_list[slave_comm[9]].myid=0;
   	               comm_list[slave_comm[9]].size=0;
   	                comm_list[slave_comm[9]].work_status=0;
	              comm_list[slave_comm[9]].group=0;

		          }
		   	}
		else if(i<slave_comm[9]&&i>slave_comm[7])
			{
							  
	             {
			slave_comm[9]--;
	             comm_list[slave_comm[9]].myid=0;
   	               comm_list[slave_comm[9]].size=0;
   	                comm_list[slave_comm[9]].work_status=0;
	              comm_list[slave_comm[9]].group=0;

		          }
		   	}

break;
	}



		 
  	 
  	
}

}

void flash_comm_list(u8 id,u8 work_status ,u8 group,u8 *slave_comm,status_list_node *comm_list)
{
u8 i;
u8 flash=0;
if(flash==0)
{
	for(i=slave_comm[2];i<slave_comm[3];i++)// 2 
	{
	if(id==comm_list[i].myid&&group==comm_list[i].group)//Ëø¶¨½Úµã½øÐÐ¸üÐÂ
		{
	   comm_list[i].myid=id;
   	   comm_list[i].work_status=work_status;
	   comm_list[i].group=group;
	   flash=1;
	   break;
		}
	}
}

if(flash==0)
{
	for(i=slave_comm[4];i<slave_comm[5];i++)// 2 
	{
	if(id==comm_list[i].myid&&group==comm_list[i].group)//Ëø¶¨½Úµã½øÐÐ¸üÐÂ
		{
	   comm_list[i].myid=id;
   	   comm_list[i].work_status=work_status;
	   comm_list[i].group=group;
	   flash=1;
	   break;
		}
	}
}

if(flash==0)
{
	for(i=slave_comm[6];i<slave_comm[7];i++)// 2 
	{
	if(id==comm_list[i].myid&&group==comm_list[i].group)//Ëø¶¨½Úµã½øÐÐ¸üÐÂ
		{
	   comm_list[i].myid=id;
   	   comm_list[i].work_status=work_status;
	   comm_list[i].group=group;
	   flash=1;
	   break;
		}
	}
}

if(flash==0)
{
	for(i=slave_comm[8];i<slave_comm[9];i++)// 2 
	{
	if(id==comm_list[i].myid&&group==comm_list[i].group)//Ëø¶¨½Úµã½øÐÐ¸üÐÂ
		{
	   comm_list[i].myid=id;
   	   comm_list[i].work_status=work_status;
	   comm_list[i].group=group;
	   flash=1;
	   break;
		}
	}
}
}
/*********************************/

void change_Queue(u8 *slave_comm,status_list_node *comm_list,u8 size)
{

u8 i=0;
u8 m,s,w,g;
{
  if(size==2)
  	{
  	m=comm_list[slave_comm[2]].myid;
	s=comm_list[slave_comm[2]].size;
	w=comm_list[slave_comm[2]].work_status;
	g=comm_list[slave_comm[2]].group;

	for(i=slave_comm[2];i<slave_comm[3]-1;i++)// 2 µÄ¶ÓÁÐÒÆ¶¯
  		{
	   comm_list[i].myid=comm_list[i+1].myid;
   	   comm_list[i].size=comm_list[i+1].size;
   	   comm_list[i].work_status=comm_list[i+1].work_status;
	   comm_list[i].group=comm_list[i+1].group;
  		}

		
	   comm_list[slave_comm[3]-1].myid=m;
   	   comm_list[slave_comm[3]-1].size=s;
   	   comm_list[slave_comm[3]-1].work_status=w;
	   comm_list[slave_comm[3]-1].group=g;

  	}
  
    if(size==5)
    	{
  	m=comm_list[slave_comm[4]].myid;
	s=comm_list[slave_comm[4]].size;
	w=comm_list[slave_comm[4]].work_status;
	g=comm_list[slave_comm[4]].group;

	for(i=slave_comm[4];i<slave_comm[5]-1;i++)//20µÄ¶ÓÁÐÒÆ¶¯
  		{
	   comm_list[i].myid=comm_list[i+1].myid;
   	   comm_list[i].size=comm_list[i+1].size;
   	   comm_list[i].work_status=comm_list[i+1].work_status;
	   comm_list[i].group=comm_list[i+1].group;
  		}

		
	   comm_list[slave_comm[5]-1].myid=m;
   	   comm_list[slave_comm[5]-1].size=s;
   	   comm_list[slave_comm[5]-1].work_status=w;
	   comm_list[slave_comm[5]-1].group=g;

  	}
	if(size==10)
  	{
  	m=comm_list[slave_comm[6]].myid;
	s=comm_list[slave_comm[6]].size;
	w=comm_list[slave_comm[6]].work_status;
	g=comm_list[slave_comm[6]].group;

	for(i=slave_comm[6];i<slave_comm[7]-1;i++)//20µÄ¶ÓÁÐÒÆ¶¯
  		{
	   comm_list[i].myid=comm_list[i+1].myid;
   	   comm_list[i].size=comm_list[i+1].size;
   	   comm_list[i].work_status=comm_list[i+1].work_status;
	   comm_list[i].group=comm_list[i+1].group;
  		}

		
	   comm_list[slave_comm[7]-1].myid=m;
   	   comm_list[slave_comm[7]-1].size=s;
   	   comm_list[slave_comm[7]-1].work_status=w;
	   comm_list[slave_comm[7]-1].group=g;

  	} 
	if(size==20)
  	{
  	m=comm_list[slave_comm[8]].myid;
	s=comm_list[slave_comm[8]].size;
	w=comm_list[slave_comm[8]].work_status;
	g=comm_list[slave_comm[8]].group;

	for(i=slave_comm[8];i<slave_comm[9]-1;i++)//20µÄ¶ÓÁÐÒÆ¶¯
  		{
	   comm_list[i].myid=comm_list[i+1].myid;
   	   comm_list[i].size=comm_list[i+1].size;
   	   comm_list[i].work_status=comm_list[i+1].work_status;
	   comm_list[i].group=comm_list[i+1].group;
  		}

		
	   comm_list[slave_comm[9]-1].myid=m;
   	   comm_list[slave_comm[9]-1].size=s;
   	   comm_list[slave_comm[9]-1].work_status=w;
	   comm_list[slave_comm[9]-1].group=g;

  	}
}
}

u8 computer_gonglu(status_list_node *comm_list,u8 *slave_comm)
{
u16 i;
s32 gl[2];
static u16 var=0;
static u8 warning_flag=0;
u16 min;
u16 TR[]={4,5,6,8,10,12,16,20,24,30,40,50,60,80,100,120};

{
if(RT_FLAG==0)	
{
        u8 err;
		gonglvyinshu();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª
if(L_C_flag==1)gl[0]=wugongkvar;
else gl[0]=-wugongkvar;

{
      	{
for(i=slave_comm[8];i<=slave_comm[9]-1;i++)
if(comm_list[i].work_status==0)

{

order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,1,CONTROL);
// OSMboxPend(RS485_RT,OS_TICKS_PER_SEC/10,&err);
    // if(err==OS_ERR_TIMEOUT);
//else 
		{
RT_FLAG=1;
var=var+(200*dianya_zhi*dianya_zhi)/450/450;
		}
}




/**********************************/
      	}


{
for(i=slave_comm[6];i<=slave_comm[7]-1;i++)
if(comm_list[i].work_status==0)
	{

order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,1,CONTROL);
//OSMboxPend(RS485_RT,OS_TICKS_PER_SEC/10,&err);
  //if(err==OS_ERR_TIMEOUT);
//else 
		{
RT_FLAG=1;
var=var+(100*dianya_zhi*dianya_zhi)/450/450;
		}
}

/*********************************/
	  }


{
for(i=slave_comm[4];i<=slave_comm[5]-1;i++)
if(comm_list[i].work_status==0)
	{

order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,1,CONTROL);
  OSMboxPend(RS485_RT,OS_TICKS_PER_SEC/10,&err);
   //  if(err==OS_ERR_TIMEOUT);
//	else 
		{
RT_FLAG=1;
var=var+(50*dianya_zhi*dianya_zhi)/450/450;
		}
}



/*********************************/

      	}




      
}
/**************************************Í¶Ö÷»ú**/
{
if(mystatus.work_status[0]==0)
{
GPIO_ResetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],1,mystatus.work_status[1],mystatus.work_time[0],mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
	  RT_FLAG=1;
var=var+(10*mystatus.work_status[0]*dianya_zhi*dianya_zhi)/450/450;

 }

if(mystatus.work_status[1]==0)
{GPIO_ResetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],1,mystatus.work_time[0],mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
	  	  RT_FLAG=1;
var=var+(10*mystatus.work_status[1]*dianya_zhi*dianya_zhi)/450/450;

 }
}
/**************************************Í¶Ö÷»úend**/

}

if(RT_FLAG==1)	
{
delay_us(2500000);//36->512
gonglvyinshu();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª
if(L_C_flag==1)gl[1]=wugongkvar;
else gl[1]=-wugongkvar;
min=abs(abs(gl[0]-gl[1])*TR[0]-var);

for(i=1;i<16;i++)
{
if(abs(abs(gl[0]-gl[1])*TR[i]-var)<=min){min=abs(abs(gl[0]-gl[1])*TR[i]-var);K_BT=TR[i];}

}
RT_FLAG=2;
/**************************************ÇÐÖ÷»ú**/
{
 	{
 	GPIO_SetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,mystatus.work_status[1],0,mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);

 }
delay_us(100000);

{GPIO_SetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],0,mystatus.work_time[0],0);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);

 }
delay_us(100000);

}
/**************************************ÇÐÖ÷»ú**/

order_trans_rs485(mybox.myid,0,1,1,0,CONTROL);
delay_us(1000000);
order_trans_rs485(mybox.myid,0,1,2,0,CONTROL);
delay_us(1000000);
return 0;


}
//tempshuzhi=K_BT;
K_BT=12;//Ð´ËÀ±ä±È´¦£¬ÓÃÓÚ·Ç×Ô¶¯ÅÐ¶Ï±ä±È£¬Èç¹û²ÉÓÃ×Ô¶¯»ñÈ¡±ä±ÈÐèÒª×¢µô¸Ã¾ä»°
if(RT_FLAG==2)
{
gonglvyinshu();//¼ÆËã¹¦ÂÊ£¬µçÑ¹µçÁ÷ÓëÏÔÊ¾°´¼ü·Ö¿ª

/**************************************¹ýÑ¹±£»¤**/
{
if((dianya_zhi>420||dianya_zhi<330))
{
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],2,mystatus.work_status[1],0,mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],2,mystatus.work_time[0],0);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);

}

if((dianya_zhi>420||dianya_zhi<330)&&warning_flag==0)
{
{
 	{
 	GPIO_SetBits(GPIOA,GPIO_Pin_0);

 }
delay_ms(2000);

{GPIO_SetBits(GPIOA,GPIO_Pin_8);

 }
delay_ms(2000);

}
order_trans_rs485(mybox.myid,0,1,1,0,CONTROL);
delay_ms(5000);
order_trans_rs485(mybox.myid,0,1,2,0,CONTROL);
delay_ms(5000);
warning_flag=1;
}
if(warning_flag==1&&dianya_zhi<=417&&dianya_zhi>=333)
	{warning_flag=0;
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,mystatus.work_status[1],0,mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],0,mystatus.work_time[0],0);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);

}
}
/**************************************¹ýÑ¹±£»¤END**/

if(dianya_zhi<=420&&dianya_zhi>=330&&warning_flag==0)
{
if(gonglvshishu<90&&L_C_flag==1)
 {
      {
      if(wugongkvar>=20)
      	{
for(i=slave_comm[8];i<=slave_comm[9]-1;i++)
if(comm_list[i].work_status==0)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,1,CONTROL);
		{
change_Queue(slave_comm,comm_list,20);
delay_ms(TIME_TQ);
return 0 ;

		}
}

      	}


	  if(wugongkvar>=10)
{
for(i=slave_comm[6];i<=slave_comm[7]-1;i++)
if(comm_list[i].work_status==0)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,1,CONTROL);
		{
change_Queue(slave_comm,comm_list,10);
delay_ms(TIME_TQ);
return 0 ;

		}
}

	  }

	  if(wugongkvar>=5)

{
for(i=slave_comm[4];i<=slave_comm[5]-1;i++)
if(comm_list[i].work_status==0)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,1,CONTROL);
		{
change_Queue(slave_comm,comm_list,5);
delay_ms(TIME_TQ);
return 0 ;

		}
}

      	}

	  if(wugongkvar>=2)

{
for(i=slave_comm[2];i<=slave_comm[3]-1;i++)
if(comm_list[i].work_status==0)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,1,CONTROL);
		{
change_Queue(slave_comm,comm_list,2);
delay_ms(TIME_TQ);
return 0 ;

		}
}



      	}
      
}
/**************************************Í¶Ö÷»ú**/
if(KEY1==1)//±ØÐëÊÇÔÚ×Ô¶¯Ìõ¼þÏÂ
{
	  if(wugongkvar>=mystatus.size[0])
{
if(mystatus.work_status[0]==0&&rework_time[0]==0)	
{
GPIO_ResetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],1,mystatus.work_status[1],mystatus.work_time[0],mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
delay_ms(TIME_TQ);
	  return 0 ;
 }
}

	  if(wugongkvar>=mystatus.size[1])
{
if(mystatus.work_status[1]==0&&rework_time[1]==0)
{GPIO_ResetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],1,mystatus.work_time[0],mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
delay_ms(TIME_TQ);
	  return 0 ;

 }
}
}
/**************************************Í¶Ö÷»úend**/

 }

if(gonglvshishu>=95&&L_C_flag==1)
   
{

/**************************************ÇÐÖ÷»ú**/
if(KEY1==1)//±ØÐëÊÇÔÚ×Ô¶¯Ìõ¼þÏÂ
{
if(mystatus.work_status[0]==1)
 	{
 	GPIO_SetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,mystatus.work_status[1],0,mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
delay_ms(TIME_TQ);
	  	  return 0 ;

 }
if(mystatus.work_status[1]==1)

{GPIO_SetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],0,mystatus.work_time[0],0);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
delay_ms(TIME_TQ);
	  return 0 ;

 }
}
/**************************************ÇÐÖ÷»ú**/

      {


{
for(i=slave_comm[2];i<=slave_comm[3]-1;i++)
if(comm_list[i].work_status==1)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,0,CONTROL);
		{
delay_ms(TIME_TQ);
return 0 ;

		}
}

}

{
for(i=slave_comm[4];i<=slave_comm[5]-1;i++)
if(comm_list[i].work_status==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,0,CONTROL);
		{
delay_ms(TIME_TQ);
return 0 ;

		}
}

}




{
for(i=slave_comm[6];i<=slave_comm[7]-1;i++)
if(comm_list[i].work_status==1)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,0,CONTROL);
		{
delay_ms(TIME_TQ);
return 0 ;

		}
}

}

{
for(i=slave_comm[8];i<=slave_comm[9]-1;i++)
if(comm_list[i].work_status==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,0,CONTROL);
		{
delay_ms(TIME_TQ);
return 0 ;

		}
}

}


       }
 }


if(L_C_flag==0)
{
/**************************************ÇÐÖ÷»ú**/
if(KEY1==1)//±ØÐëÊÇÔÚ×Ô¶¯Ìõ¼þÏÂ
{
if(mystatus.work_status[0]==1)
 	{
 	GPIO_SetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,mystatus.work_status[1],0,mystatus.work_time[1]);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
delay_ms(TIME_TQ);
	  	  return 0 ;

 }
if(mystatus.work_status[1]==1)

{GPIO_SetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],0,mystatus.work_time[0],0);
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
delay_ms(TIME_TQ);
	  return 0 ;

 }
}
/**************************************ÇÐÖ÷»ú**/

      {
{
for(i=slave_comm[8];i<=slave_comm[9]-1;i++)
if(comm_list[i].work_status==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,0,CONTROL);
		{
delay_ms(TIME_TQ);
return 0 ;

		}
}

}



{
for(i=slave_comm[6];i<=slave_comm[7]-1;i++)
if(comm_list[i].work_status==1)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,0,CONTROL);
		{
delay_ms(TIME_TQ);
return 0 ;

		}
}

}

{
for(i=slave_comm[4];i<=slave_comm[5]-1;i++)
if(comm_list[i].work_status==1)

{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,0,CONTROL);
		{
delay_ms(TIME_TQ);
return 0 ;

		}
}

}

{
for(i=slave_comm[2];i<=slave_comm[3]-1;i++)
if(comm_list[i].work_status==1)
{
order_trans_rs485(mybox.myid,comm_list[i].myid,1,comm_list[i].group,0,CONTROL);
		{
delay_ms(TIME_TQ);
return 0 ;

		}
}

}
       }

}

}
}
}
return 0;

}


