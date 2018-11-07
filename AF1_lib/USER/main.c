
#include "rs485.h"
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"	 
#include "24cxx.h"
#include "includes.h" 	 
#include "adc.h"
#include "timer.h"
#include "485ANN.h"
#include "ht1621.h"
#include "key.h"
//32
#include "lcd.h"//������
#include "stm32f10x_iwdg.h"
#include "stm32f10x_rcc.h"

#include "wdg.h"
/////////////////////////UCOSII��������///////////////////////////////////

#define SETID_TASK_PRIO       			1 
//���������ջ��С
#define SETID_STK_SIZE  		    		64
//�����ջ
OS_STK SETID_TASK_STK[SETID_STK_SIZE];
//������
void SETID_task(void *pdata);
//////////////////////////////////////////////////////////////////////////////

//��������
//�����������ȼ�

//���������ջ��С
#define MASTER_STK_SIZE  		 		64
//�����ջ	
OS_STK MASTER_TASK_STK[MASTER_STK_SIZE];
//������
 void master_task(void *pdata);

////////////////////////////////////////////////////////////////////////////
//��������
//�����������ȼ�
#define Receive_TASK_PRIO       			2 
//���������ջ��С
#define Receive_STK_SIZE  		    		64
//�����ջ
OS_STK Receive_TASK_STK[Receive_STK_SIZE];
//������
void  Receive_task(void *pdata);
////////////////////////////////////////////////////////////////////////

#define SCAN_TASK_PRIO       			4 
//���������ջ��С
#define SCAN_STK_SIZE  		    		64
//�����ջ
OS_STK SCAN_TASK_STK[SCAN_STK_SIZE];
//������
void scanf_task(void *pdata);


////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////
#define MYLED_TASK_PRIO       			5
//���������ջ��С
#define MYLED_STK_SIZE  		    		128
//�����ջ
OS_STK MYLED_TASK_STK[MYLED_STK_SIZE];
//������
void myled_task(void *pdata);

/////////////////////////////////////////////////////////////////////////
//START ����
//�����������ȼ�
#define START_TASK_PRIO      			12 //��ʼ��������ȼ�����Ϊ���
//���������ջ��С
#define START_STK_SIZE  				64
//�����ջ	
OS_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *pdata);	
 			   
/////////////////////////////////////////////////////////////////////////
  	   
extern box mybox;
  
extern  vu16  dog_clock;

extern OS_EVENT * RS485_MBOX,*RS485_STUTAS_MBOX,* RS485_RT;			//	rs485�����ź���

extern OS_EVENT *Heartbeat;			 //�����ź���
extern OS_EVENT *master_led_task;


extern OS_EVENT *scan_slave;

extern vu8 cont;//���ڸ��������ŵļǴ�����
extern  u8 token[33];//����������

extern status_box mystatus;

extern status_list_node system_status_list[33];

extern idle_list sort_idle_list_1[33];
extern idle_list sort_idle_list_2[33];
extern busy_list sort_busy_list_1[33];
extern busy_list sort_busy_list_2[33];

extern vu16 dianya_zhi;
extern vu8 hguestnum,gonglvshishu;
extern u32 idle_time,scan_time,dianliuzhi;
extern vu16 wugongkvar;
extern s8 L_C_flag;
extern vu8 tempshuzhi;
extern u8 slave[33];

extern u16 m1_opentime,m2_opentime,m1_closetime,m2_closetime;
extern u8 true_worktime1_flag,true_worktime2_flag;
extern u8 RT_FLAG;
extern vu8 rework_time[2];

//���ջ�����

//////////////////////////////////////////


u8 led_lock=0;
u8 init=20;
vu8 auto_on=1;
vu8 temperature_warn=0;
vu32 init_time=80;
int slave_control(u8,u8);
void warn(void);

void EXTI_Configuration(void);//��ʼ������

//#define ID  1
#define temperature_gate 55
#define SIZE_1 20
#define SIZE_2 20
#define WORK_STATUS_1	 0//0Ϊû�й���  1Ϊ����  2Ϊ��������ʼ��Ϊ0
#define WORK_STATUS_2    0 
#define WORK_TIME_1 0
#define WORK_TIME_2	0
/////////////////////////////////////////////
extern u8 ligt_time;
extern vu8 BT_num;
extern vu8 warn_volt_onlimt;
extern vu8 id_num;
int main(void)
 {	 
  
	 
	u8 i;
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_Configuration(); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
// 	LED_Init();			     //LED�˿ڳ�ʼ��
/*************************/
		delay_us(500000);
	HT1621_Init();
	KEY_Init();          //��ʼ���밴�����ӵ�Ӳ���ӿ�  
	AT24CXX_Init();			//IIC��ʼ��
	Adc_Init();
/************************************/
///	uart_init(9600);LCD_Init();	                                                              //������ʾ
RS485_Init(9600);	//��ʼ��RS485
	TIM4_Int_Init(9999*2,7199);//10Khz�ļ���Ƶ�ʣ�����10K��Ϊ1000ms 
	 initmybox();
	 init_mystatus(SIZE_1,SIZE_2,WORK_STATUS_1,WORK_STATUS_2,WORK_TIME_1,WORK_TIME_2);
EXTI_Configuration();//��ʼ������
//IWDG_Init(6,625); 
	 TIM3_Int_Init(9999,7199);//���ÿ��Ź��ж�
for(i=0;i<30;i++)slave[i]=0;

	OSInit();  	 			//��ʼ��UCOSII
			  
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//������ʼ����
	OSStart();	 

}							    
//��ʼ����
void start_task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;  	    
	pdata = pdata; 	
	 Heartbeat=OSSemCreate(0);
	 RS485_MBOX=OSMboxCreate((void*)0);
	 RS485_STUTAS_MBOX=OSMboxCreate((void*)0);
	 scan_slave=OSSemCreate(0);
	 RS485_RT=OSMboxCreate((void*)0);
	OSStatInit();					//��ʼ��ͳ������.�������ʱ1��������	
 	OS_ENTER_CRITICAL();			//�����ٽ���(�޷����жϴ��)    			   
	OSTaskCreate(master_task,(void *)0,(OS_STK*)&MASTER_TASK_STK[MASTER_STK_SIZE-1],MASTER_TASK_PRIO);	 				   
	OSTaskCreate(Receive_task,(void *)0,(OS_STK*)&Receive_TASK_STK[Receive_STK_SIZE-1],Receive_TASK_PRIO);
	OSTaskCreate(myled_task,(void *)0,(OS_STK*)&MYLED_TASK_STK[MYLED_STK_SIZE-1],MYLED_TASK_PRIO);
	OSTaskCreate(SETID_task,(void *)0,(OS_STK*)&SETID_TASK_STK[SETID_STK_SIZE-1],SETID_TASK_PRIO);
      OSTaskCreate(scanf_task,(void *)0,(OS_STK*)&SCAN_TASK_STK[SCAN_STK_SIZE-1],SCAN_TASK_PRIO);
 	OSTaskSuspend(START_TASK_PRIO);	//������ʼ����.
	OS_EXIT_CRITICAL();				//�˳��ٽ���(���Ա��жϴ��)
}
//LED����

 /**************�ӻ�����**********************/
void Receive_task(void *pdate)//�ӻ�����
{   u8 err;
	 u8 *msg;
	 int flag1;
	 
    while(1)
    	{
		 if(mybox.master==1)
		 	{
			OSTaskSuspend(Receive_TASK_PRIO);//����ӻ�����

		        }
	 msg=(u8 *)OSMboxPend(RS485_MBOX,0,&err);//���յ�������
	 flag1=rs485_trans_order(msg);
	 dog_clock=20;
mybox.myid=id_num;
mystatus.myid=mybox.myid;
	 if((flag1==1))/***�Ǳ�����Ϣ***/
	 	{		//LED1=!LED1;	  
		       dog_clock=20;

     				{
	 	      slave_control(mybox.relay,mybox.message);
		       	} 
	 	}

	}
	}
 /**************��������**********************/
  void master_task(void *pdata)	  //��������
  {	  OS_CPU_SR cpu_sr=0;
	  // u8 *msg,err;
   while(1)
   	{
  	if(mybox.master==1)
     {	
     hguestnum=111;
mybox.myid=id_num;
mystatus.myid=mybox.myid;
	OSSemPost(scan_slave);
	
  if(init!=0) {init--;order_trans_rs485(mybox.myid,0,1,1,0,CONTROL);order_trans_rs485(mybox.myid,0,1,2,0,CONTROL);}
if(init==1)
{
RT_FLAG=4;//�Զ����ñ��RT_FLAG=5�����Զ�RT_FLAG=4
init=0;
}
	computer_gonglu(system_status_list,slave);

	delay_time(1);

	 delay_ms(800);
  
			
			   
			     	

	}
		                                      //�������ճ�
	if(mybox.master==0)
	{
		OS_ENTER_CRITICAL();
		OSTaskSuspend(MASTER_TASK_PRIO );//����������״̬.
		OS_EXIT_CRITICAL();	
	}
	}//while
 	}

void myled_task(void *pdata)
{
while(1)
{
 warn();
 key_idset();//��������ʾ����        
delay_ms(100);
}
}




/********************************************/
void SETID_task(void *pdata)
{
        OS_CPU_SR cpu_sr=0;  	    	
          while(1)
          	{
	///	  id_num=1;//���Կ�����ʹ��
		if(id_num<1||id_num>33)
			{            		
                                      mybox.master=2;
			             OS_ENTER_CRITICAL();
                      		OSTaskSuspend(MASTER_TASK_PRIO );//����������״̬.
                                   OSTaskSuspend(Receive_TASK_PRIO);
						OS_EXIT_CRITICAL();
					HT595_Send_Byte(YELLOW_YELLOW|background_light_on);
			delay_ms(100);
		       }
               else if(id_num<=32&&id_num>=1)
               	{ 
                                  mybox.master=0;
				      mybox.myid=id_num;
				HT595_Send_Byte((GREEN_GREEN)|background_light_on);
				   OS_ENTER_CRITICAL();
		 OSTaskResume(MASTER_TASK_PRIO );//������������״̬
		 OSTaskResume(MYLED_TASK_PRIO );//������ʾ����״̬
		 OSTaskResume(Receive_TASK_PRIO );//�����ӻ�����״̬
               OSTaskSuspend(SETID_TASK_PRIO);
			   OS_EXIT_CRITICAL();	
			 }

		  }

}








/******************************************/


 /***********************************/

int slave_control(u8 i,u8 j)//������λ����ָ��	 
{ 
 if(mybox.send==0)//��̬����
   	{
	   //LED1=!LED1;
	return 0;
    }
if((tempshuzhi<temperature_gate)&&(temperature_warn==0))/***�Ǳ�����Ϣ***/
{
   if(mybox.send==1&&auto_on==1&&init_time==0) //��λ������
   	{ 	 // LED1=!LED1;
 if(i==1&&j==1)
 {
if(mystatus.work_status[0]==0&&rework_time[0]==0)
{
 GPIO_ResetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],1,mystatus.work_status[1],mystatus.work_time[0],mystatus.work_time[1]);
   if(ligt_time>0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);
      if(ligt_time==0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
 	}

 }
 if(i==1&&j==0)
 	{
if(mystatus.work_status[0]==1)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_0);

 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],3,mystatus.work_status[1],0,mystatus.work_time[1]);//״̬3Ϊ�ŵ�״̬
 rework_time[0]=1;//��time3��ʱ���򿪷ŵ�ʱ��
   if(ligt_time>0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);
      if(ligt_time==0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
 	}
 }
 if(i==2&&j==1)
 	{
if(mystatus.work_status[1]==0&&rework_time[1]==0)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],1,mystatus.work_time[0],mystatus.work_time[1]);
   if(ligt_time>0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);
      if(ligt_time==0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
 	}

 }
 if(i==2&&j==0)
 	{
if(mystatus.work_status[1]==1)
   {
	GPIO_SetBits(GPIOA,GPIO_Pin_8);

 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],3,mystatus.work_time[0],0);//״̬3Ϊ�ŵ�״̬
 rework_time[1]=1;//��time3��ʱ���򿪷ŵ�ʱ��
   if(ligt_time>0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);
      if(ligt_time==0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
 	}

 }
	return 1;
   	}
}
if(mybox.send==2)//�鿴�ӻ�״̬
 {
  status_trans_rs485(&mystatus);
return 2;
 }
if(mybox.send==3&&auto_on==1)//�鿴�ӻ�״̬
{
  status_trans_rs485_dis(&mystatus);
return 2;
 }
if(mybox.send==4&&auto_on==1)//��ʼͶ���ʱʹ�ã���֤��Ͷ��ȥ������������
{
{ 	 // LED1=!LED1;
 if(i==1&&j==1)
 {GPIO_ResetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],1,mystatus.work_status[1],mystatus.work_time[0],mystatus.work_time[1]);
   if(ligt_time>0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);
      if(ligt_time==0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
 }
 if(i==1&&j==0)
 	{GPIO_SetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,mystatus.work_status[1],0,mystatus.work_time[1]);
   if(ligt_time>0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);
      if(ligt_time==0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
 }
 if(i==2&&j==1)
 	{GPIO_ResetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],1,mystatus.work_time[0],mystatus.work_time[1]);
   if(ligt_time>0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);
      if(ligt_time==0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);

 }
 if(i==2&&j==0)
 	{GPIO_SetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],0,mystatus.work_time[0],0);
   if(ligt_time>0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);
      if(ligt_time==0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);

 }
   	}
 status_trans_rs485_RT();//�ӻ�����
return 2;
 }

 
if(mybox.send==5)//�鿴�ӻ�״̬
 {
  status_trans_rs485_scantask(&mystatus);
return 5;
 }
if(mybox.send==6&&auto_on==1)//�鿴�ӻ�״̬
 {
 { 	 
 if(i==1&&j==1)
 {GPIO_ResetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],1,mystatus.work_status[1],mystatus.work_time[0],mystatus.work_time[1]);
   if(ligt_time>0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);
      if(ligt_time==0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
 }
 if(i==1&&j==0)
 	{GPIO_SetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,mystatus.work_status[1],0,mystatus.work_time[1]);
   if(ligt_time>0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);
      if(ligt_time==0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
 }
 if(i==2&&j==1)
 	{GPIO_ResetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],1,mystatus.work_time[0],mystatus.work_time[1]);
   if(ligt_time>0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);
      if(ligt_time==0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);

 }
 if(i==2&&j==0)
 	{GPIO_SetBits(GPIOA,GPIO_Pin_8);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.work_status[0],0,mystatus.work_time[0],0);
   if(ligt_time>0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);
      if(ligt_time==0)LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);

 }
   	}
  status_trans_rs485_comm_RT();
	return 6;
 }

return 7; //����ʧ��
}

 void scanf_task(void *pdate)
 	{ u8 err;
while(1)
{
        OSSemPend(scan_slave,0,&err);
  	if(mybox.master==1)
  		{
             scanf_slave_machine();
  		}

}
}


void EXTI_Configuration(void)//��ʼ������

{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//��ʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE );	  //ʹ��ADC1ͨ��ʱ��

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	 		
	//ʹ���ⲿ�жϸ���ʱ��
	
	//ӳ��GPIOE��Pin0��EXTILine0
GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource12);



EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);         	//Ƕ�׷���Ϊ��0
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;      	//�ж�ͨ��Ϊͨ��10
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //�������ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;    		//��Ӧ���ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     		//���ж�
	NVIC_Init(&NVIC_InitStructure);
	 EXTI_GenerateSWInterrupt(EXTI_Line12);

}
void EXTI15_10_IRQHandler(void)
{
static u8 first_sen=1;

	OSIntEnter();   

  if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	
	{
delay_us(1000);
if(KEY1==1&&auto_on==0)
{
		  auto_on=1;
if((mystatus.work_status[0]==1))
	{
	GPIO_SetBits(GPIOA,GPIO_Pin_0);
rework_time[0]=1;
mystatus.work_status[0]=3;
ligt_time=16;
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);
 }

if((mystatus.work_status[1]==1))
	{
GPIO_SetBits(GPIOA,GPIO_Pin_8);
rework_time[1]=1;
mystatus.work_status[1]=3;
ligt_time=16;
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);
 }
}

if(KEY1==0&&auto_on==1&&first_sen==1)	
 {
 		first_sen=2;
 auto_on=0;
if((mystatus.work_status[0]==0)&&rework_time[0]==0)
 	{
GPIO_ResetBits(GPIOA,GPIO_Pin_0);
mystatus.work_status[0]=1;
ligt_time=16;
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);
 		}  
 }
if(KEY1==0&&auto_on==1&&first_sen==2)
 	{
 	first_sen=1;		
 	auto_on=0;
if((mystatus.work_status[1]==0)&&rework_time[1]==0)
		{
GPIO_ResetBits(GPIOA,GPIO_Pin_8);
mystatus.work_status[1]=1;
ligt_time=16;
      LIGHT(mystatus.work_status[0],mystatus.work_status[1],1);
}

 }


	}
      EXTI_ClearITPendingBit(EXTI_Line12);

	   	OSIntExit();  

}

void warn()
{
static u8 warning_flag=0;
 	temperature();
/************�ӻ����� �¶ȱ���************************/
{
if(tempshuzhi>=temperature_gate&&temperature_warn==0&&auto_on==1)
{
GPIO_SetBits(GPIOA,GPIO_Pin_0);
 set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],2,2,0,mystatus.work_time[1]);
	 delay_ms(100);
GPIO_SetBits(GPIOA,GPIO_Pin_8);
     LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
	 temperature_warn=1;
}

if(tempshuzhi<=(temperature_gate-6)&&temperature_warn==1)
{
	 temperature_warn=0;
	  rework_time[1]=1;//��time3��ʱ���򿪷ŵ�ʱ��
 rework_time[0]=1;//��time3��ʱ���򿪷ŵ�ʱ��
set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],3,3,0,mystatus.work_time[1]);
     LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);

}
}

/************�ӻ����� �¶ȱ���END***********************/	

//if(mybox.master==0)
{
/**************************************��ѹ����**
{

if((dianya_zhi>(warn_volt_onlimt+400)||dianya_zhi<330)&&warning_flag==0)
{
mystatus.work_status[0]=2;
mystatus.work_status[1]=2;
	
LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);
warning_flag=1;
}
if(warning_flag==1&&dianya_zhi<=(warn_volt_onlimt+400-3)&&dianya_zhi>=333)
	{

mystatus.work_status[0]=0;
mystatus.work_status[1]=0;
	
LIGHT(mystatus.work_status[0],mystatus.work_status[1],0);

	warning_flag=0;
     }
}
***********************************��ѹ����END**/

}

}
