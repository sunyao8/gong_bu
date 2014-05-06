#ifndef __485ANN_H
#define __485ANN_H			 
#include "sys.h"	 
#include "rs485.h"
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"	 
#include "24cxx.h"
#include "includes.h" 	 
#include <math.h>
#include "adc.h"
#include "timer.h"								  
#include "Ht1621.h"
#include "stm32f10x_exti.h"
 
 typedef struct  
{ 
  u8 start;
  u8 myid;      //��������ID��
  u8 source;
  u8 destination; //Ŀ�ĵ�����
  u8 send;      //�Ƿ��Ƿ�������1Ϊ�ǣ�0Ϊ����
  u8 relay;    //�ڼ��������
  u8 message;     //������Ϣ
  u8 master;      //��������
  u8 end;   
}box;
#define LEN_control 15
#define LEN_status 10
#define CPT_LL                                                    '^'
#define CONTROL                                                '/'

//#define RS485_TX_EN		PGout(9)	//485ģʽ����.0,����;1,����.��������
#define RS485_TX_EN		PBout(15)	//485ģʽ����.0,����;1,����.��������
//����봮���жϽ��գ��벻Ҫע�����º궨��
#define EN_USART2_RX 	1			//0,������;1,����.

#define TIME_OUT 200

#define ALL_NODE_LCD_UNLOCK 3
#define ALL_NODE_LCD_LOCK 4
#define IDLE_NODE_LCD_LOCK  5
#define BUSY_NODE_LCD_LCOK 6
#define NODE_LCD_LOCK_BASE 80
#define NODE_LCD_LOCK_OLDFUN 11




#define FIND_ALL_STATUS 7
#define FIND_IDLE_STATUS  8
#define FIND_BUSY_NODE_STATUS 9
#define Sub_Order 10

#define second 1

#define Receive_TASK_PRIO       			2 

#define MASTER_TASK_PRIO       			3

/***********************************/
 typedef struct  
{ 
  u8 myid;      //��������ID��
  u8 size[2];      //������λǧ��
  u8 work_status[2];    //����״̬ 1 ΪͶ�빤����0 Ϊû�й���
  u8 work_time[2];     //����ʱ��   
}status_box;


#define status_LEN 4




 typedef struct  
{ 
  u8 myid;      //��������ID��
  u8 size;      //������λǧ��
  u8 work_status;    //����״̬ 1 ΪͶ�빤����0 Ϊû�й���
  u8 work_time;     //����ʱ��  
  u8 group;
}status_list_node;



typedef struct
{
  u8 myid;
  u8 size;

}idle_list;

typedef struct
{
  u8 myid;
  u8 group;
  u8 size;
}turn_node;

typedef struct
{
  u8 myid;
  u8 group;
  u8 size;
}offset_node;


typedef struct
{
  u8 myid;
  u8 size;
  u8 work_time;

}busy_list;

 //////////////////////////////////////////////////////////////////// 
/*
typedef struct
{
   u8 myid;
   u8 work_time;
   u8 size;

 }time_list;
*/
void turn_master_id(u8);
void initmybox(void);
void TIM4_Int_Init(u16,u16);
void TIM3_Int_Init(u16,u16);
void order_trans_rs485(u8,u8,u8,u8,u8,u8);
int rs485_trans_order(u8 *);
void rs485_trans_status(u8 *);
void status_trans_rs485(status_box *);
void set_now_mystatus(u8 ,u8 ,u8 ,u8,u8,u8,u8);
void init_mystatus(u8 ,u8 ,u8,u8,u8,u8);
void set_statuslist_1(u8,u8,u8,u8,u8,u8);
void set_statuslist_2(u8,u8,u8,u8,u8,u8);
void delay_time(u32);//��ϵͳ����ʱ������time*450ms
u8 inquiry_slave_status(u8,u8);//��ѯ�ӻ�״̬�����浽�ӻ�״̬���У�����id��Ҫ��ѯ�Ĵӻ���
void gonglvyinshu(void);
void allphase(long *V,long *I);
void temperature(void);   //�������¶ȼ��
u8 sort_busynode_list(busy_list *,status_list_node *);
u8 sort_busynode_list_asc(busy_list *,status_list_node *);
u8 sort_idlenode_list(idle_list *,status_list_node *);
u8 turn_idlenode_list(turn_node *,status_list_node *,status_list_node *);//�����������(��������С�ɴ�С���У����ؿ��нڵ����)
void myled(void);
void try(void);
u16 comp_16(u16 ,u16 );
void led_on_off(u8,u8 ) ;
extern int slave_control(u8,u8);
void heartbeat(u8);
void Alarm(void);
void key_lcd(void);
void LIGHT(u8,u8,u8);
void scanf_slave_machine(void);
 void status_trans_rs485_dis(status_box *mystatus);//�ӻ�����
 void status_trans_rs485_RT(void);//�ӻ�����
  void status_trans_rs485_comm_RT(void);//�ӻ�����
 void status_trans_rs485_scantask(status_box *mystatus);//�ӻ�����
void init_Queue(status_list_node *comm_list,u8 *slave_comm ,u8 group);
void change_Queue(u8 list_flag,u8 Level, status_list_node *comm_list_1,status_list_node *comm_list_2,u8 *slave_comm);
u8 computer_gonglu(status_list_node *comm_list_1,status_list_node *comm_list_2,u8 *slave_comm);

//u8 sub_delaytime_15(u8);
//u8 sub_delaytime_5(u8 );


#endif	   
















