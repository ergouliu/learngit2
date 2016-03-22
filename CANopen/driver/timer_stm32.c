/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN
STM32 Port: CHEN BING

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include "stm32f2xx.h"

#include "stm32f2xx_tim.h"


// Includes for the Canfestival driver

#include "canfestival.h"
#include "timer.h"


// Define the timer registers
//#define TimerAlarm      TIM2->ARR
//#define TimerCounter    TIM2->CNT
static unsigned int debug_bit=0;
static unsigned int TimerAlarm=65535;
//#define TIMER_INTERRUPT_LEVEL		1



void timer_can_irq_handler(void);


/************************** Modul variables **********************************/
// Store the last timer value to calculate the elapsed time
static TIMEVAL last_time_set = TIMEVAL_MAX;

void initTimer(void)
/******************************************************************************
Initializes the timer, turn on the interrupt and put the interrupt time to zero
INPUT	void
OUTPUT	void
******************************************************************************/
{
TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
//TIM_OCInitTypeDef TIM_OCInitStructure;

  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

//TIM2 使用内部时钟
TIM_DeInit(TIM4);
TIM_InternalClockConfig(TIM4);

//TIM4基本设置
//设置预分频器分频系数71，即APB2=72M, TIM1_CLK=72/72=1MHz
//TIM_Period（TIM2_ARR）=1000，计数器向上计数到1000后产生更新事件，计数值归零
//向上计数模式
//TIM_RepetitionCounter(TIM1_RCR)=0，每次向上溢出都产生更新事件
TIM_BaseInitStructure.TIM_Period = 65535;
TIM_BaseInitStructure.TIM_Prescaler = 71;
TIM_BaseInitStructure.TIM_ClockDivision = 0;
TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_BaseInitStructure.TIM_RepetitionCounter = 0;
TIM_TimeBaseInit(TIM4, &TIM_BaseInitStructure);

//清中断，以免一启用中断后立即产生中断
TIM_ClearFlag(TIM4, TIM_FLAG_Update);
//使能TIM4中断源
TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

//TIM4总开关：开启
TIM_Cmd(TIM4, ENABLE);

}

void setTimer(TIMEVAL value)
/******************************************************************************
Set the timer for the next alarm.
INPUT	value TIMEVAL (unsigned long)
OUTPUT	void
******************************************************************************/
{
 
  TIM_SetAutoreload(TIM4, value);
 
}

TIMEVAL getElapsedTime(void)
/******************************************************************************
Return the elapsed time to tell the stack how much time is spent since last call.
INPUT	void
OUTPUT	value TIMEVAL (unsigned long) the elapsed time
******************************************************************************/
{
 
    unsigned int timer = TIM_GetCounter(TIM4);	// Copy the value of the running timer
  // Calculate the time difference
  return timer > last_time_set ? timer - last_time_set : last_time_set - timer;

}

//*----------------------------------------------------------------------------
//* Function Name       : timer_can_irq_handler
//* Object              : C handler interrupt function by the interrupts
//*                       assembling routine
//* Output Parameters   : calls TimeDispatch
//*----------------------------------------------------------------------------

//void timer_can_irq_handler(void)
void TIM4_IRQHandler(void)
{
	UNS64 i;
 if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
{

 TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
// i++;
	
	// for(i=0;i<25000;i++);
	
	//debug
//if(debug_bit==0)
//{
//debug_bit=1;
////GPIO_SetBits(GPIOD,GPIO_Pin_13);
//}
//else
//{
//debug_bit=0;
////GPIO_ResetBits(GPIOD,GPIO_Pin_13);
//}
//debug												\
//  last_time_set = TimerCounter;
  last_time_set = TIM_GetCounter(TIM4);
  
  TimeDispatch();	// Call the time handler of the stack to adapt the elapsed time

}

} 

//void TIM2_IRQHandler(void)
//{
// if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
//{

// TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
// //debug
//if(debug_bit==0)
//{
//debug_bit=1;
//GPIO_SetBits(GPIOD,GPIO_Pin_13);
//}
//else
//{
//debug_bit=0;
//GPIO_ResetBits(GPIOD,GPIO_Pin_13);
//}
////debug												\
////  last_time_set = TimerCounter;
//  last_time_set = TIM_GetCounter(TIM2);
//  
//  TimeDispatch();	// Call the time handler of the stack to adapt the elapsed time

//}

//} 