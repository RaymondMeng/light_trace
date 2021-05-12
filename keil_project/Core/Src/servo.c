/**
  ******************************************************************************
  * @file    servo.c
  * @author  MengCheng
  * @version V1.0.0
  * @date    2021.4.2
  * @brief   舵机驱动函数
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */
/*********************************************  includes  *****************************************/
#include "servo.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "math.h"


/*********************************************  defines  *****************************************/
//#define Error_Threshold_BigValue 10                                        //误差较大阈值，需后期调节
//#define Error_Threshold_SmallValue 5                                       //误差较小阈值，需后期调节
//#define rotation_adjust_angle1 Error_Threshold_BigValue * (160 - Real_x)   //计算旋转角
//#define pitch_adjust_angle1 Error_Threshold_BigValue * (112 - Real_y)      //计算旋转角
//#define rotation_adjust_angle2 Error_Threshold_SmallValue * (160 - Real_x) / 100 //计算俯仰角          
//#define pitch_adjust_angle2 Error_Threshold_SmallValue * (112 - Real_y) / 100   //计算俯仰角                         
//#define speed_allocate_k 100

/*********************************************  variables  *****************************************/
uint8_t Servo_Status = 1;        //舵机状态：停止或运动 
//uint8_t Target_Flag = No_Target; //串口接收到数据并判断后赋值
//double x, y;                   //用于计算调整角度

struct PID positionx_pid = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  //一定要初始化，外部引用的时候
struct PID positiony_pid = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
 


/*********************************************  functions  *****************************************/

/**
  * @brief  舵机驱动初始化函数
  * @param  None
  * @retval 函数执行状态
  */
HAL_StatusTypeDef Servo_Init(void)
{
	//HAL_StatusTypeDef status = (HAL_StatusTypeDef)(Servo_SetAngles(1, 1, 90, 0) || Servo_SetAngles(1, 2, 90, 0)); //之所以后面没被执行，是因为被短路了
	HAL_StatusTypeDef status = (HAL_StatusTypeDef)(Tim_SetPWM(1, 1, 1500) || Tim_SetPWM(1, 2, 1500));  
  HAL_Delay(1000);
	return status;
}

///**
//  * @brief  舵机角度转换pwm函数
//  * @param  angles:角度
//  * @param  tim：定时器几
//  * @param  channal:通道几
//  * @retval 函数执行状态
//  * @claim  数字舵机500us~2500us,180°或者270°，需要测试一下
//  */
//HAL_StatusTypeDef Servo_SetAngles(uint8_t tim, uint8_t channal, uint32_t pulses)
//{
//  return 
//	if(mode == 1) //模式1：巡检模式，慢速转动
//	{
//		for(i = 1; i <= speed_allocate_k; i++)
//    {
//			if(Tim_SetPWM(tim, channal, angles / speed_allocate_k * i) == HAL_OK)
//      {
//        HAL_Delay(40);
//        if(!Servo_Status) //如果有目标出现，那么舵机停止转动，云台进行微调模式
//          break;
//      }
//      else
//          return HAL_ERROR;
//        //C语言中三目运算符的表达式1，2的类型必须一致
//        //			HAL_Delay(40);
//			//if(串口函数里发来了检测到了物体的指令)
//			//那么传值Target_Detected,否则返回NO_target
//		}
//		return HAL_OK;
  //}
//	else //模式0：定角模式，快速转动
//		return Tim_SetPWM(tim, channal, angles);
//}

///**
//  * @brief  云台巡检函数，寻找目标物
//  * @param  None
//  * @retval HAL_Status: 目标物找寻结果
//  * @claim  其实也可以考虑用break,但是需要while()里变成1
//  */
//HAL_StatusTypeDef Servo_Platform_Search()
//{     
//	uint8_t bit = 0;
//  uint8_t flag = 0;
//  
//	while(flag == 0)
//	{
//		if(Target_Flag == Target_Detected)
//    {
//			//检测到了立即停转
//			Servo_Status = 0;
//      flag = 1; //跳出循环
//    }
//    else
//    {
//      Servo_SetAngles(1, 2, 45+90*bit, 1);
//      bit = bit ^ 0x01; //如果bit是用bool类型定义的，那么取反得用！
////      bit = 0;
//      HAL_Delay(500);
//    }
//	}
//  return HAL_OK;
//}


/*方案一*/
/**
  * @brief  云台目标跟踪函数
  * @param  target_x: 目标物X坐标
  * @param  target_y: 目标物y坐标
  * @param  current_angle: 当前角度
  * @retval None
  */
//void Servo_Platform_Track(uint16_t target_x, uint16_t target_y)
//{
//  /*误差较大时，进入大幅调整阶段*/
////  while(Target_Location_Error >= Error_Threshold_BigValue) //如果误差小于阈值，则锁定
////  {
////    Servo_SetAngles(1, 1, Global_Current_Angles+rotation_adjust_angle1, 0);
////    Servo_SetAngles(1, 2, Global_Current_Angles+pitch_adjust_angle1, 0);
////  }
//  /*误差较小时，进入小幅度调整阶段*/
//  while(Target_Location_Error >= Error_Threshold_SmallValue) //如果误差小于阈值，则锁定
//  {
//    Servo_SetAngles(1, 1, Global_Current_Angles_x+rotation_adjust_angle2, 0);
//    Servo_SetAngles(1, 2, Global_Current_Angles_y+pitch_adjust_angle2, 0);
//  }    
//}

/*PID静态参数设置*/
void PID_Init()
{
  positionx_pid.kp = 0.15;
  positionx_pid.ki = 0.001;
  positionx_pid.kd = 0.001;
  positionx_pid.passive_error = 0; //上一次误差量
  positionx_pid.integral = 0; //上一次积分量
  positionx_pid.set_value = 0x9f; //激光偏心x轴0x01
  positionx_pid.output = 1500;
  
  positiony_pid.kp = 0.15;
  positiony_pid.ki = 0;
  positiony_pid.kd = 0;
  positiony_pid.passive_error = 0; //上一次误差量
  positiony_pid.integral = 0; //上一次积分量
  positiony_pid.set_value = 0x78 - 0x18; //激光偏心y轴0x60
  positiony_pid.output = 1500;
}
	

