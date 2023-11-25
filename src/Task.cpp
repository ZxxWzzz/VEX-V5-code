/*------------------------头文件引用部分----------------------------*/
#include "Chassis.h"
#include "Vex.h"
#include "Task.h"
#include "robot-config.h"
/*------------------------工程相关信息------------------------------*\
作    者：翁老师
文 件 名：多任务文件
适用机型：全部
版    本：V1.0
详细内容：本文件用于机器人多任务的设定，请将任务在此处开启和设置，切勿多处设置
         而导致程度可读性变差！
并行情况：   
    任务数量：( 2 )
        任务一：遥控底盘控制任务
        任务二：显示屏UI显示任务

更新内容：首次编写
=====================================================================
\*-----------------------------END---------------------------------*/


/*------------------------系统自带函数库----------------------------*\
task name = task(function);  新建一个名为'name'的任务，任务执行'function'. (注：'function'需定义为有整型返回值)
name.suspend();              名为'name'的任务暂停
name.resume();               名为'name'的任务恢复
name.stop();                 名为'name'的任务终止
name.sleep(i);               名为'name'的任务休眠'i'毫秒
=====================================================================
\*-----------------------------END---------------------------------*/

task Task1_Chassis        = task(Task1_Chassis_fun); //任务一：建立遥控底盘控制任务
task Task2_UI_Brain       = task(Task2_UI_fun);      //任务二：建立底盘端的显示屏任务
task Task3_UI_Controller  = task(Task3_UI_fun);      //任务三：建立遥控器端的显示任务
task Task4_STOP_f  = task(Task_STOP);      //任务二：建立遥控底盘控制任务
/*===========================================================================*/


/*----------任务一回调函数-------------*\
函数功能：任务一回调函数
依    赖：Chassis_Controller函数
输入变量： 无
返 回 值： 无
\*---------------END------------------*/
int Task1_Chassis_fun()
{
  while(1)
  {
    Chassis_Controller();     //底盘控制程序
    wait(5, msec); //5ms进行一次遥控采集
  }
}
/*===========================================================================*/


/*----------任务二回调函数-------------*\
函数功能：机器人CPU屏幕显示部分
依    赖：无
输入变量： 无
返 回 值： 无
\*---------------END------------------*/
int Task2_UI_fun()
{
  Brain.Screen.setFont(prop40);
  Brain.Screen.clearScreen();
  while(1)
  {
    Brain.Screen.printAt(1, 40, "Gyro = %.3f",Chassis_Angle());
    Brain.Screen.printAt(1, 80, "Left = %.1fmm",left1.rotation(deg)/2.637f);
    Brain.Screen.printAt(1, 120,"Right = %.1fmm",right1.rotation(deg)/2.637f);
    //Brain.Screen.printAt(1, 160, "Gyro = %.3f",Chassis_Angle());
    //Brain.Screen.printAt(1, 200, "Gyro = %.3f",Chassis_Angle());
    wait(10, msec); //5ms进行一次遥控采集
  }
}
/*===========================================================================*/


/*----------任务三回调函数-------------*\
函数功能：遥控手柄部分显示部分
依    赖：无
输入变量： 无
返 回 值： 无
\*---------------END------------------*/
int Task3_UI_fun()
{
  while(1)
  {
    Controller1.Screen.clearLine();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Gy = %.1f,Er = %.1f",Chassis_Angle(),UI_test_data);
    wait(200, msec);
  }
}
/*===========================================================================*/


/*----------多任务启停函数-------------*\
函数功能：遥控手柄部分显示部分
依    赖：无
输入变量： task_state: false表示多任务暂停 true表示多任务恢复
返 回 值： 无
\*---------------END------------------*/
void Task_state(bool task_state)
{
  if(task_state == false)
  {
    Task1_Chassis.suspend();
    Task2_UI_Brain.suspend();
    Task3_UI_Controller.suspend();
  }
  else
  {
    Task1_Chassis.resume();
    Task2_UI_Brain.resume();
    Task3_UI_Controller.resume();
  }
}
/*===========================================================================*/


/*----------跑飞监测函数-------------*\
函数功能：遥控手柄部分显示部分
依    赖：无
输入变量： task_state: false表示多任务暂停 true表示多任务恢复
返 回 值： 无
\*---------------END------------------*/
int Task_STOP()
{
  while(1)
  {
    if(Controller1.ButtonDown.pressing()&&Controller1.ButtonLeft.pressing())
    {
      // while(1)
      //   Chassis_Stop(3);
    }
    wait(100, msec);
  }
}
/*===========================================================================*/

