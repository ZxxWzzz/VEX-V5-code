/*------------------------头文件引用部分----------------------------*/
#include "User_control.h"
#include "Task.h"
#include "Chassis.h"
#include "robot-config.h"
#include "math_function.h"
#include "Operate.h"
/*------------------------工程相关信息------------------------------*\
作    者：翁老师
文 件 名：手动程序部分文件
适用机型：全部
版    本：V1.0
详细内容：本文件用于机器人手动部分程序的编写和使用
更新内容：首次编写

=====================================================================
\*-----------------------------END---------------------------------*/

/*------------手动程序-------------*\
函数功能： 手动运行时的程序
依    赖： 无
输入变量： 无
返 回 值： 无
\*---------------END------------------*/

  
/*===========================================================================*/

void  User_function(void)
{
  Task_state(false);      //所有任务停止
  // Gyro_calibrate();
  Task_state(true);       //所有任务恢复
  while(1)
  {
    // ==========================================遥控程序，按键可设置================================================
    Rollmotor.spin(vex::directionType::fwd,20,vex::velocityUnits::pct);

    climberbake(Controller1.ButtonDown.pressing());                             //高挂自动回收
    shooter(Controller1.ButtonY.pressing());                                    //弹射：按着就启动，松开就停
    climber(Controller1.ButtonL2.pressing(),Controller1.ButtonL1.pressing());   //爬升：一个上，一个下，松开就停
    roller1(Controller1.ButtonR1.pressing(),Controller1.ButtonR2.pressing());    //按下就一直转，有防堵转
    Pump(Controller1.ButtonA.pressing());         //侧挂程序
    // ============================================================================================================
    // Task1_Chassis.resume();
    wait(10,msec);
  }


  while(1) wait(100, msec);
}