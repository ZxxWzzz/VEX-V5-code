/*------------------------头文件引用部分----------------------------*/
#include "Auto_control.h"
#include "Operate.h"
#include "robot-config.h"
#include "Chassis.h"
#include "Task.h"




/*------------------------工程相关信息------------------------------*\
作    者：翁老师
文 件 名：自动程序部分文件
适用机型：全部
版    本：V1.0
详细内容：本文件用于机器人自动部分程序的编写和使用
更新内容：首次编写

=====================================================================
\*-----------------------------END---------------------------------*/


/*------------自动程序-------------*\
函数功能： 比赛时15s的自动程序代码
依    赖： 无
输入变量： 无
返 回 值： 无
\*---------------END------------------*/
void Auto_function(void)
{
  Task_state(false);
/*=========第一段==========*/
  Rollmotor.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
  Chassis_Run(30,30);
  wait(350, msec);
  Chassis_Forward(-710,0,true,1);
  Chassis_Turn(-45);
  Chassis_Forward(-270,0,true,1);
  Chassis_Turn(-45);

/*========================*/

  // Chassis_Turn(90);
  // wait(100, msec);
  // Chassis_Turn(-90);
  // wait(300, msec);
  // Chassis_Forward(180,0,true,1);
  // wait(300, msec);
  // Chassis_Forward(-180,0,true,1);

}
/*===========================================================================*/
