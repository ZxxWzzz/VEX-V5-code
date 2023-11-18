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

/*========自动阶段测试======*/
// Chassis_Forward(200,0,true,45);
// Chassis_Forward(-200,0,true,45);
/*========================*/



/*=========第一段==========*/
  Rollmotor.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
  Chassis_Forward(200,0,true,65);
  Chassis_Forward(-650,0,true,45);

  Chassis_Run(-7.5,-3);
  wait(1000, msec);
  Chassis_Stop();
  wait(200, msec);

  Chassis_Run(-50,-50);
  wait(350, msec);
  Chassis_Stop();

  Chassis_Forward(150,0,true,35);
  Chassis_Stop();

  Chassis_Turn(160,35);
  Chassis_Run(50,50);
  wait(350, msec);
  Chassis_Stop();

  Chassis_Forward(-330,0,true,45);
  Chassis_Turn(-50,35);

/*========================*/


}
/*===========================================================================*/
