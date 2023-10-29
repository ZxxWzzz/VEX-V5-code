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
  Chassis_Forward(380,0,true,1);

  //Rollmotor.spin(vex::directionType::fwd,100,vex::voltageUnits::volt);    
  //left1.spin(vex::directionType::fwd,100,vex::voltageUnits::volt);  left2.spin(vex::directionType::fwd,100,vex::voltageUnits::volt); left3.spin(vex::directionType::fwd,100,vex::voltageUnits::volt);
  //right1.spin(vex::directionType::fwd,100,vex::voltageUnits::volt);  right2.spin(vex::directionType::fwd,100,vex::voltageUnits::volt); right3.spin(vex::directionType::fwd,100,vex::voltageUnits::volt);


}
/*===========================================================================*/
