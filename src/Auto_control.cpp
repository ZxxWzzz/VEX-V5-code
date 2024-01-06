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
备    注： 左旋转陀螺仪返回值为负，右旋转为正
\*---------------END--------------*/
void Auto_function_far(void)
{
   Task_state(false);

  //{展开推预装球}
  Auto_Pump(true);
  sleep(500);
  Auto_Pump(false);
  sleep(250);

  /*========中间球========*/
  Rollmotor.spin(vex::directionType::fwd,8,volt);

  RunpidStraightNTo(85,1350,5,25,1200,950,-30);

  sleep(250);

  TurnpidNTo(75, 130, 1, 520);

  sleep(450);

  Rollmotor.spin(vex::directionType::rev,12,volt);
  sleep(350);
  Rollmotor.stop();
  /*=====================*/

  /*========第二球========*/
  TurnpidNTo(75, -164, 1, 580);

  sleep(200);

  Rollmotor.spin(vex::directionType::fwd,8,volt);

  RunpidStraightNTo(85,740,5,35,650,700,0);

  sleep(350);

  RunpidStraightNTo(40,-250,5,35,200,380,0);  //第三球拿到后倒退防磕碰

  sleep(250);

  TurnpidNTo(75, 180, 1, 620);

  sleep(200);

  Auto_Pump(true);
  Rollmotor.stop();

  Chassis_Run(40, 40);
  sleep(450);
  Chassis_Stop();

  Auto_Pump(false);
  sleep(300);
  /*=====================*/

  /*========第三球========*/
  RunpidStraightNTo(25,-350,5,5,200,350,0);

  sleep(200);

  TurnpidNTo(80, 145, 1, 600);

  sleep(200);

  Rollmotor.spin(vex::directionType::fwd,8,volt);
  RunpidStraightNTo(85,1400,5,45,1200,850,0);
  sleep(300);

  RunpidStraightNTo(40,-250,5,35,200,380,0);  //第三球拿到后倒退防磕碰

  TurnpidNTo(70, 67, 1, 500);

  sleep(200);

  RunpidStraightNTo(85,-1500,5,55,1300,880,70);

  sleep(200);

  Rollmotor.stop();
  Chassis_Run(40, 40);
  sleep(1000);
  Chassis_Stop();

  Chassis_Run(-20, -20);
  sleep(250);
  Chassis_Stop(2);

  sleep(500);
  TurnpidNTo(75, -90, 1, 500);

}

/*===========================================================================*/

void Auto_function_near(void){

}

void Auto_test(){

  Task_state(false);

  Rollmotor.spin(vex::directionType::fwd,8,volt);

  RunpidStraightNTo(25,300,5,5,200,350,0);

  sleep(300);

  RunpidStraightNTo(95,-1580,5,85,1450,900,0);

  sleep(200);

  TurnpidNTo(65, -50, 1, 400);

  sleep(200);

  RunpidStraightNTo(95,-750,5,75,600,550,0);

  Auto_Pump(true);

  Rollmotor.stop();

  sleep(350);

  TurnpidNTo(50, -210, 1, 950);

  sleep(350);

  RunpidStraightNTo(45,300,5,30,250,300,-6);

  sleep(200);

  Rollmotor.spin(vex::directionType::rev,8,volt);

  sleep(200);
  Auto_Pump(false);

  Chassis_Run(85, 85);
  sleep(650);
  Chassis_Stop();

  Rollmotor.stop();
  Auto_Pump(false);

  sleep(200);

  RunpidStraightNTo(75,-420,5,70,400,420,0);

  sleep(200);

  TurnpidNTo(65, -78, 1, 400);

  sleep(200);

  Rollmotor.spin(vex::directionType::fwd,8,volt);
  RunpidStraightNTo(85,2350,5,70,2200,1400,-3);

  sleep(200);

  TurnpidNTo(65, 90, 1, 560);

  sleep(200);

  RunpidStraightNTo(65,400,5,30,350,420,25);

  Auto_Pump(true);
  TurnpidNTo(65, 70, 1, 400);

  sleep(100);

  Rollmotor.stop();
  Chassis_Run(85, 85);
  sleep(650);
  Chassis_Stop();
  Auto_Pump(false);

  RunpidStraightNTo(85,-330,5,70,300,400,0);

  sleep(200);

  TurnpidNTo(85, 185, 1, 670);

  sleep(200);

  RunpidStraightNTo(95,880,5,75,810,810,0);
  Rollmotor.spin(vex::directionType::fwd,8,volt);

  sleep(100);

  TurnpidNTo(80, 175, 1, 580);

  Rollmotor.stop();
  Chassis_Run(85, 85);
  sleep(650);
  Chassis_Stop();
}