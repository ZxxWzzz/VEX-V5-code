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

/*------------角度检测-------------*\
函数功能： 检测是否到达指定角度
依    赖： 陀螺仪
输入变量： 期望角度
返 回 值： bool-用于判断
\*---------------END--------------*/

bool angleDetection(double targetAngle) {
    double currentAngle = Chassis_Angle(0); // 读取当前角度
    const double tolerance = 2.0; // 允许误差范围
    Controller1.Screen.clearLine(1); // 清除之前的输出
    Controller1.Screen.print("Auto Angle: %.2f", currentAngle); 

    // 检查当前角度是否在目标角度加减容差范围内
    return fabs(currentAngle - targetAngle) <= tolerance;
}



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

  /*========自动阶段测试======*/
  // Chassis_DriveToAngle(-130, 4, -0.5);

  // Chassis_Turn(90);
  // Chassis_DriveToAngle(90, 5, 0);

  /*========================*/



  /*=======第一段（两球）=====*/
  Rollmotor.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
  Chassis_Forward(100,0,true); 
  Chassis_Forward(-680,0,true);

  Chassis_Run(-9.5,-5);
  wait(850, msec);
  Chassis_Stop();

  Chassis_Run(-50,-50);
  wait(450, msec);
  Chassis_Stop();

  Chassis_Run(5,5);
  wait(250, msec);
  Chassis_Stop(3);
  // Chassis_Forward(150,0,true); 

  Chassis_Turn(160,35); 

  Chassis_Run(70,70);
  Rollmotor.stop();
  wait(550, msec);
  Chassis_Stop();

  Chassis_Forward(-300,0,true);
  Chassis_Turn(-45,35); //朝第三球方向，可调整
  /*========================*/

  /*=====第二阶段（一球）=====*/
  Rollmotor.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
  Chassis_Forward(990,0,true); //前进收第三球
  wait(200, msec);


  Chassis_Turn(115,35); //冲撞角度  需要调整！！！
  Chassis_Forward(420,0,true);
  Chassis_Turn(26); //调整角度  需要调整！！！

  Rollmotor.stop();

  Chassis_Run(70,70);
  wait(550, msec);
  Chassis_Stop();

  wait(10, msec); //test

  /*========================*/

  /*=====第三阶段（两球）=====*/

  Rollmotor.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);

  Chassis_DriveToAngle(-130, 4.5, -0.5); //倒退单电机角度！！！重点调整4
  Chassis_Stop(3);
  wait(100, msec);

  Chassis_Forward(560,-45,true,45); //第四球前进距离！！！ 重点调整
  wait(350, msec);

  Chassis_Run(-5,-5);
  wait(200, msec);
  Chassis_Stop();
  // Chassis_Forward(-100,0,true,45); //倒退防撞

  Chassis_Turn(115,35);  

  Chassis_Run(70, 70);
  wait(700, msec);
  Chassis_Stop();
  Rollmotor.stop();
  /*========================*/


}
/*===========================================================================*/

void Auto_function_near(void){

  Task_state(false);


  /*========角球========*/
  Rollmotor.spin(vex::directionType::fwd,10,vex::velocityUnits::pct);

  cegua.spin(reverse, 100, pct);  //侧挂电机运行
  wait(650, msec); 
  cegua.stop();     
  cegua.setBrake(vex::brakeType::hold);  // 锁死电机

  Chassis_Turn(-90,50);
  wait(100, msec);
  Chassis_Forward(850,0,true); //前进

  /*===================*/

  /*=======预装球=======*/
  Chassis_Turn(-110,50);
  wait(250, msec);

  Chassis_Run(50, 50);
  wait(700, msec);
  Chassis_Stop();
  /*====================*/

  cegua.spin(fwd, 100, pct);  //侧挂电机回收
  wait(650, msec); 
  cegua.stop();  

  /*=======后撤=========*/
  Chassis_Forward(-155,0,true); 
  wait(100, msec);
  Chassis_Turn(-85,35);

  Chassis_Forward(900,0,true); 
  Chassis_Turn(-90,30);

  Climbmotor.spinFor(reverse,690,degrees,100,velocityUnits::pct);

  Chassis_Forward(540,0,true); 

  /*====================*/

}

void Auto_test(){

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
  sleep(700);
  Chassis_Stop();

  Chassis_Run(-20, -20);
  sleep(250);
  Chassis_Stop(2);


  sleep(500);
  TurnpidNTo(75, -90, 1, 500);

}