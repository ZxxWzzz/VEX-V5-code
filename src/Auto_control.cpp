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
void Auto_function(void)
{
  Task_state(false);

/*========自动阶段测试======*/
  // Chassis_DriveToAngle(-90, -3.6, 0);

// Chassis_Turn(90);
// Chassis_DriveToAngle(90, 5, 0);

/*========================*/



/*=======第一段（两球）=====*/
  Rollmotor.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
  Chassis_Forward(100,0,true); 
  Chassis_Forward(-700,0,true);

  Chassis_Run(-9.5,-4);
  wait(1000, msec);
  Chassis_Stop();

  Chassis_Run(-50,-50);
  wait(350, msec);
  Chassis_Stop();

  Chassis_Forward(150,0,true,35); //慢
  Chassis_Stop();

  Chassis_Turn(160,35);
  Chassis_Run(50,50);
  Rollmotor.stop();
  wait(550, msec);
  Chassis_Stop();

  Chassis_Forward(-300,0,true,45);
  Chassis_Turn(-50,35);
/*========================*/

/*=====第二阶段（一球）=====*/
  Rollmotor.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
  Chassis_Forward(980,0,true);

  Chassis_Turn(128,35); //冲撞角度  需要调整！！！

  Chassis_Run(70,70);
  wait(500, msec);
  Chassis_Run(12,12);
  wait(200, msec);
  Chassis_Stop(3);
/*========================*/

  wait(500, msec);  //测试

/*=====第三阶段（两球）=====*/
  Rollmotor.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);

  Chassis_DriveToAngle(-140, -3, 0);

  wait(500, msec);  //测试

/*========================*/


}
/*===========================================================================*/
