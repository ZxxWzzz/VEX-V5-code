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

  Chassis_Run(-9.5,-4);
  wait(950, msec);
  Chassis_Stop();

  Chassis_Run(-50,-50);
  wait(450, msec);
  Chassis_Stop();

  Chassis_Forward(150,0,true); 

  Chassis_Turn(160,35); 

  Chassis_Run(50,50);
  Rollmotor.stop();
  wait(550, msec);
  Chassis_Stop();

  Chassis_Forward(-300,0,true);
  Chassis_Turn(-50,35); //朝第三球方向，可调整
  /*========================*/

  /*=====第二阶段（一球）=====*/
  Rollmotor.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
  Chassis_Forward(980,0,true); //前进收第三球

  Chassis_Turn(115,35); //冲撞角度  需要调整！！！
  Chassis_Forward(400,0,true);
  Chassis_Turn(26); //调整角度  需要调整！！！

  Rollmotor.stop();

  Chassis_Run(70,70);
  wait(500, msec);
  Chassis_Stop();

  wait(10, msec); //test

  /*========================*/

  /*=====第三阶段（两球）=====*/

  Rollmotor.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);

  Chassis_DriveToAngle(-140, 5, -0.5); //倒退单电机角度！！！重点调整

  Chassis_Forward(550,0,true,45); //第四球前进距离！！！ 重点调整
  wait(500, msec);

  Chassis_Turn(95,35);  

  Chassis_Run(50, 50);
  wait(700, msec);
  Chassis_Stop();
  Rollmotor.stop();
  /*========================*/


}
/*===========================================================================*/

void Auto_function_near(void){
  /*========角球========*/
  cegua.spin(reverse, 100, pct);
  wait(650, msec);  // 等待0.65秒
  cegua.stop();     // 停止电机
  cegua.setBrake(vex::brakeType::hold);  // 锁死电机
  /*===================*/

}