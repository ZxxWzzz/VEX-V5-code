#include "robot-config.h"
#include "vex.h"
#include "Chassis.h"
#include "Task.h"
#include "User_control.h"
#include "Auto_control.h"
// ---- START VEXCODE CONFIGURED DEVICES ----

/*------------------------工程相关信息------------------------------*\
作    者：翁老师
工 程 名：Vex机器人工程模板
适用机型：全部
版    本：V1.0
详细内容：本工程用于后续Vex机器人代码编写的模板
按键定义/操作说明：

更新内容：

=====================================================================
\*-----------------------------END---------------------------------*/

// ---- END VEXCODE CONFIGURED DEVICES ----

competition Competition;

//多线程显示区域
void pre_auton(void)   //**
{
  vexcodeInit();
  /*--------------------------------*/    
}

//自动模式(三选一)
void autonomous(void){
  
  // Auto_function_far();
  // Auto_function_near();
  Auto_test();

}

//手动模式
void usercontrol(void){User_function();}    //手动程序

int main()     //运行程序开始处
{
  // Run the pre-autonomous function.
  pre_auton();
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
