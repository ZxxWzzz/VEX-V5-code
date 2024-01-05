/*------------------------头文件引用部分----------------------------*/
#include "Operate.h"
#include "robot-config.h"
/*------------------------工程相关信息------------------------------*\
作    者：翁老师
文 件 名：操作机构控制程序
适用机型：全部
版    本：V1.0
详细内容：本文件用于机器人操作机构部分的函数编写
所用组件： 
更新内容：首次编写
=====================================================================
\*-----------------------------END---------------------------------*/


/*-----------爬升控制-------------*\
函数功能： 控制底盘运动
依    赖： 
输入变量：   
返 回 值： 无
\*---------------END------------------*/
void climber(bool Button_a,bool Button_b)
{
  if(Button_a&&!Button_b) Climbmotor.spin(forward,50,pct);
  else if(Button_b&&!Button_a) Climbmotor.spin(forward,-50,pct);
  else Climbmotor.stop(hold);
}

/*-----------滚筒控制-------------*\
函数功能： 控制底盘转动
依    赖： 滚筒双电机
输入变量： a:
返 回 值： 无
\*---------------END------------------*/
void roller(bool Button_a, bool Button_b)
{
  static bool isCollecting = false; // 标志表示是否正在吸球
  static bool isEjecting = false;   // 标志表示是否正在吐球

  // 吸球控制
  if (Button_a) {
    if (!isCollecting) { // 如果当前没有在吸球，则开始吸球
      isCollecting = true;
      isEjecting = false; // 停止吐球
    } else {
      isCollecting = false; // 如果已经在吸球，则停止
    }
    wait(150, msec); // 按键消抖
  }

  // 吐球控制
  if (Button_b) {
    if (!isEjecting) { // 如果当前没有在吐球，则开始吐球
      isEjecting = true;
      isCollecting = false; // 停止吸球
    } else {
      isEjecting = false; // 如果已经在吐球，则停止
    }
    wait(150, msec); // 按键消抖
  }

  // 根据当前状态控制电机
  if (isCollecting) {
    Rollmotor.spin(vex::directionType::fwd, 100, vex::voltageUnits::volt);
  } else if (isEjecting) {
    Rollmotor.spin(vex::directionType::rev, 100, vex::voltageUnits::volt);
  } else {
    Rollmotor.stop(vex::brakeType::coast);
  }
}

/*-----------滚筒控制1-------------*\
函数功能： 控制底盘转动
依    赖： 滚筒双电机
输入变量： a:   
返 回 值： 无
\*---------------END------------------*/
void roller1(bool Button_a, bool Button_b)
{
  if (Button_a) {
    Rollmotor.spin(vex::directionType::fwd, 100, vex::voltageUnits::volt);
  } 
  if (Button_b) {
    Rollmotor.spin(vex::directionType::rev, 100, vex::voltageUnits::volt);
  }
}

/*-------------弹射控制---------------*\
函数功能： 击球结构发射装置
依    赖： 击球结构的电机
输入变量： Button 1：持续击打
                 0：停止
返 回 值： 无
\*---------------END------------------*/
void shooter(bool Button)
{
  if(Button) {
    Shootermotor.spin(reverse,100,pct);
    }
  else if (Button == 0){
    Shootermotor.stop(vex::brakeType::hold);
    }
}

/*-------------单击弹射控制-------------*\
函数功能： 控制底盘运动
依    赖： 发射装置电机
输入变量： 
          
返 回 值： 无
\*---------------END------------------*/
void shooterClick(bool Button) 
{
  static bool shootingState = false;
  if(Button) shootingState = !shootingState;
}

/*-----------侧挂控制-------------*\
函数功能： 控制电磁阀
依    赖： 气动电磁阀
输入变量： is_state
返 回 值： 无
\*---------------END--------------*/

int is_state = -1;

void Pump(bool Button_a) {

    if(Button_a){
      if(is_state == 1){
        Pumper.set(true);
      }else {
        Pumper.set(false);
      }
      is_state *= -1;
      sleep(250);
    }

}

/*-----------自动用侧挂控制-------------*\
函数功能： 控制电磁阀
依    赖： 气泵电磁阀
输入变量： 打开时间
返 回 值： 无
\*---------------END-------------------*/
void Auto_Pump(bool a){
  if(a) {Pumper.set(true);}
  else {Pumper.set(false);}
}

/*-----------高挂自动回收----------*\
函数功能： 控制高挂电机回收
依    赖： 高挂电机
输入变量： 无
返 回 值： 无
备    注：以进入程序时角度为基础，期望
         角度未经过换算需实际测试
\*---------------END--------------*/

void climberbake(bool Button_d){
  if(Button_d){
      Climbmotor.spinToPosition(0,degrees,100,velocityUnits::pct);
  }
}