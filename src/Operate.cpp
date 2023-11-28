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
函数功能： 控制底盘转动
依    赖： 滚筒双电机
输入变量： isMotorReversed:
          
          
返 回 值： 无
\*---------------END------------------*/

bool isMotorReversed = false;
bool starthold = false;


void gua(bool Button_a) {
    cegua.setBrake(vex::brakeType::hold);  // 锁死电机
    
    //  初始锁死
    if(!starthold){
      cegua.spin(forward, 100, pct);
      wait(350, msec);  // 等待0.35秒
      cegua.stop();     // 停止电机
      cegua.setBrake(vex::brakeType::hold);  // 锁死电机
      starthold = true;
      }

    if (Button_a) {
        // 切换电机状态
        isMotorReversed = !isMotorReversed;

        // 根据状态控制电机
        if (isMotorReversed) {
            cegua.spin(reverse, 100, pct);  // 启动电机（反转）
        } else {
            cegua.spin(forward, 100, pct); // 启动电机（正向）
        }

        wait(650, msec);  // 等待0.42秒
        cegua.stop();     // 停止电机
        cegua.setBrake(vex::brakeType::hold);  // 锁死电机
    }

}
