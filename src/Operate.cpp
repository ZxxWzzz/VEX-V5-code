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
void roller(bool Button_a,bool Button_b)
{
  static int Run_ball_flag = 0;
  static int Collect_delay = 0;
  static int Collect_flag = 0;    //0表示收球进程 1表示吐球进程
/*------------------------------滚筒装置------------------------------*/
if(Button_a)   //正转滚筒
{
  Run_ball_flag = !Run_ball_flag;
  Rollmotor.spin(vex::directionType::fwd,100,vex::voltageUnits::volt);    //A 正转 B反转
  wait(200, msec); //程序按键消抖
  Collect_flag = 0;
}
else if(Button_b)   //反转滚筒
{
  Run_ball_flag = !Run_ball_flag;
  Rollmotor.spin(vex::directionType::rev,100,vex::voltageUnits::volt);
  wait(200, msec); //程序按键消抖
  Collect_flag = 1;
}
/*---------------------------------------------------------------------*/

/*---------------------滚筒电机堵转保护程序-----------------------------*/
  if (Rollmotor.velocity(pct) == 0 ) 
  {
    Collect_delay++;
  }
  if (Collect_delay > 60) 
  {
    Rollmotor.stop(vex::brakeType::hold);
    Run_ball_flag = 0;
    Collect_delay = 0;
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
