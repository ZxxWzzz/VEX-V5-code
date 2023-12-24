/*------------------------头文件引用部分----------------------------*/
#include "Chassis.h"
#include "math_function.h"
#include "robot-config.h"
#include "Task.h"
#include "vex.h"
#include "iostream"

vex::timer T1;
vex::timer T2;
vex::timer T3;
vex::timer T4;
vex::timer TACC;

int steps;


/*------------------------工程相关信息------------------------------*\
作    者：翁老师
文 件 名：底盘控制程序
适用机型：全部
版    本：V1.0
详细内容：本文件用于机器人底盘控制的函数编写
所用组件：底盘电机、陀螺仪 
更新内容：首次编写
=====================================================================
\*-----------------------------END---------------------------------*/

double UI_test_data = 0;

/*-----------遥控控制底盘--------------*\
函数功能：使用遥控手柄控制底盘运动,内附陀螺仪控制直线定向巡航
依    赖：陀螺仪、电机、遥控手柄
输入变量： 
返 回 值： 无
\*---------------END------------------*/

//新加入PID算法，在下方底盘控制程序中应用该方法
double PID_Controller(double target, double current) {
  // PID控制算法
  double Kp = 0.8; // 比例系数
  double Ki = 0.0000; // 积分系数
  double Kd = 0.; // 微分系数
  static double prevError = 0;
  static double integral = 0;
  
  double error = target - current;
  integral += error;
  double derivative = error - prevError;
  
  double output = Kp * error + Ki * integral + Kd * derivative;
  
  prevError = error;
  
  return output;
}


double Gyro_Chassis = 0.0f;
bool ch3_change = false;                                  
void Chassis_Controller() {
  static bool Gyro_state = false;  // 陀螺仪开启
  static int8_t Chassis_Dir = 1;   // 改方向
  double ch3value = Chassis_Dir * Controller1.Axis3.position(percent); // 左摇杆
  double ch4value = Controller1.Axis1.position(percent); // 右摇杆

  /*------一键切换车头与车尾的方向-------*/
  // if(Controller1.ButtonDown.pressing()) {Chassis_Dir = -Chassis_Dir; wait(300,msec);}     // 用于车体前后反向

  // 防止摇杆死区
  if (fabs(ch3value) < 5) { ch3value = 0; }
  if (fabs(ch4value) < 5) { ch4value = 0; }

  /*----前进遥控非线性部分------*/
  if (ch3value > 0)
    ch3value = (exp(ch3value / 50)) / 7.389 * 100;
  else if (ch3value < 0)
    ch3value = -(exp(-ch3value / 50)) / 7.389 * 100;
  /*----旋转遥控非线性部分------*/
  if (ch4value > 0)
    ch4value = (exp(ch4value / 50)) / 7.389 * 100;
  else if (ch4value < 0)
    ch4value = -(exp(-ch4value / 50)) / 7.389 * 100;

  // 映射到电机速度范围
  double leftSpeed = 0;
  double rightSpeed = 0;
  // double Speed_MAX = 12;
  //------------------遥控器摇杆值映射至左右电机速度--------------------
  if (ch3value != 0) // 该情况下小车处于前进状态
  {
    if (ch4value >= 0) {
      leftSpeed = (ch3value + ch4value * 0.6) * 12 / 100.0;
      rightSpeed = (ch3value - ch4value * 0.9) * 12 / 100.0;
    }
    if (ch4value < 0) {
      leftSpeed = (ch3value + ch4value * 0.9) * 12 / 100.0;
      rightSpeed = (ch3value - ch4value * 0.6) * 12 / 100.0;
    }
  } else // 该情况下小车处于原地旋转
  {
    leftSpeed = ch4value * 11 / 100.0;
    rightSpeed = -ch4value * 11 / 100.0;
  }

  //-----------------------无陀螺仪定向控制电机运动--------------------------
  if (Gyro_state == false) {
    Chassis_Run(leftSpeed, rightSpeed);
    ch3_change = false;
  }

  //-----------------------陀螺仪定向控制电机运动--------------------------
  if (Gyro_state == true) {
    if (ch4value == 0 && ch3value == 0)
      Chassis_Stop(1);
    else if (ch4value == 0) // 仅直线行驶时锁定航向
    {
      if (ch3_change == false) {
        Gyro_Chassis = Gyro.rotation(); // 更新一次角度,获取当前车体角度信息
        ch3_change = true;            // 更新标志位，第一次进入直线行驶PID程序,表示定向巡航角度已经更新
      }
      // 使用PID控制来调整左右电机速度
      double leftSpeedPID = PID_Controller(leftSpeed, Gyro_Chassis); // 使用PID算法
      double rightSpeedPID = PID_Controller(rightSpeed, Gyro_Chassis); // 使用PID算法
      Chassis_Run(leftSpeedPID, rightSpeedPID);
    } else {
      Chassis_Run(leftSpeed, rightSpeed);
      ch3_change = false; // 更改首次直线行驶标志位
    }
  }
}

/*===========================================================================*/


/*-----------底盘停止运动-------------*\
函数功能： 控制底盘停止运动
依    赖： 底盘四个电机
输入变量： 无符号八位整型 stop_type, 
          1->coast(滑行刹车) 2->brake(电磁刹车) 3->hold(锁死刹车) 
返 回 值： 无
\*---------------END------------------*/
void Chassis_Stop(uint8_t stop_type)
{
  if(stop_type == 1)      {  right1.stop(coast); right2.stop(coast); right3.stop(coast); left1.stop(coast);  left2.stop(coast); left3.stop(coast);}
  else if(stop_type == 2) {  right1.stop(brake); right2.stop(brake); right3.stop(brake); left1.stop(brake);  left2.stop(brake); left3.stop(brake);}
  else if(stop_type == 3) {  right1.stop(hold);  right2.stop(hold); right3.stop(hold); left1.stop(hold);   left2.stop(hold);  left3.stop(hold);}
}
/*===========================================================================*/


/*-----------底盘运动控制-------------*\
函数功能： 控制底盘运动
依    赖： 底盘四个电机
输入变量： Speed_L : 左侧电机速度  Speed_R : 右侧电机速度
          Speed_mode:速度控制模式 1->电压控制模式(默认)  2->速度百分比控制模式
          Stop_mode:速度为零时的刹车模式:
            1->滑行刹车(默认)  2->电磁刹车  3->抱死刹车
返 回 值： 无
\*---------------END------------------*/
void Chassis_Run(double Speed_L,double Speed_R,uint8_t Speed_mode,uint8_t Stop_mode)
{
  if(Speed_L != 0 )
  {
    if(Speed_mode == 1)       { left1.spin(vex::directionType::fwd,Speed_L,vex::voltageUnits::volt);  left2.spin(vex::directionType::fwd,Speed_L,vex::voltageUnits::volt); left3.spin(vex::directionType::fwd,Speed_L,vex::voltageUnits::volt);}
    else if(Speed_mode == 2)  { left1.spin(vex::directionType::fwd,Speed_L,vex::velocityUnits::pct);  left2.spin(vex::directionType::fwd,Speed_L,vex::velocityUnits::pct); left3.spin(vex::directionType::fwd,Speed_L,vex::velocityUnits::pct);}
  }
  else
  {
    if(Stop_mode == 1)        { left1.stop(coast); left2.stop(coast); left3.stop(coast);}
    else if(Stop_mode == 2)   { left1.stop(brake); left2.stop(brake); left3.stop(brake);}
    else if(Stop_mode == 3)   { left1.stop(hold);  left2.stop(hold);  left3.stop(hold); }
  }

  if(Speed_R != 0 )
  {
    if(Speed_mode == 1)       { right1.spin(vex::directionType::fwd,Speed_R,vex::voltageUnits::volt);  right2.spin(vex::directionType::fwd,Speed_R,vex::voltageUnits::volt); right3.spin(vex::directionType::fwd,Speed_R,vex::voltageUnits::volt); }
    else if(Speed_mode == 2)  { right1.spin(vex::directionType::fwd,Speed_R,vex::velocityUnits::pct);  right2.spin(vex::directionType::fwd,Speed_R,vex::velocityUnits::pct); right3.spin(vex::directionType::fwd,Speed_R,vex::velocityUnits::pct); }
  }
  else
  {
    if(Stop_mode == 1)        { right1.stop(coast); right2.stop(coast); right3.stop(coast);}
    else if(Stop_mode == 2)   { right1.stop(brake); right2.stop(brake); right3.stop(brake);}
    else if(Stop_mode == 3)   { right1.stop(hold);  right2.stop(hold);  right3.stop(hold);}
  }
}
/*===========================================================================*/

/*-----------定向运行控制-------------*\
函数功能： 定向运行控制，需放入循环中持续运行
依    赖： 底盘电机、陀螺仪
输入变量： run_Speed : 运动速度  Gyro_target : 运动的角度
          Speed_mode:速度控制模式 1->电压控制模式(默认)  2->速度百分比控制模式
          Stop_mode:速度为零时的刹车模式:
            1->滑行刹车(默认)  2->电磁刹车  3->抱死刹车
返 回 值： 无
\*---------------END------------------*/
void Chassis_Gyro(double run_Speed,double Gyro_target,uint8_t Speed_mode,uint8_t Stop_mode)
{
  double Gyro_output = 0.0f;
  static double Gyro_error_n = 0;         //本次误差
  static double Gyro_error_L = 0;         //上次误差

  Gyro_error_n = (Gyro_target - Gyro.rotation());  //本次误差更新
  //如果误差小于1°则不进行修这个
  if(abs(Gyro_error_n) > 0.2f) Gyro_output = (Gyro_error_n-Gyro_error_L)*5.0f + Gyro_error_n*0.655f; //PD计算

  //限位
  if(Gyro_output>1.5f) Gyro_output=1.5f;
  else if(Gyro_output<-1.5f) Gyro_output=-1.5f;

  // 输出电机控制
  Chassis_Run(run_Speed+Gyro_output,run_Speed-Gyro_output,Speed_mode,Stop_mode);
  Gyro_error_L = Gyro_error_n;
}
/*===========================================================================*/


/*-----------陀螺仪矫正程序-------------*\
函数功能： 陀螺仪矫正程序，并在控制器上显示正在矫正
依    赖： 陀螺仪、控制器显示屏
输入变量： waitfinish:是否等待完成矫正(默认等待完成矫正)
返 回 值： 无
\*---------------END------------------*/
void Gyro_calibrate(bool waitfinish)
{
  if(Gyro.installed() == false)  //判断是否正确配置陀螺仪
  {
    Controller1.Screen.clearLine();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Not found Gyro!");
    while(1) 
    { 
      wait(100, msec); 
      if(Gyro.installed() == true) 
      {
        break;                           //如再次成功检测则离开
        Controller1.Screen.clearLine();  //清楚错误信息   
      }
    }
  }

  Gyro.calibrate();                      //开始矫正陀螺仪

  if(waitfinish == true)                 //如果需要等待陀螺仪矫正完成则进入
  {
    Controller1.Screen.clearLine();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Calibrating");
    while(Gyro.isCalibrating()) wait(10, msec);  //正在进行返回1  完成返回0
    Controller1.Screen.clearLine();              //清楚错误信息 
  }  
}
/*===========================================================================*/


/*---------------陀螺仪角度读取-------------*\
函数功能： 返回当前陀螺仪角度
依    赖： 陀螺仪
输入变量： Angle_Type:返回角度类型 
          false -> 0 - ∞°
          true  -> 0 - 360°
返 回 值： 底盘角度
\*-------------------END------------------*/
double Chassis_Angle(bool Angle_Type)
{
  if(Angle_Type == false)  return Gyro.rotation(vex::rotationUnits::deg);                 //返回增量式坐标系
  else                     return fmod(Gyro.rotation(vex::rotationUnits::deg),360.0);     //返回位置式坐标系
}
/*===========================================================================*/


/*---------------底盘旋转程序-----------------*\
函数功能： 控制车体旋转角度
依    赖： 陀螺仪、底盘电机
输入变量： Aim_Angle:目标角度
          Angle_Type:角度的形式，1->基于当前角度旋转'Angle' (默认)
                                0->基于比赛开始时的角度旋转到'Angle'
          Auto_User:自动/手动    true  -> 自动中使用(默认)
                                false -> 手动中使用
          Tolerance:可以容忍的误差大小,默认1°
返 回 值： 无
\*---------------END------------------*/
void Chassis_Turn(double Aim_Angle, double Speed_MAX,bool Angle_Type, bool Auto_User) {
  Gyro.resetRotation(); //陀螺仪重置为0

  double Kp = 0.65;  // 比例系数
  double Ki = 0.00009;  // 积分系数，根据需要调整
  double Kd = 0.1;   // 微分系数，根据需要调整

  double Toleranc = 2.0;//允许误差

  double Angle_now = Chassis_Angle(0);
  double Turn_Output = 0.0;
  double Turn_err_now = 0.0;
  double Turn_err_integral = 0.0;
  double Turn_err_derivative = 0.0;
  double prev_err = 0.0;

  uint8_t success_times = 0;

  if (Auto_User == false) {
    ch3_change = false;
    Task1_Chassis.suspend();
  }  // 判断为手动程序，暂时关闭摇杆任务

  if (Angle_Type == false)
    Aim_Angle = Aim_Angle + 360 * (int)(Angle_now / 360);  // 将输入变量转化为增量式角度即目标角度
  else
    Aim_Angle = Aim_Angle + Angle_now;  // 将输入变量转化为目标角度

  if (abs(Aim_Angle - Angle_now) > 180)
    Aim_Angle = Aim_Angle - 360.0f;  // 最短角度旋转

  while (1) {
    Angle_now = Chassis_Angle(0);
    Turn_err_now = Aim_Angle - Angle_now;

    if (abs(Turn_err_now) < Toleranc) {
      success_times++;        // 瞬时完成
      Chassis_Stop(3);        // 电磁刹车 提供阻尼
      if (success_times >= 10) {
        UI_test_data = Angle_now;  //----------------测试用
        break;                     // 完成稳定超过100ms，且误差小于0.5°时不再旋转，跳出循环
      }
    } else {
      success_times = 0;
      Turn_err_integral += Turn_err_now;
      Turn_err_derivative = Turn_err_now - prev_err;

      // 计算 PID 输出
      Turn_Output = Kp * Turn_err_now + Ki * Turn_err_integral + Kd * Turn_err_derivative;

      Turn_Output = Limitdata(Turn_Output, Speed_MAX, -Speed_MAX);  //自动限速

      // 控制输出范围
      if (Turn_Output > 0) {
        Turn_Output = fabs(Turn_Output);
        Chassis_Run(Turn_Output, -Turn_Output, 2, 3);  // 输出控制电机
      } else {
        Turn_Output = fabs(Turn_Output);
        Chassis_Run(-Turn_Output, Turn_Output, 2, 3);  // 输出控制电机
      }

      prev_err = Turn_err_now;
      if(prev_err<=1){prev_err = 0;} //限制
    }

    if (Controller1.Axis3.position(percent) >= 50 || Controller1.Axis3.position(percent) <= -50)
      break;  // 前进摇杆百分比>50%, 可强行退出旋转

    wait(1, msec);  // 控制频率
  }

  Chassis_Stop(3);  // 刹车模式3 抱死刹车

  if (Auto_User == false) Task1_Chassis.resume();
}
/*===========================================================================*/


/*---------------底盘直线运行程序-----------------*\
函数功能： 控制车体直线运行，电机减速比 84:36(仅该减速比下可用mm距离控制)
依    赖： 陀螺仪、底盘电机
输入变量： Aim_Distance:目标距离
          Distance_Type:距离的形式
                                
          Auto_User:自动/手动    true  -> 自动中使用(默认)
                                false -> 手动中使用
          Tolerance:可以容忍的误差大小,默认1.0°
          Speed_MAX:移动限速
          当前车的换算比为    2.637°/mm
返 回 值： 无 
\*---------------END------------------*/
void Chassis_Forward(double Aim_Distance, double Aim_Angle, bool Auto_User,  double Speed_MAX) {
    double Kp = 0.07;  // 比例系数
    double Ki = 0.000006;  // 积分系数
    double Kd = 0.04;  // 微分系数
    double Tolerance = 4; //允许误差
    double Forward_now = 0;
    double Forward_err_now = 0;
    double Forward_err_integral = 0;
    double Forward_err_derivative = 0;
    double prev_err = 0;

    Aim_Distance = Aim_Distance * 2.637f;  // 单位换算
    Tolerance = Tolerance * 4.0f;          // 单位换算

    left1.resetRotation();  // 复位左电机编码器

    while (1) {
        Forward_now = left1.rotation(deg);
        Forward_err_now = Aim_Distance - Forward_now;  // 误差一 目标区域误差

        if (abs(Forward_err_now) < Tolerance) {  // 任务完成判定部分
            Chassis_Stop(3);                    // 电磁刹车 提供阻尼
            UI_test_data = Forward_err_now;     // 测试用
            break;                              // 完成稳定超过10*2ms，且误差小于0.5°时不再旋转，跳出循环
        } else {
            Forward_err_integral += Forward_err_now;
            Forward_err_derivative = Forward_err_now - prev_err;

            // 计算 PID 输出
            double Forward_Output = Kp * Forward_err_now + Ki * Forward_err_integral + Kd * Forward_err_derivative;

            // 输出值限幅
            Forward_Output = Limitdata(Forward_Output, Speed_MAX, -Speed_MAX);  //自动限速

            // 使用PID算法控制前进速度
            Chassis_Gyro(Forward_Output, Aim_Angle, 2, 3);
        }

        UI_test_data = Forward_err_now;  // 测试用，用于控制器上误差显示
        wait(1, msec);

        prev_err = Forward_err_now;      
        if(prev_err<40){prev_err = 0;} //限制

    }

    Chassis_Stop(3);  // 刹车模式3 抱死刹车
}
/*===========================================================================*/

/*-----自动阶段控制电机移动指定角度------*\
函数功能： 自动阶段控制电机移动指定角度
依    赖： 陀螺仪、底盘电机
输入变量： targetAngle：目标角度
          speedLeft：左侧电机速度
          speedRight：右侧电机速度
返 回 值： 无 
\*---------------END------------------*/

// Gyro.resetRotation();

void Chassis_DriveToAngle(double targetAngle, double maxSpeedL,double maxSpeedR) {
    const double tolerance = 3.5; // 允许误差范围
    double Kp = 0.5; // 比例系数
    double Ki = 0.0; // 积分系数
    double Kd = 0.45; // 微分系数

    double integral = 0.0;
    double previous_error = 0.0;

    Gyro.resetRotation(); // 陀螺仪重置为0

    // 循环直到达到目标角度
    while(true) {
        double currentAngle = Chassis_Angle(0); // 读取当前角度
        double error = targetAngle - currentAngle;
        integral += error;
        double derivative = error - previous_error;

        // 计算PID输出
        double output = Kp * error + Ki * integral + Kd * derivative;
        previous_error = error;

        // 显示当前角度到控制器屏幕
        Controller1.Screen.clearLine(1); // 清除之前的输出
        Controller1.Screen.print("Auto Angle: %.2f", currentAngle);

        // 检查是否在目标角度的容差范围内
        if(fabs(error) <= tolerance) {
            Chassis_Stop(); // 停止底盘
            break; // 退出循环
        } else {
            // 根据PID输出调整左右轮速度
            double speedLeft = maxSpeedL * sgn(output);
            double speedRight = -maxSpeedR * sgn(output);
            Chassis_Run(speedLeft, speedRight); // 控制底盘运动
        }
        wait(1, msec); // 稍微延迟以防止CPU过载
    }
    Chassis_Stop(3);
}


/*-----以下为参考林老师新的程序------*\
\*---------------END---------------*/


void RunpidStraightNTo(double speed_limit, int aim, double err_1, double speed_limit2, 
                       int dec_point, int outtime, double newgyro) {

    double Kp = 0.26, Ki = 0.00, Kd = 0.23;
    
    double value_now = 0, EI = 0, ED = 0, err_now = 0, err_last = 0;
    double max_v = speed_limit, Kt = 0, Ktv = 0, ET = 0, ETV = 0, sum_dec = 0;
    double value_now_L = 0, value_last_L = 0, value_now_R = 0, value_last_R = 0;
    double outputL, outputR, angle_err = 0;
    int sampletime = 10;
    double acc = 0.2;
    double K_gyro = 0.7;

    left1.resetPosition();
    right1.resetPosition();
    T1.clear(); T2.clear(); T3.clear(); T4.clear(); TACC.clear();

    steps = 0; 

    while (true) {
        printf("%.2f\n", left1.velocity(pct));
        double returnangle = Gyro.rotation(degrees);

        max_v = acc * TACC.time();
        if (TACC.time() > 500) max_v = speed_limit;
        if (max_v > speed_limit) max_v = speed_limit;

        if (dec_point != -1 && fabs(left1.position(vex::rotationUnits::deg)) > dec_point) {
            Brain.Screen.drawRectangle(1, 1, 400, 400, vex::color::green);//显示绿色进入第二进程
            max_v = speed_limit - (fabs(left1.position(vex::rotationUnits::deg)) - dec_point) / 50.0;
            if (max_v < speed_limit2) max_v = speed_limit2;
        }

        value_now = left1.position(vex::rotationUnits::deg);
        if (T2.time() > 100) {
            T2.clear();
            value_last_R = value_now_R;
            value_last_L = value_now_L;
            value_now_R = right1.position(vex::rotationUnits::deg);
            value_now_L = left1.position(vex::rotationUnits::deg);
            ETV = (value_now_R - value_last_R) - (value_now_L - value_last_L);
            ET = value_now_R - value_now_L;
            sum_dec += Ktv * ETV;
        }

        if (T3.time() > sampletime) {
            T3.clear();
            err_last = err_now;
            err_now = aim - value_now;
            ED = err_now - err_last;
            if (fabs(err_now) > 100) EI = 0;
            else EI += err_now;
            outputL = Kp * err_now + Ki * EI + Kd * ED;
            if (fabs(outputL) > max_v) outputL = sgn(outputL) * max_v;

            angle_err = newgyro - returnangle;
            if (fabs(angle_err) < 1) angle_err = 0;
            outputR = outputL - Kt * ET - Ktv * ETV - K_gyro * angle_err;

            Chassis_Run(outputL, outputR);
            sleep(sampletime);
            if (fabs(err_now) > err_1) T1.clear();
            if (T1.time() > 25 || T4.time() >= outtime) {
                Chassis_Stop();
                break;
            }
        }
    }
    Chassis_Stop();
}


void RightVol(int vol_input) {
  right1.spin(fwd, 0.128*vol_input, voltageUnits::volt);
  right2.spin(fwd, 0.128*vol_input, voltageUnits::volt);
  right3.spin(fwd, 0.128*vol_input, voltageUnits::volt);
}


void LeftVol(int vol_input) {
  left1.spin(fwd, 0.128*vol_input, voltageUnits::volt);
  left2.spin(fwd, 0.128*vol_input, voltageUnits::volt);
  left3.spin(fwd, 0.128*vol_input, voltageUnits::volt);
}
void TurnVol(int turnpct) {
    LeftVol(turnpct);
    RightVol(-turnpct);
}

//新转向
void TurnpidNTo(int max_speed, double aim, double howerr, int outtime) {
    double Kp = 0.75, Ki = 0, Kd = 0.15;
    
    double err_now = 0, err_last = 0, value_now = 0, EI = 0, ED = 0, output = 0;
    int sampletime = 10;

    T1.clear();
    T4.clear();

    value_now = Gyro.rotation(degrees);
    err_now = aim - value_now;

    Brain.Screen.drawRectangle(1, 1, 400, 400, vex::color::red);

    while (true) {
        value_now = Gyro.rotation(degrees);
        err_now = aim - value_now;
        EI += err_now;
        if (fabs(err_now) > 15) EI = 0;
        ED = err_now - err_last;

        output = Kp * err_now + Ki * EI + Kd * ED;
        if (fabs(output) > max_speed) output = sgn(output) * max_speed;

        TurnVol(output);

        err_last = err_now;

        sleep(sampletime);

        if (fabs(err_now) > howerr) T1.clear();
        if (T1.time() > 25 || T4.time() >= outtime) {
            Chassis_Stop(2);
            break;
        }
    }

    Brain.Screen.drawRectangle(1, 1, 400, 400, vex::color::blue);
}
