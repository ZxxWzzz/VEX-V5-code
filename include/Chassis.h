#ifndef _CHASSIS_H
#define _CHASSIS_H

#include "vex.h"

extern double UI_test_data;

/*---------------------------底盘控制程序声明-------------------------------*/
void Chassis_Controller();           //遥控器控制底盘程序，默认开启陀螺仪
void Chassis_Stop(uint8_t stop_type=1);                  //控制底盘停止运动 默认滑行减速
void Chassis_Run(double Speed_L,double Speed_R,uint8_t Speed_mode = 1,uint8_t Stop_mode = 1); 
                                                         //控制底盘运动 默认电压控制速度
void Chassis_Gyro(double run_Speed,double Gyro_target,uint8_t Speed_mode = 1,uint8_t Stop_mode = 1);  
                                                         //控制底盘以'run_Speed'速度沿着'Gyro_target'角度运动
void Gyro_calibrate(bool waitfinish=false);               //陀螺仪矫正程序，默认等待完成
double Chassis_Angle(bool Angle_Type=true);              //底盘角度读取，默认为0~360°
void Chassis_Turn(double Aim_Angle,double Speed_MAX = 100,bool Angle_Type=true,bool Auto_User=true);                
                                                         //底盘旋转'Angle'度,默认为基于当前位置旋转
void Chassis_Forward(double Aim_Distance,double Aim_Angle,bool Auto_User=true,double Speed_MAX = 100);
                                                         //底盘前进'Aim_Distance'单位,默认为基于当前位置旋转
void Chassis_DriveToAngle(double targetAngle, double maxSpeedL,double maxSpeedR);
                                                          //自动阶段电机运动角度方法
void RunpidStraightNTo(double speed_limit, int aim,double err_1,double speed_limit2, int dec_point, int change_steps,int start_point,int outtime, double newgyro, int p_point);//新自动程序——直线
/*---------------参数声明--------------*\
double speed_limit:这是机器人移动的最大速度限制。它定义了机器人在直线行驶时可以达到的最高速度。

int aim:这是机器人需要达到的目标位置。通常，这个值代表机器人旋转轮子的目标旋转度数，用于确定机器人应该行进的距离。

double err_1:这是允许的误差阈值。当机器人的当前位置与目标位置的差值小于或等于这个值时，可以认为机器人已经达到了目标位置。

double speed_limit2:这是第二阶段的速度限制。当机器人接近目标位置时，可能需要减速以更精确地停在目标点。这个参数定义了减速阶段的速度限制。

int dec_point:这是减速点。当机器人行进的距离达到这个点时，它将开始减速，使用speed_limit2作为新的速度限制。

int change_steps:这个参数用于定义在特定点改变行为的步骤编号。它可以用于在特定的行进阶段改变机器人的行为或启动不同的任务。

int start_point:这是开始执行change_steps中定义的行为的起始点。

int outtime:这是操作的超时时间。如果机器人在这段时间内未能达到目标，操作将终止。这可以防止机器人在遇到问题时无休止地尝试达到目标。

double newgyro:这是目标陀螺仪角度。在直线运动中，它通常用于保持机器人的方向稳定，确保它直线行驶。

int p_point:这个参数用于选择不同的PID控制参数配置。在不同的行驶阶段或条件下，可以通过改变PID参数来优化机器人的行为。
\*----------------END------------------*/

void RightVol(int vol_input);
void LeftVol(int vol_input);
void TurnVol(int turnpct);
void TurnpidNTo(int max_speed, double aim, double howerr, int outtime); //新转向
        //参数：最大限速、目标角度、允许误差、限制时间
/*--------------------------------END-------------------------------------*/
#endif

