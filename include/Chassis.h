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
void Chassis_Turn(double Aim_Angle,bool Angle_Type=true,bool Auto_User=true,double Tolerance=1.0f);                
                                                         //底盘旋转'Angle'度,默认为基于当前位置旋转
void Chassis_Forward(double Aim_Distance,double Aim_Angle,bool Auto_User=true,double Tolerance=1.0f);
                                                         //底盘前进'Aim_Distance'单位,默认为基于当前位置旋转



/*--------------------------------END-------------------------------------*/
#endif

