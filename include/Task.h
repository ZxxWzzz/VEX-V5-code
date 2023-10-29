#ifndef _TASK_H
#define _TASK_H
#include "vex.h"

extern task Task1_Chassis;
extern task Task2_UI_Brain;
extern task Task3_UI_Controller;
extern task Task4_STOP_f;


int Task1_Chassis_fun();            //底盘控制任务的回调函数
int Task2_UI_fun();                 //机器人主脑UI显示函数的回调函数
int Task3_UI_fun();                 //遥控手柄UI显示函数的回调函数
int Task_STOP();
void Task_state(bool task_state);   //多任务启停函数
#endif