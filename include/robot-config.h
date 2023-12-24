#ifndef _ROBOT_CONFIG_H
#define _ROBOT_CONFIG_H
#include "vex.h"

extern brain  Brain;
extern controller Controller1;
extern motor left1;
extern motor left2;
extern motor left3;

extern motor right1;             
extern motor right2;
extern motor right3;

extern inertial Gyro;
extern motor Rollmotor;
extern motor cegua;
extern motor Climbmotor;
extern motor Shootermotor;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );

extern vex::timer T1;
extern vex::timer T2;
extern vex::timer T3;
extern vex::timer T4;
extern vex::timer TACC;

extern int steps;

#define sleep(a) vex::task::sleep(a)

#endif
