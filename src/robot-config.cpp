#include "robot-config.h"
// A global instance of brain used for printing to the V5 Brain screen



// VEXcode device constructors

/*---------------------------端口配置-------------------------------*/
brain  Brain;
controller Controller1 = controller(primary);
/*--------------------------*\
红色齿轮箱   36:1   ratio36_1 
绿色齿轮箱   18:1   ratio18_1
蓝色齿轮箱    6:1   ratio6_1
\*--------------------------*/
// VEXcode device constructors
motor left1 = motor(PORT2, ratio6_1, true);
motor left2 = motor(PORT3, ratio6_1, false);
motor left3 = motor(PORT4, ratio6_1, true);
motor right1 = motor(PORT9, ratio6_1, false);
motor right2 = motor(PORT8, ratio6_1, true);
motor right3 = motor(PORT7, ratio6_1, false);
inertial Gyro = inertial(PORT6);
motor Rollmotor = motor(PORT5, ratio6_1, true);
motor cegua = motor(PORT21, ratio6_1, true);
motor Climbmotor = motor(PORT1, ratio36_1, false);
motor Shootermotor = motor(PORT1, ratio36_1, false);

/*--------------------------------END-------------------------------------*/

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}