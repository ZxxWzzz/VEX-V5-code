#ifndef _AUTO_CONTROL_H
#define _AUTO_CONTROL_H


void Auto_function_far(void);      //远端自动程序
void Auto_function_near(void);  //近端自动程序

void Auto_test(void);  //测试用自动程序


bool angleDetection(double targetAngle); //角度校准函数



#endif
