/*------------------------头文件引用部分----------------------------*/
#include "math.h" 
/*------------------------工程相关信息------------------------------*\
作    者：翁老师
文 件 名：基本数学库，用于基本的数学运算
适用机型：全部
版    本：V1.0
详细内容：

更新内容：首次编写
=====================================================================
\*-----------------------------END---------------------------------*/


/*-----------绝对值函数-------------*\
函数功能： 将输入的值转换为绝对值后输出
依    赖： 无
输入变量： v 需要转换为绝对值的数值
          0->hold(锁死刹车) 1->coast(滑行刹车) 2->brake(电磁刹车)
返 回 值： 输入变量的绝对值
\*---------------END------------------*/
double abs(double v)
{
  if(v<0) return -v;
  return v;
}
/*===========================================================================*/


/*-----------符号函数-------------*\
函数功能： 将输入的值转换为符号变量，即输入正数返回1，
          输入负数返回-1
依    赖： 无
输入变量： v 需要转换为符号的数值
返 回 值： 输入变量的符号值
\*---------------END------------------*/
double sgn(double v)
{
    if(v<0)  return -1.0;
    else     return  1.0;
}
/*===========================================================================*/


/*-----------限幅函数-------------*\
函数功能： 将输入的值做限幅处理
依    赖： 无
输入变量： v 需要限幅的数值
          limia 最大值 limib最小值
返 回 值： 输入变量的符号值
\*---------------END------------------*/
double Limitdata(double v,double limia,double limib)
{
    if(v>limia)          return limia;      //超出最大值,返回最大值
    else if(v<limib)     return limib;      //超出最小值,返回最小值
    else                 return v;          //未超出限幅,返回原始值
}
/*===========================================================================*/

/*-----------非线性函数-------------*\
函数功能： 非线性函数
依    赖： 无
输入变量： v 为自变量(0-100)
返 回 值： 输出为非线性的(0-100)
\*---------------END------------------*/
double nonlinear(double v)
{
  return (exp(v/50)-1)/6.389*100;
}
/*===========================================================================*/




