/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * 地址：https://github.com/br3ttb/Arduino-PID-Library/
 * 本PID库修改自Beginner所编写的PID库，我保留了一部分其中的特性，删除了本项目中所用不到的内容。
 **********************************************************************************************/

#include "PID_v1.h"

void PID_Init(PID_ParameterTypeDef * PID){

   PID->FirstCompute =1;

   PID->MaxIntegral = PID->OutMax;
   PID->MinIntegral = PID->OutMin;

   if(PID->ControllerDirection != DIRECT){
      PID->Kp = (0 - PID->Kp);
      PID->Ki = (0 - PID->Ki);
      PID->Kd = (0 - PID->Kd);
      PID->ControllerDirection = REVERSE;
   }
}

/**
 * PID为PID_Parameter结构体的PID参数
 * Feedback为反馈值，也就是当前系统的输出值
 * Reference为参考值，也就是系统期望的输出值
 * PID_OutPut为PID运算的输出结果，用于调整系统输出
 */
inline void PID_Compute(PID_ParameterTypeDef * PID,  float Feedback ,float Reference ,uint32_t *PID_OutPut ){

   float Error = 0;
   Error = Reference - Feedback;
   
   PID->Integral = PID->Integral + PID->Ki * Error;

   //积分限幅
   if(PID->Integral > PID->OutMax) PID->Integral= PID->OutMax;
   else if(PID->Integral < PID->OutMin) PID->Integral= PID->OutMin;

   float OutPut;
   OutPut = PID->Kp * Error;

   OutPut += PID->Integral;

   if(OutPut > PID->OutMax) OutPut = PID->OutMax;
   else if(OutPut < PID->OutMin) OutPut = PID->OutMin;

   *PID_OutPut =(uint32_t)OutPut;
}


//PID预载积分，防止初始积分太大/太小引起初始输出与实际差异过大引起爆炸
//InitialInput为预载积分的一个参数，设置为Vcap，可以使积分设置为Vcap/Vbat*最大输出，即占空比约为Vcap/Vbat*100%
inline void PID_Preload_Integral(PID_ParameterTypeDef * PID ,float InitialInput){
   if(PID->FirstCompute){
      //因为PID最后的输出PID_OutPut = Integral +Ki * Error +Kp * Error，主要取决于Integral数值的大小
      //PID计算的输出值会直接赋值给PWM的比较值，直接作用于占空比
      //当两边电压不确定的时候，integral正常初始化是0，会导致前几次进入PID时，integral需要多次计算累积，PID会一直输出较小的值，导致半桥开关的升降压比与实际电压比不同，就会爆炸
      //所以首次进入PID计算的时候，我们需要初始化他的Integral，使首次PID计算的输出值作用于占空比的时候，让占空比约为输出输入占空比
      //0.04是默认电池电压为25V，乘法以提高速度，有一点误差影响不是很大
      PID->Integral = InitialInput *PID->OutMax *0.04f;
      if(PID->Integral > PID->OutMax) PID->Integral = PID->OutMax;
      else if(PID->Integral < PID->OutMin) PID->Integral = PID->OutMin;
      PID->FirstCompute =0;
   }
}

//清除PID的积分，使其下一次进入的时候进行一次预加载。
inline void PID_Clear_Integral(PID_ParameterTypeDef * PID){
   PID->FirstCompute =1;
   PID->Integral =0;
}

//环路竞争，用于保护CC，CV，CP保护，Buck模式下使用，取小的值作为PWM占空比以达到保护目的
inline void Loop_Competition_Buck(uint32_t Loop1,uint32_t Loop2,uint32_t *OutputCompare){
   if(Loop1 < Loop2)*OutputCompare = Loop1;
   else *OutputCompare = Loop2;
}

//环路竞争，用于保护CC，CV，CP保护，Boost模式下使用，取大的值作为PWM占空比以达到保护目的
inline void Loop_Competition_Boost(uint32_t Loop1,uint32_t Loop2,uint32_t *OutputCompare){
   if(Loop1 > Loop2)*OutputCompare = Loop1;
   else *OutputCompare = Loop2;
}