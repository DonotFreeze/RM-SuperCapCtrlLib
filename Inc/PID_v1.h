#ifndef PID_V1_h
#define PID_V1_h


#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"


#define DIRECT  0
#define REVERSE  1

typedef struct{
  float Kp,Ki,Kd;//三个系数
  uint8_t ControllerDirection;
  uint8_t FirstCompute;

  float Integral;
  float LastFeedback;

  float MinIntegral,MaxIntegral;//积分限幅
  uint32_t OutMin,OutMax;//输出限幅
}PID_ParameterTypeDef;

void PID_Init(PID_ParameterTypeDef * PID); 

void PID_Compute(PID_ParameterTypeDef * PID,  float Feedback ,float Reference ,uint32_t *PID_OutPut );

void PID_Preload_Integral(PID_ParameterTypeDef * PID ,float InitialInput);

void PID_Clear_Integral(PID_ParameterTypeDef * PID);

void Loop_Competition_Buck(uint32_t Loop1,uint32_t Loop2,uint32_t *OutputCompare);

void Loop_Competition_Boost(uint32_t Loop1,uint32_t Loop2,uint32_t *OutputCompare);

#ifdef __cplusplus
}
#endif

#endif

