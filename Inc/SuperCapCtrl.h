#ifndef SUPERCAPCTRL_H
#define SUPERCAPCTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/

/**
  * 滞回比较器状态结构体
  */
typedef enum
{
  FALLING = 0,
  RISING = 1
} ComparatorStateTypeDef;

typedef enum
{
  DISCHARGE = 0,
  CHARGE = 1,
  WAIT = 2,
  SOFRSTART_PROTECTION = 3,
  OVER_LOAD_PROTECTTION = 4,
  OVP_BAT_PROTECTION = 5,
  UVP_BAT_PROTECTION = 6,
  UVP_CAP_PROTECTION = 7,
  OTP_PROTECTION = 8,
  BOOM_PROTECTION = 9,
  CAN_OFFLINE = 10,
} SuperCapStateTypeDef;

typedef enum
{
  UNREADY = 0,
  READY = 1,
} SuperCapReadyTypeDef;


//这是记录超级电容控制板运行状态的标志位
//本来是打算使用位域进行定义的，但是后面调试的时候发现位域的变量不会显示变量名，所以用了uint8，Flash管够
typedef struct{
  uint8_t SoftStart_bit ; //超级电容软起动
  uint8_t Charge_bit ; //超级电容充电。1充电，0放电
  uint8_t Enable_bit ;  //超级电容给使能。1使能，0失能

  uint8_t CAN_Offline_bit;  //控制板CAN离线检测。1离线，0在线
  uint8_t UVP_Bat_bit ; //电池欠压保护。1欠压，0正常
  uint8_t UVP_Cap_bit ; //电容组过放保护。1过放，0正常
  uint8_t OTP_MOS_bit ; //控制板温度保护。1过温，0正常
  uint8_t OTP_CAP_bit ; //电容组过温保护。1过温，0正常
  uint8_t OCP_bit ;     //控制板过流保护。1过流，0正常
  uint8_t OVP_Cap_bit ; //电容组过压保护。1过压，0正常
  uint8_t OVP_Bat_bit ; //母线过压保护。1过压，0正常
  uint8_t BOOM_bit ; //保险丝熔断标志。1熔断，0正常
} StateFlagsTypeDef;

typedef struct{
  float RisingThreshold;
  float FallingThreshold;
  uint32_t Timeout;
  uint32_t Counter;
  uint8_t IsInit;
  ComparatorStateTypeDef SchmittTriggerState;
  ComparatorStateTypeDef SchmittTriggerDirection;
} AnomalyDetectionTypeDef;


//发送给电控的数据
typedef struct {
  uint8_t SuperCapEnergy;//超级电容可用能量：0-100%
  uint8_t ChassisPower; //底盘功率，0-512，由于传输的时候为了扩大量程右移了一位，所以接收的时候需要左移还原（丢精度）。
  SuperCapReadyTypeDef SuperCapReady;//超级电容【可用标志】：1为可用，0为不可用
  SuperCapStateTypeDef SuperCapState;//超级电容【状态标志】：各个状态对应的状态码查看E_SuperCapState枚举。
  uint8_t BatVoltage; //通过超级电容监控电池电压*10，
  uint8_t BatPower;
  int8_t DebugOut_SuperCapPower;
}CAN_TransmitDataTypeDef;

//这个是计算平均值使用的结构体
typedef struct {
  uint32_t Vcap;
  uint32_t Vbat;
  uint32_t Ibat;
  uint32_t Icap;
  uint32_t Tmos;
  uint32_t Tcap;
}ADC_ValueTypeDef;

//这个是用来记录校准参数的结构体
typedef struct {
  uint32_t UID0;
  uint32_t UID1;
  uint32_t UID2;

  float Vbat_A;
  float Vbat_B;
  float Vcap_A;
  float Vcap_B;
  float Ibat_A;
  float Ibat_B;
  float Icap_A;
  float Icap_B;

}ADC_Fit_ParametersTypeDef;



/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
//宏定义一些函数，提高可读性。使用寄存器操作的这些IO口是为了提高他的运行速度
//为什么后面的就不用寄存器操作？因为后面的函数影响不大

#define TEST_OUT_HIGH  TEST_OUT_GPIO_Port->BSRR = (uint32_t)TEST_OUT_Pin
#define TEST_OUT_LOW  TEST_OUT_GPIO_Port->BRR = (uint32_t)TEST_OUT_Pin

#define SET_PWM_COMPARE(a) htim1.Instance->CCR2 = (a)

#ifdef LITE

#define PMOS_ON CAP_CTRL_GPIO_Port->BSRR = (uint32_t)CAP_CTRL_Pin
#define PMOS_OFF CAP_CTRL_GPIO_Port->BRR = (uint32_t)CAP_CTRL_Pin

#define TIM1_PWM2_BREAK TIM1_Break_CTRL_GPIO_Port->BSRR = (uint32_t)TIM1_Break_CTRL_Pin
#define TIM1_PWM2_DISBREAK TIM1_Break_CTRL_GPIO_Port->BRR = (uint32_t)TIM1_Break_CTRL_Pin

#define TEST_MODE_PIN (TEST_MODE_GPIO_Port->IDR & TEST_MODE_Pin)

#define TIM1_PWM2_IS_OUT ((TIM1_BKIN_GPIO_Port->IDR & TIM1_BKIN_Pin) == 0)

#define LED_GREEN_ON HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_SET)
#define LED_GREEN_OFF HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_RESET)
#define LED_GREEN_BLINK HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin)

#define LED_BLUE_ON HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_SET)
#define LED_BLUE_OFF HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_RESET)
#define LED_BLUE_BLINK HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin)

#define SET_ADC_TRIGGER_COMPARE(b) htim1.Instance->CCR1 = ((b >> 3) + (b >> 2))

#endif


#ifdef PLUS

typedef struct {
  float ch_data[15];
  char tail[4];
}JustFloatFrameTypedef;

#define ENABLE_CAP  ENABLE_CAP_GPIO_Port->BSRR = (uint32_t)ENABLE_CAP_Pin
#define DISABLE_CAP  ENABLE_CAP_GPIO_Port->BRR = (uint32_t)ENABLE_CAP_Pin

#define TIM1_PWM2_IS_OUT (TIM1_BKIN_GPIO_Port->IDR & TIM1_BKIN_Pin)

#define BOOM ((BOOM_IN_GPIO_Port -> IDR & BOOM_IN_Pin) == 0)

#define LED_RED_ON HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_SET)
#define LED_RED_OFF HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_RESET)
#define LED_RED_BLINK HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin)

#define LED_GREEN_ON HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_SET)
#define LED_GREEN_OFF HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_RESET)
#define LED_GREEN_BLINK HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin)

#define LED_BLUE_ON HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_SET)
#define LED_BLUE_OFF HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_RESET)
#define LED_BLUE_BLINK HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin)

#define SET_ADC_TRIGGER_COMPARE(b) htim1.Instance->CCR1 = (b >> 1)

#endif



/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
//分频系数，只能是2的幂次
#define DIV2 0x0001
#define DIV4 0x0003
#define DIV8 0x0007
#define DIV16 0x000F
#define DIV32 0x001F
#define DIV64 0x003F
#define DIV128 0x007F
#define DIV256 0x00FF
#define DIV512 0x01FF
#define DIV1024 0x03FF
#define DIV2048 0x07FF
#define DIV4096 0x0FFF
#define DIV8192 0x1FFF

//在100Khz的环路运行下，每++一次，多少次之后能达到真实的时间的计数定义。
#define COUNT_TO_10MS_ON_100Khz 1000
#define COUNT_TO_15MS_ON_100Khz 1500
#define COUNT_TO_20MS_ON_100Khz 2000
#define COUNT_TO_50MS_ON_100Khz 5000

#define COUNT_TO_100MS_ON_100Khz 10000
#define COUNT_TO_200MS_ON_100Khz 20000
#define COUNT_TO_500MS_ON_100Khz 50000 

#define COUNT_TO_1S_ON_100Khz 100000
#define COUNT_TO_2S_ON_100Khz 200000
#define COUNT_TO_5S_ON_100Khz 500000

#define COUNT_TO_10S_ON_100Khz 1000000
/*
------------------------------------------------------------------------------------------------------------------------------------------
*/

#ifdef PLUS
//Buck电压环PI
#define PID_BUCK_V_KP 200
#define PID_BUCK_V_KI 20

//Buck电流环PI
#define PID_BUCK_I_KP 20
#define PID_BUCK_I_KI 1

//Boost电压环PI
#define PID_BOOST_V_KP 10
#define PID_BOOST_V_KI 1

#define PID_AUTOMATIC_COMPENSATION_KP 20
#define PID_AUTOMATIC_COMPENSATION_KI 2

#endif

#ifdef LITE
//Buck电压环PI
#define PID_BUCK_V_KP 200
#define PID_BUCK_V_KI 20

//Buck电流环PI
#define PID_BUCK_I_KP 10
#define PID_BUCK_I_KI 0.5

//Boost电压环PI
#define PID_BOOST_V_KP 10
#define PID_BOOST_V_KI 1

#define PID_AUTOMATIC_COMPENSATION_KP 5
#define PID_AUTOMATIC_COMPENSATION_KI 0.5f

#endif

#define MAX_PWM_COMPARE 26000 //PLUS版最大下管占空比，LITE版最大上管占空比。
#define MIN_PWM_COMPARE 2700  //PLUS版最小下管占空比，不能再小了，再小电流就采不到了。

//ADC采样NTC热敏电阻，上拉47K电阻的拟合参数，对数拟合。Plus
#define NTC3950_100K_RH_47K_Fitting_A -41.18f
#define NTC3950_100K_RH_47K_Fitting_B 367

//ADC采样NTC热敏电阻，下拉12K电阻的拟合参数，三项拟合。Lite
#define NTC3950_100K_RL_12K_Fitting_A 2e-09f
#define NTC3950_100K_RL_12K_Fitting_B -2e-05f
#define NTC3950_100K_RL_12K_Fitting_C 0.062f
#define NTC3950_100K_RL_12K_Fitting_D -7.2809f

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
//CAN通信ID
#define CAN_SUPERCAP_TO_BUS_ID 0x100 //超电发数据给电控使用的ID
#define CAN_BUS_TO_SUPERCAP_ID 0x001  //电控发数据给超电使用的ID
#define CAN_TEST_ID 0x255

#define CAN_OFFLINE_STAY  //CAN离线时，超级电容控制板状态保持
// #define CAN_OFFLINT_STOP //CAN离线时，超级电容控制板停止运行

// #define DEBUG_MODE
//测试模式，正式使用请勿开启（注释掉）
//测试模式是在不装车的情况下用的，开启测试模式之后，CAN只发不收
//开启测试模式之后，超级电容控制板的灯效会一直以“CAN离线”的状态进行显示
//开启测试模式后，功率限制默认为50W，可通过宏定义修改

#define DEBUG_MODE_POWER_LIMIT 50 //测试模式下，将底盘功率限制在50W

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
//这些是软件保护
#define SOFTWARE_OVP_VCAP 20.0f //最大电容组充电电压，电容组过压保护
#define SOFTWARE_OVP_RECOVER_VCAP 18.0f //电容组过压保护恢复阈值，必须比上面的数值小

#define SOFTWARE_UVP_VCAP 6.0f  //最小电容组放电截至电压，这里设定为6V。
//是因为电容组快没电的时候，需要迟滞一段时间才会进入欠压保护，避免瞬时的大功率导致的电容压降误触发欠压保护。
#define SOFTWARE_UVP_RECOVER_VCAP 10  //电容组欠压保护恢复阈值，必须比上面的数值大

#define SOFTWARE_OVP_VBAT 35 //电池端口最大电压，同时也是过压阈值
#define SOFTWARE_OVP_RECOVER_VBAT 30  //电池端口过压恢复阈值

#define SOFTWARE_OCP_IBAT 30 //电池最大输入电流，母线过流保护
#define SOFTWARE_OCP_RECOVER_IBAT 25  //母线过流保护恢复阈值，必须比上面的数值小

#define SOFTWARE_CHARGE_OCP_ICAP 15 //电容组最大充电电流，电容组过流保护
#define SOFTWARE_CHARGE_OCP_RECOVER_ICAP 13 //电容组保护恢复阈值，必须比上面的数值小

#define SOFTWARE_DISCHARGE_OCP_ICAP -35 //电容组最大放电电流，电容组过流保护
#define SOFTWARE_DISCHARGE_OCP_RECOVER_ICAP -30 //电容组过流保护恢复阈值，必须比上面的数值小（电流是矢量，所以-18比-20小）

#define SOFTWARE_OTP_MOS 100 //MOS过温保护。PCB板子耐温度高一点，不会有事
#define SOFTWARE_OTP_RECOVER_MOS 80 //MOS过温保护恢复阈值，必须比上面的数值小

#define SOFTWARE_OTP_CAP 70 //电容组过温保护。电容组高温寿命会衰减，建议不要继续使用了（虽然不知道影响有多大）
#define SOFTWARE_OTP_RECOVER_CAP 50 //电容组过温保护恢复阈值，必须比上面的数值小

//这些是硬件限制
#define HARDWARE_OVP_VCAP 21  //最大电容组充电电压，超过这个电压，泄放保护就会生效，长时间泄放保护板温度过高会爆炸。
#define HARDWARE_UVP_VCAP 5 //最小电容组放电截至电压，电容组电压太低，就无法满足输出功率需求
#define HARDWARE_CHARGE_OCP_ICAP 15 //电容组最大充电电流，电容组大电流充电容易发烫
#define HARDWARE_DISCHARGE_OCP_ICAP 25  //电容组最大放电电流，这个大概是电感的饱和电流，同时，长时间处于这个电流会导致温度过高
#define HARDWARE_OCP_IBAT 12  //电池最大输入电流，长时间超过这个电流会导致电管保护
#define HARDWARE_OVP_VBAT 35  //电池端口最大电压，长时间超过这个电压会导致TVS爆炸

#define SOFTWARE_UVP_BAT 19 //电池欠压保护阈值，电池低于这个电压的时候，就认为电池快要没电了，超电会无法使用

#ifdef LITE
#define PBAT_POWER_LOSS 5   //电池功率误差补偿，LITE需要补偿线损，由于不确定线长，所以给了一个比较大的值，避免超功率。

#endif

#ifdef PLUS
#define PBAT_POWER_LOSS 1   //电池功率误差补偿，PLUS版本拥有远端补偿，可以忽略线损，这个值给1是避免与裁判系统的相对误差。

#endif

// #define SUPERCAP_DCR_COMPENSATION	//想要关闭就注释掉这个宏定义
//电容组DCR补偿，用于计算剩余能量的时候，补偿一下内阻产生的压降，从而展示更真实的剩余能量
//差不多就是，电容组大电流放电的时候，由于内阻的原因，对外的电压不等于其开路时的电压，会被内阻分压掉一部分，
//所以这个功能的目的是在电容组大电流放电的时候，避免让回读的能量突然减少一大半的作用，仅仅是为了看起来更舒服！！
#define SUPERCAP_DCR 0.1f	//超级电容内阻，单位Ω，我这边测出来的超级电容内阻在100mR左右

#define SAFE_CHARGE_ICAP 10  //电容组充电安全电流
//测试发现，我所用的超级电容组，从0V开始10A恒流充电到满电（电流小于0.1A），然后100W恒功率放电到1V，连续循环10次，电容组温升50°C左右
#define SAFE_CHARGE_VCAP 10 //当电容电压过低时，必须要等待电容充电超过这个电压才会允许使用超级电容

#define SOFTSTART_CHARGE_ICAP 3 //软起动时的超级电容充电电流。
#define SOFTSTART_CHARGE_VCAP 5 //退出软起动时的超级电容充电电压。

#define PMOS_OFF_CURRENT 0.5f //当母线流过的电流小于这个值时，PMOS关闭，作为机器人死亡的判断依据，起到底盘断电的作用
#define PMOS_ON_CURRENT 1 //当母线流过的电流大于这个值时，PMOS打开，作为机器人运行的判断依据，超级电容恢复工作

#define PMOS_OFF_CURRENT_ON_DISCHARGE 1 //放电的时候，当底盘流过的电流小于这个值时，PMOS关闭，作为机器人死亡的判断依据，起到底盘断电的作用
#define PMOS_ON_CURRENT_ONDISCHARGE 1.5f 

#define TRICKLE_CHARGE_CURRENT_CAP 0.5f //涓流电流，充电的时候，使用这个电流来判断是否关闭超电

#define SUPERCAP_AVAILABLE_VOLTAGE  (SOFTWARE_OVP_VCAP - SOFTWARE_UVP_VCAP) //电容组可用电压范围，用于粗略计算电容组的能量百分比
/*
------------------------------------------------------------------------------------------------------------------------------------------
*/ 

extern uint16_t ADC1Value[3];
extern uint16_t ADC2Value[3];


void ADC_Curve_Fitting(void);
void FDCAN_Filter_Init(void);
void Power_Loop_Parameter_Init(void);
void Protection_Init(void);
void A_Timing_Ranking_Idea(void);

void Free_Loop(void);

#ifdef __cplusplus
}
#endif

#endif /* SUPERCAP_CTRL_H */
