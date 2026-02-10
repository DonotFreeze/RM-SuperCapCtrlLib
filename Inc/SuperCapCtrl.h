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
  OCP_PROTECTION = 4,
  OVP_BAT_PROTECTION = 5,
  UVP_BAT_PROTECTION = 6,
  UVP_CAP_PROTECTION = 7,
  OTP_PROTECTION = 8
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
  uint8_t OCP_bit ; //控制板过流保护。1过流，0正常
  uint8_t OVP_Cap_bit ; //电容组过压保护。1过压，0正常
  uint8_t OVP_Bat_bit ; //控制板过压保护。1过压，0正常
} StateFlagsTypeDef;


//电控发过来的数据
typedef struct {
  uint8_t Charge ; //超级电容充电。1充电，0放电
  uint8_t Enable ; //超级电容给使能。1使能，0失能
  uint8_t PowerLimint;  //裁判系统功率限制
  uint8_t Dead; //机器人死亡状态，没用到
  float PowerLimitAfterOffset;
}CAN_ReceiveDataTypeDef;


//发送给电控的数据
typedef struct {
  uint8_t SuperCapEnergy;//超级电容可用能量：0-100%
  uint8_t ChassisPower; //底盘实时功率：0-255。绝大多数情况下，底盘功率只会运行在200W内，超出200W的基本都是瞬时的，可以忽略，所以uint8就够用了
  SuperCapReadyTypeDef SuperCapReady;//超级电容【可用标志】：1为可用，0为不可用
  SuperCapStateTypeDef SuperCapState;//超级电容【状态标志】：各个状态对应的状态码查看E_SuperCapState枚举。
  uint8_t VoltageBat; //通过超级电容监控电池电压*10，
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
#define PMOS_ON CAP_CTRL_GPIO_Port->BSRR = (uint32_t)CAP_CTRL_Pin
#define PMOS_OFF CAP_CTRL_GPIO_Port->BRR = (uint32_t)CAP_CTRL_Pin

#define TIM1_PWM2_BREAK TIM1_Break_CTRL_GPIO_Port->BSRR = (uint32_t)TIM1_Break_CTRL_Pin
#define TIM1_PWM2_DISBREAK TIM1_Break_CTRL_GPIO_Port->BRR = (uint32_t)TIM1_Break_CTRL_Pin

#define TEST_OUT_HIGH  TEST_OUT_GPIO_Port->BSRR = (uint32_t)TEST_OUT_Pin
#define TEST_OUT_LOW  TEST_OUT_GPIO_Port->BRR = (uint32_t)TEST_OUT_Pin

#define TEST_MODE (TEST_MODE_GPIO_Port->IDR & TEST_MODE_Pin)

#define LED_CAP_ON HAL_GPIO_WritePin(LED_CAP_GPIO_Port,LED_CAP_Pin,GPIO_PIN_SET)
#define LED_CAP_OFF HAL_GPIO_WritePin(LED_CAP_GPIO_Port,LED_CAP_Pin,GPIO_PIN_RESET)
#define LED_CAP_BLINK HAL_GPIO_TogglePin(LED_CAP_GPIO_Port,LED_CAP_Pin)

#define LED_CHASSIS_ON HAL_GPIO_WritePin(LED_CHASSIS_GPIO_Port,LED_CHASSIS_Pin,GPIO_PIN_SET)
#define LED_CHASSIS_OFF HAL_GPIO_WritePin(LED_CHASSIS_GPIO_Port,LED_CHASSIS_Pin,GPIO_PIN_RESET)
#define LED_CHASSIS_BLINK HAL_GPIO_TogglePin(LED_CHASSIS_GPIO_Port,LED_CHASSIS_Pin)


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
//Buck电压环PI
#define PID_BUCK_V_KP 200
#define PID_BUCK_V_KI 20

//Buck电流环PI
#define PID_BUCK_I_KP 10
#define PID_BUCK_I_KI 5

//Boost电压环PI
#define PID_BOOST_V_KP 10
#define PID_BOOST_V_KI 1

//Boost电流环PI
#define PID_BOOST_I_KP 10
#define PID_BOOST_I_KI 1

//超级电容充电（功率环？）PI
#define PID_CAP_CHARGE_KP 20
#define PID_CAP_CHARGE_KI 1.5

//电池与超电同时输出（功率环？）PI
#define PID_DISCHARGE_BTA_PLIMIT_KP 20
#define PID_DISCHARGE_BTA_PLIMIT_KI 1.5

//Buck工作模式下的最大和最小占空比（最大最小PID输出）
#define BUCK_DUTY_COMPARE_MAX 26000
#define BUCK_DUTY_COMPARE_MIN 100

//Boost工作模式下的最大和最小占空比（最大最小PID输出）
#define BOOST_DUTY_COMPARE_MAX 23000
#define BOOST_DUTY_COMPARE_MIN 1000

//ADC采样NTC热敏电阻的拟合参数，采用二项拟合。
//其实二项拟合效果不算太好，超过50度会偏大，小于50度又偏小。如果使用三项拟合基本就与实际一致了。
#define NTC3950_100K_Fitting_A -5E-06f
#define NTC3950_100K_Fitting_B 0.0515f
#define NTC3950_100K_Fitting_C -12.347f

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
//CAN通信ID
#define CAN_SUPERCAP_ID 0x100 //电控发数据给超电使用的ID
#define CAN_C_BOARD_ID 0x001  //超电发数据给电控使用的ID
#define CAN_TEST_ID 0x255

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
//这些是软件保护
#define SOFTWARE_OVP_VCAP 20.0f //最大电容组充电电压，电容组过压保护
#define SOFTWARE_OVP_RECOVER_VCAP 19 //电容组过压保护恢复阈值，必须比上面的数值小

#define SOFTWARE_UVP_VCAP 5.0f  //最小电容组放电截至电压，这里设定为5V，比20V的30%小一点。
//是因为电容组低压下电流比较大，会产生较大的线损，板子读到的电压就偏小了，这也算是一种补偿。
#define SOFTWARE_UVP_RECOVER_VCAP 10  //电容组欠压保护恢复阈值，必须比上面的数值大

#define SOFTWARE_OVP_VBAT 30 //电池端口最大电压
#define SOFTWARE_OVP_RECOVER_VBAT 28  //MOS过温保护恢复阈值，必须比上面的数值小

#define SOFTWARE_CHARGE_OCP_ICAP 10 //电容组最大充电电流，电容组过流保护
#define SOFTWARE_CHARGE_OCP_RECOVER_ICAP 8 //电容组保护恢复阈值，必须比上面的数值小

#define SOFTWARE_DISCHARGE_OCP_ICAP -20 //电容组最大放电电流，电容组过流保护
#define SOFTWARE_DISCHARGE_OCP_RECOVER_ICAP -18 //电容组过流保护恢复阈值，必须比上面的数值小（电流是矢量，所以-18比-20小）

#define SOFTWARE_OCP_IBAT 10 //电池最大输入电流，母线过流保护
#define SOFTWARE_OCP_RECOVER_IBAT 8  //母线过流保护恢复阈值，必须比上面的数值小

#define SOFTWARE_OTP_MOS 100 //MOS过温保护。PCB板子耐温度高一点，不会有事
#define SOFTWARE_OTP_RECOVER_MOS 70 //MOS过温保护恢复阈值，必须比上面的数值小

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
#define PBAT_POWER_LOSS 10   //电池功率误差补偿，通常包含线损和功率误差的补偿

#define SAFE_CHARGE_ICAP 5  //电容组充电安全电流，这个是根据超级电容手册中的额定电流来决定的
#define SAFE_CHARGE_VCAP 10 //当电容电压过低时，必须要等待电容充电超过这个电压才会允许使用超级电容

#define SOFTSTART_CHARGE_ICAP 3 //软起动时的超级电容充电电流。

#define PMOS_OFF_CURRENT 0.5f //当底盘流过的电流小于这个值时，PMOS关闭，作为机器人死亡的判断依据，起到底盘断电的作用
#define PMOS_ON_CURRENT 1 

#define TRICKLE_CHARGE_CURRENT_CAP 0.3f //涓流电流，充电的时候，使用这个电流来判断是否关闭超电

#define SUPERCAP_AVAILABLE_VOLTAGE  (SOFTWARE_OVP_VCAP - SOFTWARE_UVP_VCAP) //电容组可用电压范围，用于粗略计算电容组的能量百分比
/*
------------------------------------------------------------------------------------------------------------------------------------------
*/ 

extern uint16_t ADC1Value[3];
extern uint16_t ADC2Value[3];



void ADC_Curve_Fitting(void);
void FDCAN_Filter_Init(void);
void Power_Loop_Parameter_Init(void);
void A_Timing_Ranking_Idea(void);


#ifdef __cplusplus
}
#endif

#endif /* SUPERCAP_CTRL_H */
