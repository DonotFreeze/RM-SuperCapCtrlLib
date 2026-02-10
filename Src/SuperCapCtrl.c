/**********************************************************************************************
 * 超级电容功率控制库：V1.1.2504
 * 代码建立日期：2024年4月28日
 * 最后修改日期：2025年1月26日
 * 编码格式：GB2312
 * 作者：某桂工的魔法电容使者
 * 
 * 本代码为RM2024桂林理工大学群星战队开源的超级电容控制板Lite的完整代码
 * 
 * 本代码搭配超级电容控制板LiteV1.1的PCB使用
 * 
 * 本代码使用MIT协议进行授权。
 **********************************************************************************************/

#include "SuperCapCtrl.h"
#include "tim.h"
#include "fdcan.h"
#include "string.h" //给memcmp用的
#include "iwdg.h"

PID_ParameterTypeDef sPID_buck_V              = {0};
PID_ParameterTypeDef sPID_buck_I              = {0};
PID_ParameterTypeDef sPID_boost_V             = {0};
PID_ParameterTypeDef sPID_boost_I             = {0};
PID_ParameterTypeDef sPID_PowerLoop_Charge    = {0};
PID_ParameterTypeDef sPID_PowerLoop_Discharge = {0};

ADC_ValueTypeDef sADC_Value_SUM = {0};
ADC_ValueTypeDef sADC_Value_AVG = {0};

FDCAN_FilterTypeDef sFilterConfig;
FDCAN_RxHeaderTypeDef sFDCAN1_RxHeader;
FDCAN_TxHeaderTypeDef sFDCAN1_TxHeader;

CAN_TransmitDataTypeDef sCAN_TX_data = {0};
CAN_ReceiveDataTypeDef sCAN_RX_data  = {0};

StateFlagsTypeDef sStateBit = {0};

// 滞回比较器的状态存储变量
ComparatorStateTypeDef IbatOCPEdge = FALLING;
ComparatorStateTypeDef IcapOCPEdge = FALLING;

ComparatorStateTypeDef VbatOVPEdge = FALLING;
ComparatorStateTypeDef VbatUVPEdge = RISING;
ComparatorStateTypeDef VcapOVPEdge = FALLING;
ComparatorStateTypeDef VcapUVPEdge = RISING;

ComparatorStateTypeDef VcapDischargeOVPEgde = FALLING;

ComparatorStateTypeDef TmosOTPEdge = FALLING;
ComparatorStateTypeDef TcapOTPEdge = FALLING;

ComparatorStateTypeDef VcapUVPRecoverEdge = FALLING;
ComparatorStateTypeDef IbatPMOSOffEdge    = RISING;

const ADC_Fit_ParametersTypeDef csADC_Fit_Parameters[10] = {
    // UID0     ,UID1      ,UID2       ,Vbat_A  ,Vbat_B  ,Vcap_A  ,Vcap_B  ,Ibat_A  ,Ibat_B  ,Icap_A  ,Icap_B
    {0x003B0057, 0x54315005, 0x20333830, 0.0095f, 0.0173f, 0.0068f, 0.0461f, 0.0036f, 0.0360f, 0.0123f, -33.496f},
    {0x003C0022, 0x54315005, 0x20333830, 0.0094f, 0.0175f, 0.0068f, 0.0460f, 0.0036f, 0.0360f, 0.0123f, -33.496f},
    {0x003B006D, 0x54315005, 0x20333830, 0.0095f, 0.0173f, 0.0068f, 0.0461f, 0.0036f, 0.0360f, 0.0123f, -33.496f},
    {0x003D0022, 0x54315005, 0x20333830, 0.0095f, 0.0173f, 0.0068f, 0.0461f, 0.0036f, 0.0360f, 0.0123f, -33.496f},
    {0x004F0051, 0x484E5006, 0x20353031, 0.0096f, 0.0732f, 0.0068f, 0.0501f, 0.0040f, 0.1937f, 0.0123f, -33.372f},
    {0x00230060, 0x484E5004, 0x20353031, 0.0095f, 0.0349f, 0.0068f, 0.0595f, 0.0042f, 0.1252f, 0.0129f, -34.796f},
    {0x00250045, 0x484E5004, 0x20353031, 0.0095f, 0.0793f, 0.0068f, 0.0441f, 0.0040f, 0.1331f, 0.0129f, -34.918f},
    {0x001B0051, 0x484E5004, 0x20353031, 0.0095f, 0.0476f, 0.0067f, 0.0569f, 0.0041f, 0.1250f, 0.0128f, -34.728f},
    {0x0023005B, 0x484E5004, 0x20353031, 0.0095f, 0.0731f, 0.0068f, 0.0180f, 0.0041f, 0.1517f, 0.0128f, -34.490f},
    {0x001E0044, 0x484E5004, 0x20353031, 0.0095f, 0.0631f, 0.0068f, 0.0413f, 0.0042f, 0.2127f, 0.0127f, -34.627f},
    };

// 初始化之后，ADC的校拟合参数就会被传入这个结构体，也就是在代码中会调用的数据。
ADC_Fit_ParametersTypeDef sADC_Fit;

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/

static inline void ADC_Convert_To_Reality(void);
static inline void Power_Loop(void);

static inline void Power_Calculations(void);
static inline void Temperature_Calculations(void);

static inline void Over_Temperature_Protect_Loop(void);
static inline void CAN_Offline_Loop(void);
static inline void State_Change(void);
static inline void Soft_Start_Loop(void);
static inline void Over_Current_Protection_Loop(void);
static inline void Bat_Over_Voltage_Protection_Loop(void);
static inline void Bat_Undervoltage_Protection_Loop(void);
static inline void CAP_Undervoltage_Protection_Loop(void);
static inline void Charge_Loop(void);
static inline void Discharge_Loop(void);
static inline void PMOS_Off(void);
static inline void Wait_Loop(void);

static inline void Can_SendMess(CAN_TransmitDataTypeDef *TX_temp);
static inline void ADC_Value_AVG_Compute(uint32_t Times);
static inline void LED_Refresh(void);
static inline void Hysteresis_Comparator(float input, float rising_threshold, float falling_threshold, ComparatorStateTypeDef *state);

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/

uint16_t ADC1Value[3] = {0};
uint16_t ADC2Value[3] = {0};

uint32_t CAN_WDG_Count = 0; // CAN离线检测看门狗计数

uint32_t PID_PowerLoopOut   = 0; // 中间量，功率环PID输出
uint32_t PID_CurrentLoopOut = 0; // 中间量，电流环PID输出
uint32_t PID_VoltageLoopOut = 0; // 中间量，电压环PID输出
uint32_t TIM1_PWM2_Comapre  = 0; // 最后赋值给PWM比较值的量，决定占空比输出

float Vbat; // 【ADC2_0】电池电压、底盘供电电压
float Vcap; // 【ADC2_1】超级电容组电压
float Tcap; // 【ADC2_2】超级电容组温度

float Ibat; // 【ADC1_0】电池供电电流（裁判系统Chassis口电流）【功率限制】
float Icap; // 【ADC1_1】超级电容组电流（充电为正，放电为负）
float Tmos; // 【ADC1_2】超级电容控制板半桥温度

float SafeChargePcap = 0; // 额定电容充电功率，使用Vcap*Icap计算，用于限制电容的充电功率

float Pcap        = 0; // 环路计算中间量，电容端口功率
float Pbat        = 0; // 环路计算中间量，电池端口功率
float P_ChargeCap = 0; // 电容充电时候设定的功率


uint32_t PMOS_OffDelay = 0; //PMOS关断延迟

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
//
void Power_Loop_Parameter_Init(void){

    TIM1_PWM2_BREAK;
    // 开机需要把半桥关掉，防止初始比较值产生的PWM导致超电爆炸。
    PMOS_OFF;
    // 开机直接先把PMOS关掉。

    sStateBit.SoftStart_bit = 1; // 第一次开机必须是软起动
    sStateBit.Charge_bit    = 1; // 除了处于放电状态，充电标志位都应该是1
    sStateBit.Enable_bit    = 0; // 上电处于失能状态，等待CAN信息。

    sStateBit.CAN_Offline_bit = 1; // 上电默认CAN掉线，必须收到CAN信息之后才会运行。
    sStateBit.UVP_Bat_bit     = 0;
    sStateBit.UVP_Cap_bit     = 0;
    sStateBit.OTP_MOS_bit     = 0;
    sStateBit.OTP_CAP_bit     = 0;
    sStateBit.OCP_bit         = 0;
    sStateBit.OVP_Bat_bit     = 0;

    sPID_buck_V.Kp                  = PID_BUCK_V_KP;
    sPID_buck_V.Ki                  = PID_BUCK_V_KI;
    sPID_buck_V.OutMax              = BUCK_DUTY_COMPARE_MAX;
    sPID_buck_V.OutMin              = BUCK_DUTY_COMPARE_MIN;
    sPID_buck_V.ControllerDirection = DIRECT; // 意思是，控制方向与变化方向相同
    PID_Init(&sPID_buck_V);
    // Buck电压环PID参数初始化

    sPID_buck_I.Kp                  = PID_BUCK_I_KP;
    sPID_buck_I.Ki                  = PID_BUCK_I_KI;
    sPID_buck_I.OutMax              = BUCK_DUTY_COMPARE_MAX;
    sPID_buck_I.OutMin              = BUCK_DUTY_COMPARE_MIN;
    sPID_buck_I.ControllerDirection = DIRECT; // 意思是，控制方向与变化方向相同
    PID_Init(&sPID_buck_I);
    // Buck电流环PID参数初始化

    sPID_boost_V.Kp                  = PID_BOOST_V_KP;
    sPID_boost_V.Ki                  = PID_BOOST_V_KI;
    sPID_boost_V.OutMax              = BOOST_DUTY_COMPARE_MAX;
    sPID_boost_V.OutMin              = BOOST_DUTY_COMPARE_MIN;
    sPID_boost_V.ControllerDirection = REVERSE; // 意思是，控制方向与变化方向相反
    // 当设定输出为20V，当前输出为10V，正方向的PID提高电压的方法是增大输出，PID的输出直接作为上管PWM的占空比，当PID输出（上管占空比）变大时，电压变小；故控制反向
    PID_Init(&sPID_boost_V);
    // Boost电压环PID参数初始化

    sPID_boost_I.Kp                  = PID_BOOST_I_KP;
    sPID_boost_I.Ki                  = PID_BOOST_I_KI;
    sPID_boost_I.OutMax              = BOOST_DUTY_COMPARE_MAX;
    sPID_boost_I.OutMin              = BOOST_DUTY_COMPARE_MIN;
    sPID_boost_I.ControllerDirection = REVERSE; // 意思是，控制方向与变化方向相反
    // 当设定输出为10A，当前输出为5A，提高输出电流的方法是增大输出电压，故与上面相同，控制反向
    PID_Init(&sPID_boost_I);
    // Boost电流环PID参数初始化

    sPID_PowerLoop_Charge.Kp                  = PID_CAP_CHARGE_KP;
    sPID_PowerLoop_Charge.Ki                  = PID_CAP_CHARGE_KI;
    sPID_PowerLoop_Charge.OutMax              = BUCK_DUTY_COMPARE_MAX;
    sPID_PowerLoop_Charge.OutMin              = BUCK_DUTY_COMPARE_MIN;
    sPID_PowerLoop_Charge.ControllerDirection = DIRECT; // 意思是，控制方向与变化方向同向
    PID_Init(&sPID_PowerLoop_Charge);

    sPID_PowerLoop_Discharge.Kp                  = PID_DISCHARGE_BTA_PLIMIT_KP;
    sPID_PowerLoop_Discharge.Ki                  = PID_DISCHARGE_BTA_PLIMIT_KI;
    sPID_PowerLoop_Discharge.OutMax              = BOOST_DUTY_COMPARE_MAX;
    sPID_PowerLoop_Discharge.OutMin              = BOOST_DUTY_COMPARE_MIN;
    sPID_PowerLoop_Discharge.ControllerDirection = DIRECT; // 意思是，控制方向与变化方向同向
    // 当电池功率限制50W，当前输出30W，提高电池输出趋近功率限制，就需要将Boost的输出功率降低，降低功率的方法是降低Boost电压，
    // 想要降低Boost的输出电压就需要增大上管的占空比，故输出需要增大；故控制方向正向
    PID_Init(&sPID_PowerLoop_Discharge);
}

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/

uint32_t TimingCNT = 0; // 时间片计数，用于区分不同时间节点运行的不同程序。

// ATRI算法，名为一个时间片算法，就是硬凑
// 在周期中断中调用保证其定时运行
void A_Timing_Ranking_Idea(void){
    TEST_OUT_HIGH;
    TimingCNT++;

    ADC_Convert_To_Reality();

    State_Change();
    Power_Loop();

    ADC_Value_AVG_Compute(DIV128); // 运行频率：100K/128 = 781hz

    if ((TimingCNT & DIV128) == 1) Power_Calculations();        // 运行频率：100K/128 = 781hz
    if ((TimingCNT & DIV128) == 2) Can_SendMess(&sCAN_TX_data); // 运行频率：100K/128 = 781hz
    // 计算平均值、功率计算、CAN发送。数据更新将以最慢的那个为准。

    if ((TimingCNT & DIV8192) == 3) Temperature_Calculations(); // 运行频率：100K/8192 = 12hz
    if ((TimingCNT & DIV8192) == 10) LED_Refresh();             // 运行频率：100K/8192 = 12hz
    if ((TimingCNT & DIV8192) == 10) HAL_IWDG_Refresh(&hiwdg);  // 运行频率：100K/8192 = 12hz

    TEST_OUT_LOW;
}

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
// 这个函数的作用是，将ADC的值转化为其对应的各个量的真实值
void ADC_Convert_To_Reality(void){
    Vbat = ADC2Value[0] * sADC_Fit.Vbat_A + sADC_Fit.Vbat_B;
    Vcap = ADC2Value[1] * sADC_Fit.Vcap_A + sADC_Fit.Vcap_B;

    Ibat = ADC1Value[0] * sADC_Fit.Ibat_A + sADC_Fit.Ibat_B;
    Icap = ADC1Value[1] * sADC_Fit.Icap_A + sADC_Fit.Icap_B;
}

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
// 这个函数是将功率计算出来
void Power_Calculations(void){
    float Vbat_AVG =0;
    float Vcap_AVG =0;
    float Ibat_AVG =0;
    float Icap_AVG =0;

    uint8_t BatPower;
    int SuperCapPower;

    Vbat_AVG = sADC_Value_AVG.Vbat * sADC_Fit.Vbat_A + sADC_Fit.Vbat_B;
    Vcap_AVG = sADC_Value_AVG.Vcap * sADC_Fit.Vcap_A + sADC_Fit.Vcap_B;

    Ibat_AVG = sADC_Value_AVG.Ibat * sADC_Fit.Ibat_A + sADC_Fit.Ibat_B;
    Icap_AVG = sADC_Value_AVG.Icap * sADC_Fit.Icap_A + sADC_Fit.Icap_B;

    BatPower      = Ibat_AVG * Vbat_AVG;
    SuperCapPower = Vcap_AVG * Icap_AVG;

    if (sStateBit.Charge_bit) {
        sCAN_TX_data.ChassisPower = BatPower +PBAT_POWER_LOSS; // 充电的时候底盘消耗的总功率（电机+电容充电）就是电池功率啦，+上电池功率损失是一个补偿。

    } else if (!sStateBit.Charge_bit) {
        // 放电的时候，电流是负的，所以他算出来的放电功率也是负的
        // 电池功率 - 放电功率（负的），负负的正，总功率就是|电容放电功率| + |电池功率|。虽然上面的也是剪掉，但是我这样区分来写更清楚一点。
        if ((BatPower - SuperCapPower) < 250) {
            // 限制大小防止上溢
            sCAN_TX_data.ChassisPower = BatPower - SuperCapPower + PBAT_POWER_LOSS;
        } else {
            sCAN_TX_data.ChassisPower = 250;
        }
    }
    if (Vcap > SOFTWARE_UVP_VCAP) {
        // 限制大小，防止下溢
        sCAN_TX_data.SuperCapEnergy = (Vcap - SOFTWARE_UVP_VCAP) * 100 / SUPERCAP_AVAILABLE_VOLTAGE;
    } else {
        sCAN_TX_data.SuperCapEnergy = 0;
    }
}

uint32_t T_count = 0;
/**
 * 温度计算
 */
void Temperature_Calculations(void){
    T_count++;
    // 因为温度的拟合使用二次项函数，需要进行三次乘法运算，比较花时间，所以让MOS和CAP的温度交替计算，以减小算力开销，反正这玩意实时性要求不高。
    if (T_count & 1)
        Tcap = NTC3950_100K_Fitting_A * sADC_Value_AVG.Tcap * sADC_Value_AVG.Tcap + NTC3950_100K_Fitting_B * sADC_Value_AVG.Tcap + NTC3950_100K_Fitting_C;
    else
        Tmos = NTC3950_100K_Fitting_A * sADC_Value_AVG.Tmos * sADC_Value_AVG.Tmos + NTC3950_100K_Fitting_B * sADC_Value_AVG.Tmos + NTC3950_100K_Fitting_C;
}

/**
 * 滞回比较器函数，用于边界过度，防止震荡
 * input：输入
 * rising_threshold：上升阈值
 * falling_threshold：下降阈值
 * state：状态，ComparatorState枚举类型的变量，用于保存不同参数变化状态。
 */
void Hysteresis_Comparator(float input, float rising_threshold, float falling_threshold, ComparatorStateTypeDef *state){

    if (*state == FALLING) {
        // 初始状态为LOW
        if (input > rising_threshold) {
            // 如果输入超过上阈值
            *state = RISING; // 更新状态为HIGH
        }
    } else { // 当前状态为HIGH
        if (input < falling_threshold) {
            // 如果输入低于下阈值
            *state = FALLING; // 更新状态为LOW
        }
    }
    // 如果没有状态变化，则保持当前状态
}

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/

uint32_t OCP_Delay    = 0;
uint32_t OVP_CapDelay = 0;
uint32_t OVP_BatDelay = 0;
uint32_t OTP_Delay    = 0;
uint32_t UVP_BatDelay = 0;

/**
 * 这个函数是用来改变状态标志位的，程序需要改变当前的运行状态均需要通过这个函数来判断。
 * 需要注意的是，有一些保护状态，一旦进入，这条函数就不能改变他它的运行状态了，只能依靠保护函数对应的逻辑来退出。
 */
void State_Change(void){
    // 超电的充电与放电，使能直接由电控来决定。但是超电的保护优先级永远高于电控的指令。
    sStateBit.Charge_bit = sCAN_RX_data.Charge;
    sStateBit.Enable_bit = sCAN_RX_data.Enable;

    // 判断是否过流保护
    // 电源管理模块，Chassis端口保护时间实测：
    // 15A，无保护
    // 16A，15-20s
    // 20A，2s
    // 25A，800ms
    // 30A，400ms
    // 我这里过流保护阈值太低了，是因为我的检流量程就这么大，建议和电管保持同步量程，30A过流保护。不然容易误触发。
    Hysteresis_Comparator(Ibat, SOFTWARE_OCP_IBAT, SOFTWARE_OCP_RECOVER_IBAT, &IbatOCPEdge);
    if (IbatOCPEdge == RISING) {
        OCP_Delay++;
        // 迟滞防止误保护
        if (OCP_Delay >= COUNT_TO_1S_ON_100Khz) {
            sStateBit.OCP_bit = 1;
            OCP_Delay             = 0;
        }
    } else if (IbatOCPEdge == FALLING) {
        OCP_Delay = 0;
    }

    // 判断电容组电压是否过压保护，电容组过压是由于过充引起的（可能会在某些情况下出现）
    Hysteresis_Comparator(Vcap, HARDWARE_OVP_VCAP, SOFTWARE_OVP_RECOVER_VCAP, &VcapOVPEdge);
    if (VcapOVPEdge == RISING) {
        OVP_CapDelay++;
        // 迟滞防止误保护
        if (OVP_CapDelay >= COUNT_TO_5S_ON_100Khz) {
            // 保护时间设置久一点，反正有泄放电路顶着，太久也不行，泄放太烫。
            sStateBit.OVP_Cap_bit = 1;
            OVP_CapDelay          = 0;
        }
    } else if (VcapOVPEdge == FALLING) {
        OVP_CapDelay = 0;
    }

    // 判断母线电压是否过压，母线电压过压是由电机刹车动能回收引起的。
    Hysteresis_Comparator(Vbat, SOFTWARE_OVP_VBAT, SOFTWARE_OVP_RECOVER_VBAT, &VbatOVPEdge);
    if (VbatOVPEdge == RISING) {
        OVP_BatDelay++;
        // 迟滞防止误保护
        if (OVP_BatDelay >= COUNT_TO_100MS_ON_100Khz) {
            sStateBit.OVP_Bat_bit = 1;
            OVP_BatDelay          = 0;
        }
    } else if (VbatOVPEdge == FALLING) {
        OVP_BatDelay = 0;
    }

    // 判断电池是否低电压保护
    Hysteresis_Comparator(Vbat, SOFTWARE_UVP_BAT + 2, SOFTWARE_UVP_BAT, &VbatUVPEdge);
    if (VbatUVPEdge == FALLING) {
        UVP_BatDelay++;
        // 迟滞防止误保护
        if (UVP_BatDelay >= COUNT_TO_5S_ON_100Khz) {
            sStateBit.UVP_Bat_bit = 1;
            UVP_BatDelay          = 0;
        }
    } else if (VbatUVPEdge == RISING) {
        UVP_BatDelay = 0;
    }

    // 判断是否要进入温度保护
    Hysteresis_Comparator(Tmos, SOFTWARE_OTP_MOS, SOFTWARE_OTP_RECOVER_MOS, &TmosOTPEdge);
    Hysteresis_Comparator(Tcap, SOFTWARE_OTP_CAP, SOFTWARE_OTP_RECOVER_CAP, &TcapOTPEdge);
    if (TmosOTPEdge == RISING || TcapOTPEdge == RISING) {
        OTP_Delay++;
        // 迟滞防止误保护，但是其实温度更新速度很慢
        if (OTP_Delay >= COUNT_TO_5S_ON_100Khz) {
            // 温度保护久一点在生效也没事，反正烧不坏
            // 两个放一起是因为懒得分开了，反正有一个保护就不给用就是了
            sStateBit.OTP_MOS_bit = 1;
            sStateBit.OTP_CAP_bit = 1;
        }
    } else if (TmosOTPEdge == FALLING || TcapOTPEdge == FALLING) {
        OTP_Delay = 0;
    }

    // 判断CAN是否离线。
    CAN_WDG_Count++;
    // 如果持续1s没有收到CAN消息（看门狗计数没被清零），则判定CAN离线
    if (CAN_WDG_Count > COUNT_TO_1S_ON_100Khz) sStateBit.CAN_Offline_bit = 1;
}

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
/**这里是整个电源控制环路
 * 这里使用if是因为写起来比较方便，同时可以拥有优先级判断，能够保证所有保护均会生效。
 */
void Power_Loop(){
    if (sStateBit.CAN_Offline_bit) {
        // CAN离线
        CAN_Offline_Loop();
    } else if (sStateBit.SoftStart_bit) {
        // 软起动
        sCAN_TX_data.SuperCapReady = 0;
        sCAN_TX_data.SuperCapState = SOFRSTART_PROTECTION;
        Soft_Start_Loop();
    } else if (sStateBit.OCP_bit) {
        // 过流保护，也是短路保护
        // 过流不会CC，只会直接关断。
        sCAN_TX_data.SuperCapReady = 0;
        sCAN_TX_data.SuperCapState = OCP_PROTECTION;
        Over_Current_Protection_Loop();
    } else if (sStateBit.OVP_Bat_bit) {
        // 母线过压保护
        sCAN_TX_data.SuperCapReady = 0;
        sCAN_TX_data.SuperCapState = OVP_BAT_PROTECTION;
        Bat_Over_Voltage_Protection_Loop();
    } else if (sStateBit.UVP_Bat_bit) {
        // 电池欠压保护，一旦进入，就必须换电池
        sCAN_TX_data.SuperCapReady = 0;
        sCAN_TX_data.SuperCapState = UVP_BAT_PROTECTION;
        Bat_Undervoltage_Protection_Loop();
    } else if (sStateBit.UVP_Cap_bit) {
        //  超级电容低电压保护，设定值为电容组额定电压的30%
        sCAN_TX_data.SuperCapReady = 0;
        sCAN_TX_data.SuperCapState = UVP_CAP_PROTECTION;
        CAP_Undervoltage_Protection_Loop();
    } else if (sStateBit.OVP_Cap_bit) {
        //  超级电容过充电保护。
        //  超级电容过充保护已经结合在环路中了，他会进行CV。
        sCAN_TX_data.SuperCapReady = 0;
        sCAN_TX_data.SuperCapState = UVP_CAP_PROTECTION;
    } else if (sStateBit.OTP_CAP_bit || sStateBit.OTP_MOS_bit) {
        // 过温保护
        sCAN_TX_data.SuperCapReady = 0;
        sCAN_TX_data.SuperCapState = OTP_PROTECTION;
        Over_Temperature_Protect_Loop();
    } else if (sStateBit.Charge_bit && sStateBit.Enable_bit) {
        // 【充电】：让超电充电，且使能超电
        sCAN_TX_data.SuperCapReady = 1;
        sCAN_TX_data.SuperCapState = CHARGE;
        Charge_Loop();
    } else if ((!sStateBit.Charge_bit) && sStateBit.Enable_bit) {
        // 【放电】：让超电放电，且使能超电
        sCAN_TX_data.SuperCapReady = 1;
        sCAN_TX_data.SuperCapState = DISCHARGE;
        Discharge_Loop();
    } else if (!sStateBit.Enable_bit) {
        // 【待机】：未开启超电，且移动；防止超电充电会跟底盘抢功率
        //  开启超电，且不移动；防止超电无脑将电池功率拉至功率限制，但是车子又不走，此时所有功率都会往超级电容走，可能会导致超级电容被充爆
        sCAN_TX_data.SuperCapReady = 1;
        sCAN_TX_data.SuperCapState = WAIT;
        Wait_Loop();
    }
}

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
uint32_t SaftChargeOutDelay = 0;
/**
 * 缓启动保护，每次上电必须执行。
 * sStateBit.SoftStart_bit =1
 * 电控发过来的信息以下两个必须为：
 * sCAN_RX_data.Charge = 1
 * sCAN_RX_data.Enable = 1
 * 发回去给电控的数据
 * sCAN_TX_data.SuperCapReady = 0
 * sCAN_TX_data.SuperCapState = 3
 */
void Soft_Start_Loop(void){
    // 软起动的时候，PMOS是处于关闭状态的，但是不影响充电，因为功率比较小，不会导致PMOS烧毁
    if (sCAN_RX_data.Charge && sCAN_RX_data.Enable) {
        PID_Preload_Integral(&sPID_buck_I, Vcap);
        PID_Compute(&sPID_buck_I, Icap, SOFTSTART_CHARGE_ICAP, &PID_CurrentLoopOut);

        PID_Preload_Integral(&sPID_buck_V, Vcap);
        PID_Compute(&sPID_buck_V, Vcap, SOFTWARE_OVP_VCAP, &PID_VoltageLoopOut);

        Loop_Competition_Buck(PID_CurrentLoopOut, PID_VoltageLoopOut, &TIM1_PWM2_Comapre);

        // 半桥PWM比较值赋值，设定PWM占空比，这里是以上半桥的正占空比进行设置。
        htim1.Instance->CCR2 = TIM1_PWM2_Comapre;
        // ADC采样触发比较值赋值，设定为上半桥开通时间的3/8处
        htim1.Instance->CCR1 = (TIM1_PWM2_Comapre >> 3) + (TIM1_PWM2_Comapre >> 2);
        // 需要更新PMW比较值之后才能输出PWM，防止PWM比较值初始值与目标值差异过大而导致爆炸。
        TIM1_PWM2_DISBREAK;
    } else {
        TIM1_PWM2_BREAK;
        PID_Clear_Integral(&sPID_buck_I);
    }

    // 第一次上电的时候需要充电到10V以上才会退出缓启动
    if (Vcap > SAFE_CHARGE_VCAP) {
        SaftChargeOutDelay++;
        if (SaftChargeOutDelay >= COUNT_TO_1S_ON_100Khz) {
            sStateBit.SoftStart_bit = 0;
            SaftChargeOutDelay      = 0;
            PMOS_ON;
        }
    } else {
        SaftChargeOutDelay = 0;
    }
}

uint32_t CAN_ONline_Delay = 0;
/**
 * CAN离线保护环路
 * sStateBit.CAN_Offline_bit =1
 * 无法接收到电控的数据，进入待机。
 * 无法发送数据给电控。
 */
void CAN_Offline_Loop(void){
    TIM1_PWM2_BREAK;
    PMOS_Off();
    PID_Clear_Integral(&sPID_buck_V);
    PID_Clear_Integral(&sPID_buck_I);
    PID_Clear_Integral(&sPID_boost_V);
    PID_Clear_Integral(&sPID_boost_I);
    PID_Clear_Integral(&sPID_PowerLoop_Charge);
    PID_Clear_Integral(&sPID_PowerLoop_Discharge);

    // 这里判定是否接收到CAN数据。你问为什么要=1？
    // 因为执行这条函数之前在State_Change();中执行了一次CAN_WDG_Count++；
    if (CAN_WDG_Count == 1) {
        CAN_ONline_Delay++;
        // 接收到十次数据才会判定为CAN在线。
        if (CAN_ONline_Delay >= 10) {
            sStateBit.CAN_Offline_bit = 0;
            CAN_ONline_Delay          = 0;
        }
    } else if (CAN_WDG_Count > COUNT_TO_1S_ON_100Khz) {
        // 一秒没收到，就清除计数，不退出。
        CAN_ONline_Delay = 0;
    }
}

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
uint32_t OCP_RecoverDelay = 0;
/**
 * 过流保护
 * 状态标志位如下：
 * sStateBit.OCP_Bat_bit =1
 * 无视电控发过来的标志。
 * 发回去给电控的数据
 * sCAN_TX_data.SuperCapReady = 0
 * sCAN_TX_data.SuperCapState = 4
 */
void Over_Current_Protection_Loop(void){
    TIM1_PWM2_BREAK;
    PMOS_Off();
    PID_Clear_Integral(&sPID_buck_V);
    PID_Clear_Integral(&sPID_buck_I);
    PID_Clear_Integral(&sPID_boost_V);
    PID_Clear_Integral(&sPID_boost_I);
    PID_Clear_Integral(&sPID_PowerLoop_Charge);
    PID_Clear_Integral(&sPID_PowerLoop_Discharge);

    // 电流恢复正常之后5s解除保护
    if ((Ibat < (SOFTWARE_OCP_IBAT - 2)) && (Icap < (SOFTWARE_CHARGE_OCP_ICAP - 2)) && (Icap > (SOFTWARE_DISCHARGE_OCP_ICAP + 2))) {
        OCP_RecoverDelay++;
        if (OCP_RecoverDelay >= COUNT_TO_5S_ON_100Khz) {
            sStateBit.OCP_bit = 0;
            OCP_RecoverDelay  = 0;
            PMOS_ON;
        }
    } else {
        OCP_RecoverDelay = 0;
    }
}

uint32_t OVP_RecoverDelay = 0;
/**
 * 母线过压保护
 * 状态标志位如下：
 * sStateBit.OVP_Bat_bit =1
 * 无视电控发过来的标志。
 * 发回去给电控的数据
 * sCAN_TX_data.SuperCapReady = 0
 * sCAN_TX_data.SuperCapState = 5
 */
void Bat_Over_Voltage_Protection_Loop(void){
    TIM1_PWM2_BREAK;
    PMOS_Off();
    PID_Clear_Integral(&sPID_buck_V);
    PID_Clear_Integral(&sPID_buck_I);
    PID_Clear_Integral(&sPID_boost_V);
    PID_Clear_Integral(&sPID_boost_I);
    PID_Clear_Integral(&sPID_PowerLoop_Charge);
    PID_Clear_Integral(&sPID_PowerLoop_Discharge);

    if (Vbat < SOFTWARE_OVP_RECOVER_VBAT) {
        OVP_RecoverDelay++;

        // 电压恢复正常5s之后解除过压保护
        if (OVP_RecoverDelay >= COUNT_TO_5S_ON_100Khz) {
            sStateBit.OVP_Bat_bit = 0;
            OVP_RecoverDelay      = 0;
            PMOS_OffDelay         = 0;
            PMOS_ON;
        }
    } else {
        OVP_RecoverDelay = 0;
    }
}

/**
 * 电池欠压保护，进入此函数将无法退出，必须更换电池。
 * sStateBit.UVP_Cap_bit =1
 * 无视电控的发过来的标志。
 * 发回去给电控的数据
 * sCAN_TX_data.SuperCapReady = 0
 * sCAN_TX_data.SuperCapState = 6
 */
void Bat_Undervoltage_Protection_Loop(void){
    TIM1_PWM2_BREAK;
    PMOS_Off();
}

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
uint16_t UVP_CapRecover_CNT = 0;
/**
 * 超电过放保护环路
 * sStateBit.UVP_Cap_bit =1
 * 电控发过来的信息以下两个必须为：
 * sCAN_RX_data.Charge = 1
 * sCAN_RX_data.Enable = 1
 * 如果不给超电充电，则永远不会解除欠压保护。
 * 发回去给电控的数据
 * sCAN_TX_data.SuperCapReady = 0
 * sCAN_TX_data.SuperCapState = 7
 */
void CAP_Undervoltage_Protection_Loop(void){
    if (sCAN_RX_data.Enable && sCAN_RX_data.Charge) {
        // 进入欠压保护环路之后，需要等到电控允许充电才会给超级电容充电，否则则会一直保持保护状态，直到将电容组充到安全电压。

        // 使用最大充电电流算出一个最大充电功率，将最大充电功率限制在这个值之内。
        SafeChargePcap = Vcap * SAFE_CHARGE_ICAP;
        if (sCAN_RX_data.PowerLimitAfterOffset > SafeChargePcap) {
            P_ChargeCap = SafeChargePcap;
        } else {
            P_ChargeCap = sCAN_RX_data.PowerLimitAfterOffset;
        }

        Pcap = Vcap * Icap;
        PID_Preload_Integral(&sPID_PowerLoop_Charge, Vcap);
        PID_Compute(&sPID_PowerLoop_Charge, Pcap, P_ChargeCap, &PID_PowerLoopOut);

        PID_Preload_Integral(&sPID_buck_V, Vbat);
        PID_Compute(&sPID_buck_V, Vcap, SOFTWARE_OVP_VCAP, &PID_VoltageLoopOut);
        Loop_Competition_Buck(PID_PowerLoopOut, PID_VoltageLoopOut, &TIM1_PWM2_Comapre);

        // 半桥PWM比较值赋值，设定PWM占空比，这里是以上半桥的正占空比进行设置。
        htim1.Instance->CCR2 = TIM1_PWM2_Comapre;
        // ADC采样触发比较值赋值，设定为上半桥开通时间的3/8处
        htim1.Instance->CCR1 = (TIM1_PWM2_Comapre >> 3) + (TIM1_PWM2_Comapre >> 2);
    }

    if (Ibat < PMOS_OFF_CURRENT) {
        // 电容欠压充电的时候，只会有一种情况，电池电流会掉到0.5A以下：
        // 超级电容欠压充电的时候，突然死掉，底盘断电了
        // 所以当这种情况下，把半桥关掉，电容就达到了断电的目的
        PMOS_OffDelay++;
        if (PMOS_OffDelay < COUNT_TO_10MS_ON_100Khz) {
            // 这里前10MS还要打开是为了防止一些问题导致运行的时候管子和半桥没打开。
            // 比如第一次上电的时候，没电流，但是半桥又是关闭状态，就会导致充不进电。
            if (sCAN_RX_data.Enable && sCAN_RX_data.Charge) {
                // 只有【充电】【使能】同时为1时，才会打开半桥，防止充电的时候和底盘电机抢功率
                TIM1_PWM2_DISBREAK;
            } else {
                TIM1_PWM2_BREAK;
                // 清除充电环路的PI参数和状态，让他切换过去的时候会进行积分预加载。
                PID_Clear_Integral(&sPID_PowerLoop_Charge);
                PID_Clear_Integral(&sPID_buck_V);
                PID_Clear_Integral(&sPID_buck_I);
            }
            PMOS_ON;
        } else if (PMOS_OffDelay >= COUNT_TO_100MS_ON_100Khz) {
            // 迟滞100ms才会关闭PMOS，防止误关断导致PMOS反复开关
            // 充电的时候，半桥处于Buck模式，直接关闭PMOS不会有事
            PMOS_OFF;
        }
    } else {
        // 电流大于0.5A，要么是在移动，要么是在充电
        // 如果是在移动，就必须把PWM关掉，防止抢功率
        if (sCAN_RX_data.Enable && sCAN_RX_data.Charge) {
            // 只有【充电】【使能】同时为1时，才会打开半桥，防止充电的时候和底盘电机抢功率
            TIM1_PWM2_DISBREAK;
        } else {
            TIM1_PWM2_BREAK;
            // 清除充电环路的PI参数和状态，让他切换过去的时候会进行积分预加载。
            PID_Clear_Integral(&sPID_PowerLoop_Charge);
            PID_Clear_Integral(&sPID_buck_V);
            PID_Clear_Integral(&sPID_buck_I);
        }
        PMOS_ON;
        PMOS_OffDelay = 0;
    }

    // 清除放电环路的PI参数和状态，让他切换过去的时候会进行积分预加载。
    PID_Clear_Integral(&sPID_PowerLoop_Discharge);
    PID_Clear_Integral(&sPID_boost_V);
    PID_Clear_Integral(&sPID_boost_I);

    // 这个if是判断超电什么时候解除电容组UVP的
    // 每次进入UVP的时候，就说明电控把超电榨干了，需要给他充电到一定电压才能允许再次开启超电。
    Hysteresis_Comparator(Vcap, SAFE_CHARGE_VCAP, SAFE_CHARGE_VCAP - 2, &VcapUVPRecoverEdge);
    if (VcapUVPRecoverEdge == RISING && sCAN_RX_data.Charge) {
        // 与上  电控的【充电】是因为，欠压保护只会出现在电控一直放电榨干了超电。
        // 超级电容对欠压的处理是一种保护机制，会告诉电控已经保护了，
        // 只要超级电容处于保护状态，电控都应该将【充电】置1，
        // 若是【充电】一直是0，则认为电控 从要求超电放电开始-榨干超电-超电进入保护。这个过程中，电控一直强制要求超电放电，
        // 超电将会一直处于保护状态，直到【充电】置1，也就是电控不要求放电之后才会解除保护，允许下一次使用。
        UVP_CapRecover_CNT++;
        if (UVP_CapRecover_CNT >= COUNT_TO_10MS_ON_100Khz) {
            sStateBit.UVP_Cap_bit = 0;
            sStateBit.Charge_bit  = 1;
            sStateBit.Enable_bit  = 0;
            PMOS_ON;
        }
    } else if (VcapUVPRecoverEdge == FALLING || (!sCAN_RX_data.Charge)) {
        // 电容充电没达到安全电压，或者电控没有把【充电】置1，都不会退出欠压保护
        UVP_CapRecover_CNT = 0;
    }
}

uint32_t OTP_RecoverDelay = 0;
/**
 * 过温保护函数
 * sStateBit.UVP_Cap_bit =1
 * 无视电控发过来的标志。
 * 发回去给电控的数据
 * sCAN_TX_data.SuperCapReady = 0
 * sCAN_TX_data.SuperCapState = 8
 */
void Over_Temperature_Protect_Loop(void){
    TIM1_PWM2_BREAK;
    PMOS_Off();
    PID_Clear_Integral(&sPID_buck_V);
    PID_Clear_Integral(&sPID_buck_I);
    PID_Clear_Integral(&sPID_boost_V);
    PID_Clear_Integral(&sPID_boost_I);
    PID_Clear_Integral(&sPID_PowerLoop_Charge);
    PID_Clear_Integral(&sPID_PowerLoop_Discharge);

    if (Tmos < SOFTWARE_OTP_RECOVER_MOS && Tcap < SOFTWARE_OTP_RECOVER_CAP) {
        OTP_RecoverDelay++;

        // 温度降低5s之后解除过压保护
        if (OTP_RecoverDelay >= COUNT_TO_5S_ON_100Khz) {
            sStateBit.OTP_MOS_bit = 0;
            sStateBit.OTP_CAP_bit = 0;
            OTP_RecoverDelay      = 0;
            PMOS_OffDelay         = 0;
            PMOS_ON;
        }
    } else {
        OTP_RecoverDelay = 0;
    }
}

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
uint32_t TrickleChargeDelay = 0;
/**
 * 超电充电环路
 * 无任何保护状态的时候，才能进入充电环路，状态标志位如下：
 * sStateBit.Charge_bit =1
 * sStateBit.Enable_bit =1
 * 电控发过来的信息以下两个必须为：
 * sCAN_RX_data.Charge = 1
 * sCAN_RX_data.Enable = 1
 * 发回去给电控的数据
 * sCAN_TX_data.SuperCapReady = 1
 * sCAN_TX_data.SuperCapState = 1
 */
void Charge_Loop(void){
    // 使用安全充电电流算出一个最大充电功率，将最大充电功率限制在这个值之内。
    SafeChargePcap = Vcap * SAFE_CHARGE_ICAP;
    if (sCAN_RX_data.PowerLimitAfterOffset > SafeChargePcap) {
        P_ChargeCap = SafeChargePcap;
    } else {
        P_ChargeCap = sCAN_RX_data.PowerLimitAfterOffset;
    }

    Pcap = Vcap * Icap; // 当前电容功率
    PID_Preload_Integral(&sPID_PowerLoop_Charge, Vcap);
    PID_Compute(&sPID_PowerLoop_Charge, Pcap, P_ChargeCap, &PID_PowerLoopOut);

    PID_Preload_Integral(&sPID_buck_V, Vbat);
    PID_Compute(&sPID_buck_V, Vcap, SOFTWARE_OVP_VCAP, &PID_VoltageLoopOut);
    Loop_Competition_Buck(PID_PowerLoopOut, PID_VoltageLoopOut, &TIM1_PWM2_Comapre);

    // 半桥PWM比较值赋值，设定PWM占空比，这里是以上半桥的正占空比进行设置。
    htim1.Instance->CCR2 = TIM1_PWM2_Comapre;
    // ADC采样触发比较值赋值，设定为上半桥开通时间的3/8处
    htim1.Instance->CCR1 = (TIM1_PWM2_Comapre >> 3) + (TIM1_PWM2_Comapre >> 2);

    TIM1_PWM2_DISBREAK;
    //充电的时候就一直打开PWM，不需要关闭。

    // 充电的时候，只会有两种情况，电池电流会很小
    // 一是超级电容充满电了，处于涓流状态
    // 二是超级电容充电的时候，突然死掉，底盘断电了
    // 充满电的时候，电流很小，就算把PMOS关掉，使用体二极管进行涓流充电，也不会产生太大的损耗。
    Hysteresis_Comparator(Ibat ,PMOS_ON_CURRENT ,TRICKLE_CHARGE_CURRENT_CAP , &IbatPMOSOffEdge);
    if (IbatPMOSOffEdge == FALLING) {
        PMOS_OffDelay++;
        if (PMOS_OffDelay >= COUNT_TO_100MS_ON_100Khz) {
            // 迟滞100ms才会关闭PMOS，防止误关断导致PMOS反复开关
            // 充电的时候，半桥处于Buck模式，直接关闭PMOS不会有事
            PMOS_OFF;
        }
    }else {
        PMOS_OffDelay =0;
        //就算电流变大也不打开PMOS了，只有进去其他环路的时候才会打开PMOS。
    }

    // 清除放电环路的PI参数和状态，让他切换过去的时候会进行积分预加载。
    PID_Clear_Integral(&sPID_PowerLoop_Discharge);
    PID_Clear_Integral(&sPID_boost_V);
    PID_Clear_Integral(&sPID_boost_I);
}

uint32_t UVP_CapDelay = 0;
/**
 * 超电放电环路
 * 无任何保护状态的时候，才能进入放电环路，状态标志位如下：
 * sStateBit.Charge_bit =0
 * sStateBit.Enable_bit =1
 * 电控发过来的信息以下两个必须为：
 * sCAN_RX_data.Charge = 0
 * sCAN_RX_data.Enable = 1
 * 发回去给电控的数据
 * sCAN_TX_data.SuperCapReady = 1
 * sCAN_TX_data.SuperCapState = 0
 */
void Discharge_Loop(void){
    Pbat = Vbat * Ibat;

    // 这个PI运算是主要的放电运算
    PID_Preload_Integral(&sPID_PowerLoop_Discharge, Vcap);
    PID_Compute(&sPID_PowerLoop_Discharge, Pbat, sCAN_RX_data.PowerLimitAfterOffset, &PID_PowerLoopOut);

    // 这个PI运算是防止一直升压的过压保护使用
    PID_Preload_Integral(&sPID_boost_V, Vcap);
    PID_Compute(&sPID_boost_V, Vbat, SOFTWARE_OVP_VBAT, &PID_VoltageLoopOut);

    Loop_Competition_Boost(PID_PowerLoopOut, PID_VoltageLoopOut, &TIM1_PWM2_Comapre);

    // 这里是防止某些电控逻辑错误导致的超电过充保护。
    Hysteresis_Comparator(Vcap, SOFTWARE_OVP_VCAP + 1, SOFTWARE_OVP_VCAP - 1, &VcapDischargeOVPEgde);
    if (VcapDischargeOVPEgde == RISING) {

        PID_Preload_Integral(&sPID_buck_V, Vbat);
        PID_Compute(&sPID_buck_V, Vcap, SOFTWARE_OVP_VCAP, &PID_VoltageLoopOut);

        Loop_Competition_Buck(TIM1_PWM2_Comapre, PID_VoltageLoopOut, &TIM1_PWM2_Comapre);
    } else if (VcapDischargeOVPEgde == FALLING) {
        PID_Clear_Integral(&sPID_buck_V);
    }

    // 半桥PWM比较值赋值，设定PWM占空比，这里是以上半桥的正占空比进行设置。
    htim1.Instance->CCR2 = TIM1_PWM2_Comapre;
    // ADC采样触发比较值赋值，设定为上半桥开通时间的3/8处
    htim1.Instance->CCR1 = (TIM1_PWM2_Comapre >> 3) + (TIM1_PWM2_Comapre >> 2);

    // 这个if是判断超电什么时候用完电，进入过放保护的
    // 因为超电没电只存在两种情况，一种是第一次上电，另一种是放电的时候放完了
    // 前者会在上电的时候进行判断，这里是后者，只会在这里进行判断
    Hysteresis_Comparator(Vcap ,SOFTWARE_UVP_VCAP + 2 ,SOFTWARE_UVP_VCAP ,&VcapUVPEdge);
    if (VcapUVPEdge == FALLING) {
        UVP_CapDelay++;
        if (UVP_CapDelay >= COUNT_TO_100MS_ON_100Khz) {
            // 迟滞100ms才会确认超电已经欠压，防止瞬时电流引起的电压跌落导致误触发欠压保护
            sStateBit.Charge_bit  = 1;
            sStateBit.Enable_bit  = 0;
            sStateBit.UVP_Cap_bit = 1;
            UVP_CapDelay          = 0;
        }
    } else if(VcapUVPEdge == RISING) {
        // 不欠压就清零计数，这里是单次持续100ms才会保护，而不是多次累计达到100ms
        UVP_CapDelay = 0;
    }

    // 放电的时候，只会有一种情况，电池电流会掉到0.5A以下
    // 也就是：突然死掉，底盘断电了。
    // 所以当这情况下，把半桥关掉，就可以达到底盘断电的目的。
    if (Ibat < PMOS_OFF_CURRENT) {
        PMOS_OffDelay++;

        if (PMOS_OffDelay < COUNT_TO_10MS_ON_100Khz) {
            // 这里前10MS还要打开是为了防止一些逻辑问题导致运行的时候管子没打开。
            TIM1_PWM2_DISBREAK;
            PMOS_ON;
        } else if (PMOS_OffDelay > COUNT_TO_15MS_ON_100Khz) {
            // 只有放电的时候才需要关闭半桥，因为放电的时候处于Boost模式，电感续流需要通过PMOS
            TIM1_PWM2_BREAK;
        } else if (PMOS_OffDelay >= COUNT_TO_100MS_ON_100Khz) {
            // PMOS比半桥晚关的原因是，半桥关闭后可能会有电感续流
            //  如果直接断掉PMOS电感续流会无处可去，会产生一个电压尖峰，可能会导致管子击穿
            PMOS_OFF;
        }
    } else {
        TIM1_PWM2_DISBREAK;
        PMOS_ON;
        PMOS_OffDelay = 0;
    }

    // 清除充电环路的PI参数和状态，让他切换过去的时候会进行积分预加载。
    PID_Clear_Integral(&sPID_PowerLoop_Charge);
    PID_Clear_Integral(&sPID_buck_I);
}

/**
 * 待机模式
 * 关闭半桥，并且关闭PMOS，将超级电容对底盘功率消耗降到最低。
 * 进入待机状态一般是处于两种状态：
 * 超电充电，但是车子在移动，需要超电不进行充电，以免跟底盘抢功率
 * 超电放电，但是车子又不走，防止超电无脑将电池功率拉至功率限制，此时所有功率都会往超级电容走，可能会导致超级电容被充爆（不过写了保护，大概不会吧）
 * 电控发过来的标志：
 * sStateBit.Enable_bit =0
 * 发回去给电控的标志
 * sCAN_TX_data.SuperCapReady = 1
 * sCAN_TX_data.SuperCapState = 2
 */
void Wait_Loop(void){
    TIM1_PWM2_BREAK;
    PMOS_Off();
    PID_Clear_Integral(&sPID_buck_V);
    PID_Clear_Integral(&sPID_buck_I);
    PID_Clear_Integral(&sPID_boost_V);
    PID_Clear_Integral(&sPID_boost_I);
    PID_Clear_Integral(&sPID_PowerLoop_Charge);
    PID_Clear_Integral(&sPID_PowerLoop_Discharge);
}

/**
 * PMOS关闭函数，在除了充电和放电的时候使用，因为充电和放电的时候需要考虑其他因素，所以不能直接使用这条函数来关PMOS
 */
void PMOS_Off(void){
    // 因为PMOS导通可能比较慢，所以还是需要在死掉的时候才会关闭PMOS,防止没必要的时候关闭PMOS导致出问题

    Hysteresis_Comparator(Ibat, PMOS_ON_CURRENT, PMOS_OFF_CURRENT, &IbatPMOSOffEdge);
    if (IbatPMOSOffEdge == FALLING) {
        PMOS_OffDelay++;
        if (PMOS_OffDelay > COUNT_TO_100MS_ON_100Khz) {
            PMOS_OFF;
        }
    } else if (IbatPMOSOffEdge == RISING) {
        PMOS_ON;
        PMOS_OffDelay = 0;
    }
}

uint32_t LED_Refresh_CNT = 0; // 中间变量，用于同步灯光闪烁
/**
 * LED刷新函数，刷新频率大概为12Hz
 * 如果LED灯没亮，说明程序卡在初始化校准了
 * 但凡有一个LED灯在闪烁，都说明处于保护状态
 */
void LED_Refresh(void){
    LED_Refresh_CNT++;
    if (sStateBit.CAN_Offline_bit) {
        // CAN掉线的时候，两个灯同时闪烁
        if (LED_Refresh_CNT & 1) {
            LED_CAP_ON;
            LED_CHASSIS_ON;
        } else {
            LED_CAP_OFF;
            LED_CHASSIS_OFF;
        }
    } else if (sStateBit.OTP_CAP_bit || sStateBit.OTP_MOS_bit) {
        // 过温保护，两个灯交替闪烁
        if (LED_Refresh_CNT & 1) {
            LED_CAP_ON;
            LED_CHASSIS_OFF;
        } else {
            LED_CAP_OFF;
            LED_CHASSIS_ON;
        }
    } else if (sStateBit.UVP_Bat_bit) {
        // 电池没电的时候，超电关闭，两个灯全部熄灭
        LED_CAP_OFF;
        LED_CHASSIS_OFF;
    } else if (sStateBit.OCP_bit) {
        // 母线过流的时候，底盘灯闪烁
        // 电容灯亮
        LED_CAP_ON;
        LED_CHASSIS_BLINK;
    } else if (sStateBit.OVP_Bat_bit) {
        // 动能回收母线过压的时候底盘的灯闪烁
        // 电容灯灭
        LED_CAP_OFF;
        LED_CHASSIS_BLINK;
    } else if (sStateBit.UVP_Cap_bit) {
        // 电容用完电的时候，电容的灯闪烁
        // 底盘灯灭
        LED_CAP_BLINK;
        LED_CHASSIS_OFF;
    } else if (sStateBit.OVP_Cap_bit) {
        // 电容过充的时候，电容的灯闪烁
        // 底盘灯亮
        LED_CAP_BLINK;
        LED_CHASSIS_ON;
    }else if (sCAN_TX_data.SuperCapEnergy >= 95) {
        // 充满电的时候两个灯常亮
        LED_CAP_ON;
        LED_CHASSIS_ON;
    }else if (sCAN_TX_data.SuperCapReady) {
        if (sStateBit.Charge_bit) {
            // 充电的时候电容的灯常亮
            LED_CAP_ON;
            LED_CHASSIS_OFF;
        } else if (!sStateBit.Charge_bit) {
            // 放电的时候底盘的灯常亮
            LED_CAP_OFF;
            LED_CHASSIS_ON;
        }
    }
}

/**
 * 过滤器初始化函数
 */
void FDCAN_Filter_Init(void){
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;       // 标准CANID
    sFilterConfig.FilterIndex  = 0;                       // 过滤器序列0，只配置一个过滤器
    sFilterConfig.FilterType   = FDCAN_FILTER_DUAL;       // 双ID过滤模式，只能接受两个指定ID的数据
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 接收的数据保存到FIFO0
    sFilterConfig.FilterID1    = CAN_C_BOARD_ID;         // ID白名单1
    sFilterConfig.FilterID2    = CAN_TEST_ID;             // ID白名单2

    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
    // 全局过滤器配置，与过滤器不匹配的ID数据将会拒绝接收，不会进入中断。
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
}

/**
 * fdcan接收回调函数，定义了这条函数回调就会到这里执行，在HAL_FDCAN_IRQHandler函数中调用了。
 * 其实这条函数也就是把FDCAN的接收缓冲数组赋值给结构体而已，方便使用变量
 * 只有与FDCAN过滤器匹配的ID信息才会触发FDCAN接收中断，产生回调
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
    uint8_t CAN_RX_Buff[8];
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &sFDCAN1_RxHeader, CAN_RX_Buff); // 接收数据

    // 在这里直接把数据分配到结构体中，方便调用
    sCAN_RX_data.Enable      = (uint8_t)CAN_RX_Buff[0];
    sCAN_RX_data.Charge      = (uint8_t)CAN_RX_Buff[1];
    sCAN_RX_data.PowerLimint = (uint8_t)CAN_RX_Buff[2];

    // 将收到的电控的功率限制进行一个损耗补偿，避免与裁判系统检测出的功率不同而超功率。
    sCAN_RX_data.PowerLimitAfterOffset = sCAN_RX_data.PowerLimint - PBAT_POWER_LOSS;
    if (sCAN_RX_data.PowerLimitAfterOffset < 5) sCAN_RX_data.PowerLimitAfterOffset = 5;

    // CAN看门狗计数清零，确保CAN在线
    CAN_WDG_Count = 0;
}

uint8_t FirstSend = 1;
/**
 * FDCAN发送函数
 * TX_temp是相关发送数据的结构体，详见CAN_TransmitData
 */
void Can_SendMess(CAN_TransmitDataTypeDef *TX_temp){
    // 这些根据DJI的CAN协议来配。
    if (FirstSend) {
        sFDCAN1_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;     // 波特率切换关闭
        sFDCAN1_TxHeader.FDFormat      = FDCAN_CLASSIC_CAN; // 经典CAN
        sFDCAN1_TxHeader.Identifier    = CAN_SUPERCAP_ID;   // 发送ID
        sFDCAN1_TxHeader.IdType        = FDCAN_STANDARD_ID; // 标准CANID
        sFDCAN1_TxHeader.DataLength    = FDCAN_DLC_BYTES_8; // 数据长度
        sFDCAN1_TxHeader.TxFrameType   = FDCAN_DATA_FRAME;  // 数据类型
        FirstSend                      = 0;
    }

    uint8_t CAN_TX_BUFF[8] = {0};
    CAN_TX_BUFF[0]         = TX_temp->SuperCapReady;
    CAN_TX_BUFF[1]         = TX_temp->SuperCapState;
    CAN_TX_BUFF[2]         = TX_temp->SuperCapEnergy;
    CAN_TX_BUFF[3]         = TX_temp->ChassisPower;
    CAN_TX_BUFF[4]         = TX_temp->VoltageBat;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &sFDCAN1_TxHeader, CAN_TX_BUFF);
}

/**
 * 这个函数将带有96位UID的结构体数组与STM32出厂自带的唯一UID进行对比
 * 如果UID匹配则使用对应的ADC拟合参数对ADC采样值进行转换。
 */
void ADC_Curve_Fitting(void){
    LED_CHASSIS_OFF;
    LED_CAP_OFF;
    while(1){
        for (uint8_t i = 0; i < 10; i++) {
            if (!memcmp((const void *)UID_BASE, &csADC_Fit_Parameters[i], 12)) {
                sADC_Fit = csADC_Fit_Parameters[i];
                return;
            }
        }
    }
}


uint32_t ADC_SUM_CNT = 0;//中间计数变量
/**
 * 这条函数用来对ADC数值进行累加，进行均值滤波
 * 计算出来的均值用于数据监控，不参与环路计算，不需要很高的运算频率，可以取更多的点进行平均
 * Times ：均值滤波点数，只能是2的幂次。如果这条函数每周期都运行一次，可以直接传入DIVxxx的分频系数。
 * 均值滤波之后，数据会变得更平滑，但是将会引入额外的延迟，所以均值滤波之后的数据仅用作电控回传，而不参与环路运算。
 */
void ADC_Value_AVG_Compute(uint32_t Times){
    ADC_SUM_CNT++;

    sADC_Value_SUM.Ibat += (uint32_t)ADC1Value[0];
    sADC_Value_SUM.Icap += (uint32_t)ADC1Value[1];
    sADC_Value_SUM.Tcap += (uint32_t)ADC1Value[2];

    sADC_Value_SUM.Vbat += (uint32_t)ADC2Value[0];
    sADC_Value_SUM.Vcap += (uint32_t)ADC2Value[1];
    sADC_Value_SUM.Tmos += (uint32_t)ADC2Value[2];

    if ((ADC_SUM_CNT & Times) == 0) {
        uint8_t AVG_Times = 0;
        switch (Times) {
            case DIV2:AVG_Times = 1;break;
            case DIV4:AVG_Times = 2;break;
            case DIV8:AVG_Times = 3;break;
            case DIV16:AVG_Times = 4;break;
            case DIV32:AVG_Times = 5;break;
            case DIV64:AVG_Times = 6;break;
            case DIV128:AVG_Times = 7;break;
            case DIV256:AVG_Times = 8;break;
            case DIV512:AVG_Times = 9;break;
            case DIV1024:AVG_Times = 10;break;
            case DIV2048:AVG_Times = 11;break;
            case DIV4096:AVG_Times = 12;break;
            case DIV8192:AVG_Times = 13;break;
        }

        sADC_Value_AVG.Ibat = sADC_Value_SUM.Ibat >> AVG_Times;
        sADC_Value_AVG.Icap = sADC_Value_SUM.Icap >> AVG_Times;
        sADC_Value_AVG.Vbat = sADC_Value_SUM.Vbat >> AVG_Times;
        sADC_Value_AVG.Vcap = sADC_Value_SUM.Vcap >> AVG_Times;
        sADC_Value_AVG.Tcap = sADC_Value_SUM.Tcap >> AVG_Times;
        sADC_Value_AVG.Tmos = sADC_Value_SUM.Tmos >> AVG_Times;

        sADC_Value_SUM.Ibat = 0;
        sADC_Value_SUM.Icap = 0;
        sADC_Value_SUM.Vbat = 0;
        sADC_Value_SUM.Vcap = 0;
        sADC_Value_SUM.Tcap = 0;
        sADC_Value_SUM.Tmos = 0;
    }
}
