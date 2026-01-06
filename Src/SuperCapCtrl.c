/**********************************************************************************************
 * 超级电容功率控制库：V1.2.2519
 * 代码建立日期：2024年4月13日
 * 最后修改日期：2025年5月6日
 * 编码格式：GB2312
 * CubeMX版本：6.12.0
 * STM32G4固件包版本：1.6.1
 * 本代码为RM2025桂林理工大学群星战队开源的超级电容控制板Lite & Plus的完整代码
 * 作者：某桂工的魔法电容使者
 * 
 * V1.2版本的代码通过修改宏定义，兼容Lite和Plus控制板。
 * 宏定义在LITE或者PLUS的相对应的CubeMX生成的初始化main.h中定义了，请勿自行定义，两个版本的初始化配置完全不同。
 * 
 * 需要注意的是，本控制库只是关于运行逻辑方面的代码，具体使用Lite或者Plus版本控制板还需要搭配其相应的初始化工程。
 * 
 * 本代码使用MIT协议进行授权。
 **********************************************************************************************/

#include "SuperCapCtrl.h"
#include "tim.h"
#include "fdcan.h"
#include "string.h" //给memcmp用的
#include "iwdg.h"
#include "adc.h"
#include "math.h" //给NTC拟合用的

#ifdef PLUS
#include "usart.h"
#include "stdio.h"

#endif

static PID_ParameterTypeDef sPID_buck_V                    = {0};
static PID_ParameterTypeDef sPID_buck_I                    = {0};
static PID_ParameterTypeDef sPID_buck_I_UVP                = {0};
static PID_ParameterTypeDef sPID_boost_V                   = {0};
static PID_ParameterTypeDef sPID_AutomaticCompensation     = {0};

static ADC_ValueTypeDef sADC_Value_SUM = {0};
static ADC_ValueTypeDef sADC_Value_AVG = {0};

static FDCAN_FilterTypeDef sFilterConfig;
static FDCAN_RxHeaderTypeDef sFDCAN1_RxHeader;
static FDCAN_TxHeaderTypeDef sFDCAN1_TxHeader;

static JustFloatFrameTypedef sUART_TxData = {0};

static CAN_TransmitDataTypeDef sCAN_TX_data = {0};
static CAN_ReceiveDataTypeDef sCAN_RX_data  = {0};

static StateFlagsTypeDef sStateBit = {0};

static AnomalyDetectionTypeDef sOCPbatConfig = {0};
static AnomalyDetectionTypeDef sOVPbatConfig = {0};
static AnomalyDetectionTypeDef sUVPbatConfig = {0};

static AnomalyDetectionTypeDef sOCPcapConfig = {0};
static AnomalyDetectionTypeDef sUVPcapConfig = {0};

static AnomalyDetectionTypeDef sOTPmosConfig = {0};
static AnomalyDetectionTypeDef sOTPcapConfig = {0};

// 滞回比较器的状态存储变量
static ComparatorStateTypeDef VcapUVPEdge = RISING;
static ComparatorStateTypeDef PbatOPLEdge = FALLING;    //超电没电之后，超级电容无法补偿超出功率的时候，认为超电没电了

static ComparatorStateTypeDef IcapAutoLoopEgde = FALLING;
static ComparatorStateTypeDef VcapAutoLoopEgde = FALLING;
static ComparatorStateTypeDef VbatAutoLoopEgde = FALLING;

static ComparatorStateTypeDef IcapUvpLoopDownEgde = RISING;
static ComparatorStateTypeDef IcapUvpLoopUpEgde = FALLING;
static ComparatorStateTypeDef VcapUvpLoopEgde = FALLING;

static ComparatorStateTypeDef VcapUVPRecoverEdge = FALLING;
static ComparatorStateTypeDef PowerdownEdge    = RISING;

static const ADC_Fit_ParametersTypeDef csADC_Fit_Parameters[50] = {
    // UID0     ,UID1      ,UID2       ,Vbat_A  ,Vbat_B  ,Vcap_A  ,Vcap_B  ,Ibat_A  ,Ibat_B  ,Icap_A  ,Icap_B
    {0x003B0057, 0x54315005, 0x20333830, 0.0094f, 0.0680f, 0.0068f, 0.0079f, 0.0034f, 0.0237f, 0.0175f, -47.279f},//Lite
    {0x003C0022, 0x54315005, 0x20333830, 0.0094f, 0.0175f, 0.0068f, 0.0460f, 0.0036f, 0.0360f, 0.0123f, -33.496f},//Lite
    {0x003B006D, 0x54315005, 0x20333830, 0.0095f, 0.0173f, 0.0068f, 0.0461f, 0.0036f, 0.0360f, 0.0123f, -33.496f},//Lite
    {0x003D0022, 0x54315005, 0x20333830, 0.0094f, 0.0491f, 0.0068f, 0.0392f, 0.0034f, -0.0283f, 0.0124f, -32.954f},//Lite
    {0x004F0051, 0x484E5006, 0x20353031, 0.0096f, 0.0732f, 0.0068f, 0.0501f, 0.0040f, 0.1937f, 0.0123f, -33.372f},//Lite
    {0x00230060, 0x484E5004, 0x20353031, 0.0095f, 0.0349f, 0.0068f, 0.0595f, 0.0042f, 0.1252f, 0.0129f, -34.796f},//Lite
    {0x00250045, 0x484E5004, 0x20353031, 0.0095f, 0.0793f, 0.0068f, 0.0441f, 0.0040f, 0.1331f, 0.0129f, -34.918f},//Lite
    {0x001B0051, 0x484E5004, 0x20353031, 0.0095f, 0.0476f, 0.0067f, 0.0569f, 0.0041f, 0.1250f, 0.0128f, -34.728f},//Lite
    {0x001e0051, 0x534b5016, 0x20343932, 0.0087f, 0.0392f, 0.0058f, 0.0339f, 0.0035f, 0.2045f, 0.0133f, -35.374f},//Lite
    {0x00450053, 0x534B5016, 0x20343932, 0.0099f, 0.0697f, 0.0099f, 0.0698f, 0.0077f, -0.0124f, -0.0167f, 23.121f},//Plus
    {0x002F0052, 0x534B5016, 0x20343932, 0.0100f, 0.0895f, 0.0100f, 0.0657f, 0.0080f, -0.0057f, -0.0157f, 21.968f},//Plus
    {0x003A0021, 0x54315005, 0x20333830, 0.0099f, 0.1116f, 0.0099f, 0.0959f, 0.0080f, 0.0130f, -0.0157f, 22.019f},//Plus
    {0x0059004F, 0x534B5016, 0x20343932, 0.0100f, 0.0937f, 0.0099f, 0.0484f, 0.0081f, 0.0391f, -0.0159f, 21.705f},//Plus
    {0x005D0050, 0x534B5016, 0x20343932, 0.0100f, 0.0937f, 0.0099f, 0.0484f, 0.0081f, 0.0391f, -0.0159f, 21.705f},//Plus
    {0x004A0053, 0x534B5016, 0x20343932, 0.0099f, 0.1116f, 0.0099f, 0.0959f, 0.0080f, 0.0130f, -0.0157f, 22.019f},//Plus
    {0x003F0052, 0x534B5016, 0x20343932, 0.0099f, 0.1146f, 0.0100f, 0.0813f, 0.0078f, 0.0019f, -0.0158f, 21.98f},//Plus
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
static inline void Boom_Protection_Loop(void);
static inline void Bat_Undervoltage_Protection_Loop(void);
static inline void CAP_Undervoltage_Protection_Loop(void);
static inline void Automatic_Power_Compensation_Loop(void);
static inline void Power_Down(void);
static inline void Wait_Loop(void);

static inline uint8_t Anomaly_Detection_Init(AnomalyDetectionTypeDef *pAnomalyDetection);
static inline void Anomaly_Detection(AnomalyDetectionTypeDef *pAnomalyDetection, float detected_input ,uint8_t *state_output);

static inline void DebugOut_UART(void);

static inline uint8_t SuperCap_Disable(void);
static inline uint8_t SuperCap_Enable(void);

static inline void Can_SendMess(CAN_TransmitDataTypeDef *TX_temp);
static inline void ADC_Value_AVG_Compute(uint32_t Times);
static inline void LED_Refresh(void);
static inline void Hysteresis_Comparator(float input, float rising_threshold, float falling_threshold, ComparatorStateTypeDef *state);

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/

uint16_t ADC1Value[3] = {0};
uint16_t ADC2Value[3] = {0};

static uint32_t CAN_WDG_Counter = 0; // CAN离线检测看门狗计数
static uint8_t  CAN_ReceiveDataRefresh_Flag =1; // CAN接收数据刷新标志，用于限制接收到的CAN数据的刷新速率

static uint32_t PID_PowerLoopOut   = 0; // 中间量，功率环PID输出
static uint32_t PID_CurrentLoopOut = 0; // 中间量，电流环PID输出
static uint32_t PID_VoltageLoopOut = 0; // 中间量，电压环PID输出
static uint32_t TIM1_PWM2_Comapre  = 0; // 最后赋值给PWM比较值的量，决定占空比输出

static float Vbat; //电池电压、底盘供电电压
static float Vcap; //超级电容组电压
static float Tcap; //超级电容组温度

static float Ibat; //电池供电电流（裁判系统Chassis口电流）【功率限制】
static float Icap; //超级电容组电流（充电为正，放电为负）
static float Tmos; //超级电容控制板半桥温度

static float SafeChargePcap = 0; // 额定电容充电功率，使用Vcap*Icap计算，用于限制电容的充电功率

static float Pcap        = 0; // 环路计算中间量，电容端口功率
static float Pbat        = 0; // 环路计算中间量，电池端口功率
static float P_ChargeCap = 0; // 电容充电时候设定的功率


static uint16_t BatPower_AVG;
static uint16_t ChassisPower_AVG;
static int SuperCapPower_AVG;
static uint8_t SuperCapEnergy_AVG;

#ifdef PLUS
//因为Plus使用的是注入组ADC，所以需要调用的是注入中断
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc){
    TEST_OUT_HIGH;

    ADC2Value[0] = hadc2.Instance->JDR1;
    ADC2Value[1] = hadc2.Instance->JDR2;

    ADC1Value[0] = hadc1.Instance->JDR1;
    ADC1Value[1] = hadc1.Instance->JDR2;

    A_Timing_Ranking_Idea();

    TEST_OUT_LOW;

}
#endif

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/

uint32_t TimingCNT = 0; // 时间片计数，用于区分不同时间节点运行的不同程序。
// ATRI算法，名为一个时间片算法，就是硬凑
// 在周期中断中调用保证其定时运行
inline void A_Timing_Ranking_Idea(void){
    
    TimingCNT++;

    ADC_Convert_To_Reality();

    State_Change();
    Power_Loop();
    

    ADC_Value_AVG_Compute(DIV512); // 运行频率：100K/512 = 200hz
    if ((TimingCNT & DIV512) == 1) Power_Calculations();        // 运行频率：100K/512 = 200hz
    if ((TimingCNT & DIV512) == 2) Can_SendMess(&sCAN_TX_data); // 运行频率：100K/512 = 200hz
    // 计算平均值、功率计算、CAN发送。数据更新将以最慢的那个为准。
    //这
    
    if ((TimingCNT & DIV1024) == 5) CAN_ReceiveDataRefresh_Flag = 1;   // 运行频率：100K/1024 = 98hz
    //限制超级电容控制板从CAN中

    if ((TimingCNT & DIV4096) == 5) HAL_IWDG_Refresh(&hiwdg);  // 运行频率：100K/4096 = 24hz
    if ((TimingCNT & DIV8192) == 10) LED_Refresh();             // 运行频率：100K/8192 = 12hz
}


//这条函数会被放到while1里空闲运行，所以没有实时性要求的东西比如温度检测，串口数据回传等处理放到这里就行
inline void Free_Loop(void){

#ifdef PLUS
    HAL_ADC_Start_DMA(&hadc1,(uint32_t *)(ADC2Value + 2),1);
    HAL_ADC_Start_DMA(&hadc2,(uint32_t *)(ADC1Value + 2),1);

    DebugOut_UART();
#endif
    
    Temperature_Calculations();
    HAL_Delay(5);

}




/*
------------------------------------------------------------------------------------------------------------------------------------------
*/

uint32_t BOOM_counter = 0; // 保护状态计数器，防止误保护
/**
 * 这个函数是用来改变状态标志位的，程序需要改变当前的运行状态均需要通过这个函数来判断。
 * 需要注意的是，有一些保护状态，一旦进入，这条函数就不能改变他它的运行状态了，只能依靠保护函数对应的逻辑来退出。
 */
void State_Change(void){
    // 超电的充电与放电，使能直接由电控来决定。但是超电的保护优先级永远高于电控的指令。
    sStateBit.Enable_bit = sCAN_RX_data.Enable;

    Anomaly_Detection(&sOCPbatConfig, Ibat, &sStateBit.OCP_bit);
    Anomaly_Detection(&sOCPcapConfig, Icap, &sStateBit.OCP_bit);

    Anomaly_Detection(&sOVPbatConfig, Vbat, &sStateBit.OVP_Bat_bit);
    Anomaly_Detection(&sUVPbatConfig, Vbat, &sStateBit.UVP_Bat_bit);

    Anomaly_Detection(&sOTPmosConfig, Tmos, &sStateBit.OTP_MOS_bit);
    Anomaly_Detection(&sOTPcapConfig, Tcap, &sStateBit.OTP_CAP_bit);

    if(BOOM){
        BOOM_counter ++;
        if(BOOM_counter > COUNT_TO_5S_ON_100Khz){
            BOOM_counter =0;
            sStateBit.BOOM_bit = 1;
        }
    }else {
        BOOM_counter = 0;
    }

    // 判断CAN是否离线。
    CAN_WDG_Counter++;
    // 如果持续1s没有收到CAN消息（看门狗计数没被清零），则判定CAN离线
    if (CAN_WDG_Counter > COUNT_TO_1S_ON_100Khz) sStateBit.CAN_Offline_bit = 1;
}

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
/**这里是整个电源控制环路
 * 这里使用if是因为写起来比较方便，同时可以拥有优先级判断，能够保证所有保护均会生效。
 */
void Power_Loop(){
    if(sStateBit.BOOM_bit){
        sCAN_TX_data.SuperCapReady = UNREADY;
        sCAN_TX_data.SuperCapState = BOOM_PROTECTION;
        Boom_Protection_Loop();
    }else if (sStateBit.UVP_Bat_bit) {
        // 电池欠压保护，一旦进入，就必须换电池，重新上电
        sCAN_TX_data.SuperCapReady = UNREADY;
        sCAN_TX_data.SuperCapState = UVP_BAT_PROTECTION;
        Bat_Undervoltage_Protection_Loop();
    } else if (sStateBit.OCP_bit) {
        // 过流保护，也是短路保护
        // 过流不会CC，只会直接关断。
        sCAN_TX_data.SuperCapReady = UNREADY;
        sCAN_TX_data.SuperCapState = OVER_LOAD_PROTECTTION;
        Over_Current_Protection_Loop();
    } else if (sStateBit.OVP_Bat_bit) {
        // 母线过压保护
        sCAN_TX_data.SuperCapReady = UNREADY;
        sCAN_TX_data.SuperCapState = OVP_BAT_PROTECTION;
        Bat_Over_Voltage_Protection_Loop();
    } else if (sStateBit.OTP_CAP_bit || sStateBit.OTP_MOS_bit) {
        // 过温保护
        sCAN_TX_data.SuperCapReady = UNREADY;
        sCAN_TX_data.SuperCapState = OTP_PROTECTION;
        Over_Temperature_Protect_Loop();
    } else if (sStateBit.CAN_Offline_bit) {
        // CAN离线
        sCAN_TX_data.SuperCapReady = UNREADY;
        sCAN_TX_data.SuperCapState = CAN_OFFLINE;
        CAN_Offline_Loop();
    } else if (sStateBit.SoftStart_bit) {
        // 软起动
        sCAN_TX_data.SuperCapReady = UNREADY;
        sCAN_TX_data.SuperCapState = SOFRSTART_PROTECTION;
        Soft_Start_Loop();
    } else if (sStateBit.UVP_Cap_bit) {
        //  超级电容低电压保护，设定值为电容组额定电压的30%
        sCAN_TX_data.SuperCapReady = UNREADY;
        sCAN_TX_data.SuperCapState = UVP_CAP_PROTECTION;
        CAP_Undervoltage_Protection_Loop();
    } else if (sStateBit.Enable_bit == ENABLE) {
        // 【使能】：
        sCAN_TX_data.SuperCapReady = READY;
        if(Icap > 0){
            sCAN_TX_data.SuperCapState = CHARGE;
        }else {
            sCAN_TX_data.SuperCapState = DISCHARGE;
        }
        Automatic_Power_Compensation_Loop();
    } else if (sStateBit.Enable_bit == DISABLE) {
        // 【失能】：
        sCAN_TX_data.SuperCapReady = READY;
        sCAN_TX_data.SuperCapState = WAIT;
        Wait_Loop();
    }
}

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
static uint32_t SoftStartOutDelay = 0; // 退出软启动计数
/**
 * 缓启动保护，每次上电必须执行。
 * sStateBit.SoftStart_bit =1
 * 电控发过来的信息以下两个必须为：
 * sCAN_RX_data.Enable = 1
 * 发回去给电控的数据
 * sCAN_TX_data.SuperCapReady = 0
 * sCAN_TX_data.SuperCapState = 3
 */
void Soft_Start_Loop(void){
    // 软起动的时候，PMOS是处于关闭状态的，但是不影响充电，因为功率比较小，不会导致PMOS烧毁
    if (sCAN_RX_data.Enable == ENABLE) {
        PID_Preload_Integral(&sPID_buck_I, Vcap);
        PID_Compute(&sPID_buck_I, Icap, SOFTSTART_CHARGE_ICAP, &PID_CurrentLoopOut);

        PID_Preload_Integral(&sPID_buck_V, Vcap);
        PID_Compute(&sPID_buck_V, Vcap, SOFTWARE_OVP_VCAP, &PID_VoltageLoopOut);

        Loop_Competition_Buck(PID_CurrentLoopOut, PID_VoltageLoopOut, &TIM1_PWM2_Comapre);

        // 半桥PWM比较值赋值，设定PWM占空比，这里是以上半桥的正占空比进行设置。
        SET_PWM_COMPARE(TIM1_PWM2_Comapre);

        // ADC采样触发比较值赋值
        SET_ADC_TRIGGER_COMPARE(TIM1_PWM2_Comapre);


        // 需要更新PMW比较值之后才能输出PWM，防止PWM比较值初始值与目标值差异过大而导致爆炸。
        //如果PWM处于刹车状态，则PID保持初始状态，避免计算错误导致
        if(!TIM1_PWM2_IS_OUT){
            PID_Clear_Integral(&sPID_boost_V);
            PID_Clear_Integral(&sPID_buck_V);
            PID_Clear_Integral(&sPID_buck_I);
            PID_Clear_Integral(&sPID_buck_I_UVP);
            PID_Clear_Integral(&sPID_AutomaticCompensation);
        }

        SuperCap_Enable();
    } else {
        SuperCap_Disable();
        PID_Clear_Integral(&sPID_boost_V);
        PID_Clear_Integral(&sPID_buck_V);
        PID_Clear_Integral(&sPID_buck_I);
        PID_Clear_Integral(&sPID_buck_I_UVP);
        PID_Clear_Integral(&sPID_AutomaticCompensation);
    }

    // 第一次上电的时候需要充电到5V以上才会退出缓启动，然后进入欠压保护，如果此时电压大于10V，则直接可以运行。
    if (Vcap > SAFE_CHARGE_VCAP){
        SoftStartOutDelay++;
        if (SoftStartOutDelay >= COUNT_TO_1S_ON_100Khz) {
            sStateBit.SoftStart_bit = 0;
            sStateBit.UVP_Cap_bit   = 0;
            SoftStartOutDelay      = 0;
        }
    }else if (Vcap > SOFTSTART_CHARGE_VCAP) {
        SoftStartOutDelay++;
        if (SoftStartOutDelay >= COUNT_TO_1S_ON_100Khz) {
            sStateBit.SoftStart_bit = 0;
            sStateBit.UVP_Cap_bit   = 1;
            SoftStartOutDelay      = 0;
        }
    } else {
        SoftStartOutDelay = 0;
    }
}

uint32_t CAN_ONline_Delay = 0;
/**
 * CAN离线保护环路
 * sStateBit.CAN_Offline_bit =1
 * 无法接收到电控的数据，进入待机。
 */
void CAN_Offline_Loop(void){
    SuperCap_Disable();
    PID_Clear_Integral(&sPID_buck_V);
    PID_Clear_Integral(&sPID_buck_I);
    PID_Clear_Integral(&sPID_buck_I_UVP);
    PID_Clear_Integral(&sPID_boost_V);
    PID_Clear_Integral(&sPID_AutomaticCompensation);

    // 这里判定是否接收到CAN数据。你问为什么要=1？
    // 因为执行这条函数之前在State_Change();中执行了一次CAN_WDG_Count++；
    if (CAN_WDG_Counter == 1) {
        CAN_ONline_Delay++;
        // 接收到十次数据才会判定为CAN在线。
        if (CAN_ONline_Delay >= 10) {
            sStateBit.CAN_Offline_bit = 0;
            CAN_ONline_Delay          = 0;
        }
    } else if (CAN_WDG_Counter > COUNT_TO_1S_ON_100Khz) {
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
    SuperCap_Disable();
    PID_Clear_Integral(&sPID_buck_V);
    PID_Clear_Integral(&sPID_buck_I);
    PID_Clear_Integral(&sPID_buck_I_UVP);
    PID_Clear_Integral(&sPID_boost_V);
    PID_Clear_Integral(&sPID_AutomaticCompensation);

    // 电流恢复正常之后5s解除保护
    if ((Ibat < (SOFTWARE_OCP_IBAT - 2)) && (Icap < (SOFTWARE_CHARGE_OCP_ICAP - 2)) && (Icap > (SOFTWARE_DISCHARGE_OCP_ICAP + 2))) {
        OCP_RecoverDelay++;
        if (OCP_RecoverDelay >= COUNT_TO_5S_ON_100Khz) {
            sStateBit.OCP_bit = 0;
            OCP_RecoverDelay  = 0;
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
    SuperCap_Disable();
    PID_Clear_Integral(&sPID_buck_V);
    PID_Clear_Integral(&sPID_buck_I);
    PID_Clear_Integral(&sPID_buck_I_UVP);
    PID_Clear_Integral(&sPID_boost_V);
    PID_Clear_Integral(&sPID_AutomaticCompensation);

    if (Vbat < SOFTWARE_OVP_RECOVER_VBAT) {
        OVP_RecoverDelay++;

        // 电压恢复正常5s之后解除过压保护
        if (OVP_RecoverDelay >= COUNT_TO_5S_ON_100Khz) {
            sStateBit.OVP_Bat_bit = 0;
            OVP_RecoverDelay      = 0;
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
    SuperCap_Disable();

}

void Boom_Protection_Loop(void){
    SuperCap_Disable();
}
/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
uint32_t UVP_CapRecover_CNT = 0;
/**
 * 超电过放保护环路
 * sStateBit.UVP_Cap_bit =1
 * 电控发过来的信息必须为：
 * sCAN_RX_data.Enable = 1
 * 发回去给电控的数据
 * sCAN_TX_data.SuperCapReady = 0
 * sCAN_TX_data.SuperCapState = 7
 */
void CAP_Undervoltage_Protection_Loop(void){
    // 超级电容对欠压的处理是一种保护机制，会告诉电控超级电容无法继续释放能量了。
    if (sCAN_RX_data.Enable == ENABLE) {
        // 进入欠压保护环路之后，需要等到电控允许充电才会给超级电容充电，否则则会一直保持保护状态，直到将电容组充到安全电压。

        Pbat = Vbat * Ibat;
        PID_Preload_Integral(&sPID_AutomaticCompensation, Vcap);
        PID_Compute(&sPID_AutomaticCompensation, Pbat, sCAN_RX_data.PowerLimint, &PID_PowerLoopOut);
        TIM1_PWM2_Comapre = PID_PowerLoopOut;

        // 这里是限制最大充电电流的PI运算，进入电流窗口后才会运行，减少非必要计算
        Hysteresis_Comparator(Icap, SAFE_CHARGE_ICAP, SAFE_CHARGE_ICAP - 4, &IcapUvpLoopUpEgde);
        if (IcapUvpLoopUpEgde == RISING) {
            PID_Preload_Integral(&sPID_buck_I, Vcap);
            PID_Compute(&sPID_buck_I, Icap, SAFE_CHARGE_ICAP, &PID_CurrentLoopOut);

            Loop_Competition_Buck(TIM1_PWM2_Comapre, PID_CurrentLoopOut, &TIM1_PWM2_Comapre);
        } else if (IcapUvpLoopUpEgde == FALLING) {
            PID_Clear_Integral(&sPID_buck_I);
            TIM1_PWM2_Comapre = PID_PowerLoopOut;
        }

        // 这里由于AutomaticCompensation会在底盘超功率的时候自动补偿，但是现在已经处于欠压状态之后无法补偿了，所以需要避免他产生反向电流（负电流）
        Hysteresis_Comparator(Icap, 5, 0, &IcapUvpLoopDownEgde);
        if (IcapUvpLoopDownEgde == FALLING) {
            PID_Preload_Integral(&sPID_buck_I_UVP, Vcap);
            PID_Compute(&sPID_buck_I_UVP, Icap, 0, &PID_CurrentLoopOut);

            Loop_Competition_Boost(TIM1_PWM2_Comapre, PID_CurrentLoopOut, &TIM1_PWM2_Comapre);
        } else if (IcapUvpLoopDownEgde == RISING) {
            PID_Clear_Integral(&sPID_buck_I_UVP);
            TIM1_PWM2_Comapre = PID_PowerLoopOut;
        }

        // 这里是超电恒压充电的PI计算，进入电压窗口后才会运行，减少非必要计算
        Hysteresis_Comparator(Vcap, SOFTWARE_OVP_VCAP, SOFTWARE_OVP_RECOVER_VCAP, &VcapUvpLoopEgde);
        if (VcapUvpLoopEgde == RISING) {
            PID_Preload_Integral(&sPID_buck_V, Vcap);
            PID_Compute(&sPID_buck_V, Vcap, SOFTWARE_OVP_VCAP, &PID_VoltageLoopOut);

            Loop_Competition_Buck(TIM1_PWM2_Comapre, PID_VoltageLoopOut, &TIM1_PWM2_Comapre);
        } else if (VcapUvpLoopEgde == FALLING) {
            PID_Clear_Integral(&sPID_buck_V);
            TIM1_PWM2_Comapre = PID_PowerLoopOut;
        }

        // 半桥PWM比较值赋值，设定PWM占空比，这里是以上半桥的正占空比进行设置。
        SET_PWM_COMPARE(TIM1_PWM2_Comapre);

        // ADC采样触发比较值赋值
        SET_ADC_TRIGGER_COMPARE(TIM1_PWM2_Comapre);


        // 电容欠压充电的时候，只会有一种情况，电池电流会掉到0.5A以下：
        // 超级电容欠压充电的时候，突然死掉，底盘断电了
        // 所以当这种情况下，把半桥关掉，电容就达到了断电的目的
        Power_Down();

    }else {
        SuperCap_Disable();
        PID_Clear_Integral(&sPID_boost_V);
        PID_Clear_Integral(&sPID_buck_V);
        PID_Clear_Integral(&sPID_buck_I);
        PID_Clear_Integral(&sPID_buck_I_UVP);
        PID_Clear_Integral(&sPID_AutomaticCompensation);
    }

    // 清除放电环路的PI参数和状态，让他切换过去的时候会进行积分预加载。
    PID_Clear_Integral(&sPID_boost_V);

    // 这个if是判断超电什么时候解除电容组UVP的
    // 每次进入UVP的时候，就说明电控把超电榨干了，需要给他充电到一定电压才能允许再次开启超电。
    Hysteresis_Comparator(Vcap, SAFE_CHARGE_VCAP, SAFE_CHARGE_VCAP - 2, &VcapUVPRecoverEdge);
    if (VcapUVPRecoverEdge == RISING) {
        UVP_CapRecover_CNT++;
        if (UVP_CapRecover_CNT >= COUNT_TO_2S_ON_100Khz) {
            sStateBit.UVP_Cap_bit = 0;
            sStateBit.Enable_bit  = 0;
        }
    } else{
        // 电容充电没达到安全电压，都不会退出欠压保护
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
    SuperCap_Disable();
    PID_Clear_Integral(&sPID_buck_V);
    PID_Clear_Integral(&sPID_buck_I);
    PID_Clear_Integral(&sPID_buck_I_UVP);
    PID_Clear_Integral(&sPID_boost_V);
    PID_Clear_Integral(&sPID_AutomaticCompensation);

    if (Tmos < SOFTWARE_OTP_RECOVER_MOS && Tcap < SOFTWARE_OTP_RECOVER_CAP) {
        OTP_RecoverDelay++;

        // 温度降低5s之后解除过压保护
        if (OTP_RecoverDelay >= COUNT_TO_5S_ON_100Khz) {
            sStateBit.OTP_MOS_bit = 0;
            sStateBit.OTP_CAP_bit = 0;
            OTP_RecoverDelay      = 0;
        }
    } else {
        OTP_RecoverDelay = 0;
    }
}

/**
 * 超电自动补偿环路，超级电容会根据母线功率消耗自动进入充电或者放电状态。
 * 无任何保护状态的时候，才能进入放电环路，状态标志位如下：
 * sStateBit.Enable_bit =1
 * 电控发过来的信息必须为：
 * sCAN_RX_data.Enable = 1
 * 发回去给电控的数据
 * sCAN_TX_data.SuperCapReady = 1
 * sCAN_TX_data.SuperCapState = 0
 */
static uint32_t UVP_CapDelay =0;
static uint32_t OPL_batDelay =0;
void Automatic_Power_Compensation_Loop(void){

    Pbat = Vbat * Ibat;

    // 这个PI运算是主要的自动功率补偿运算
    PID_Preload_Integral(&sPID_AutomaticCompensation, Vcap);
    PID_Compute(&sPID_AutomaticCompensation, Pbat, sCAN_RX_data.PowerLimint, &PID_PowerLoopOut);
    TIM1_PWM2_Comapre = PID_PowerLoopOut;

    // 这里是超电处于充电状态时，限制充电电流的PI计算
    Hysteresis_Comparator(Icap, SAFE_CHARGE_ICAP, SAFE_CHARGE_ICAP - 4, &IcapAutoLoopEgde);
    if (IcapAutoLoopEgde == RISING) {
        PID_Preload_Integral(&sPID_buck_I, Vcap);
        PID_Compute(&sPID_buck_I, Icap, SAFE_CHARGE_ICAP, &PID_CurrentLoopOut);

        Loop_Competition_Buck(TIM1_PWM2_Comapre, PID_CurrentLoopOut, &TIM1_PWM2_Comapre);
    } else if (IcapAutoLoopEgde == FALLING) {
        PID_Clear_Integral(&sPID_buck_I);
        TIM1_PWM2_Comapre = PID_PowerLoopOut;
    }

    // 这里超电放电时，避免升压过高的PI计算，基本不会进这里
    Hysteresis_Comparator(Vbat, SOFTWARE_OVP_VBAT, SOFTWARE_OVP_RECOVER_VBAT, &VbatAutoLoopEgde);
    if(VbatAutoLoopEgde == RISING){
        PID_Preload_Integral(&sPID_boost_V, Vcap);
        PID_Compute(&sPID_boost_V, Vbat, SOFTWARE_OVP_RECOVER_VBAT, &PID_VoltageLoopOut);

        Loop_Competition_Boost(TIM1_PWM2_Comapre, PID_VoltageLoopOut, &TIM1_PWM2_Comapre);
    }else if(VbatAutoLoopEgde == FALLING){
        PID_Clear_Integral(&sPID_boost_V);
        TIM1_PWM2_Comapre = PID_PowerLoopOut;
    }

    // 这里是超电充电时，避免过充的PI计算，充满电之后的恒压阶段才会运行这里
    Hysteresis_Comparator(Vcap, SOFTWARE_OVP_VCAP, SOFTWARE_OVP_RECOVER_VCAP, &VcapAutoLoopEgde);
    if (VcapAutoLoopEgde == RISING) {
        PID_Preload_Integral(&sPID_buck_V, Vcap);
        PID_Compute(&sPID_buck_V, Vcap, SOFTWARE_OVP_VCAP, &PID_VoltageLoopOut);

        Loop_Competition_Buck(TIM1_PWM2_Comapre, PID_VoltageLoopOut, &TIM1_PWM2_Comapre);
    } else if (VcapAutoLoopEgde == FALLING) {
        PID_Clear_Integral(&sPID_buck_V);
        TIM1_PWM2_Comapre = PID_PowerLoopOut;
    }

    // 半桥PWM比较值赋值，设定PWM占空比，这里是以上半桥的正占空比进行设置。
    SET_PWM_COMPARE(TIM1_PWM2_Comapre);

    // ADC采样触发比较值赋值。
    SET_ADC_TRIGGER_COMPARE(TIM1_PWM2_Comapre);

    // 超级电容模组进行自动功率补偿的时候，有两种情况会进入Power_Down()函数
    // 一：超级电容充满电了，处于涓流状态
    // 二：机器人死亡，底盘断电，电源管理模块无法输出电流
    // 所以当这情况下，把半桥关掉，就可以达到底盘断电的目的。
    Power_Down();

    // 这个if是判断超电什么时候用完电，进入过放保护的
    // 因为超电没电只存在两种情况，一种是第一次上电，另一种是放电的时候放完了
    // 前者会在上电的时候进行判断，这里是后者，只会在这里进行判断
    Hysteresis_Comparator(Vcap ,SOFTWARE_UVP_VCAP + 2 ,SOFTWARE_UVP_VCAP ,&VcapUVPEdge);
    Hysteresis_Comparator(Pbat ,sCAN_RX_data.PowerLimint + 100, sCAN_RX_data.PowerLimint , &PbatOPLEdge);
    if (VcapUVPEdge == FALLING) {
        UVP_CapDelay++;
        if(PbatOPLEdge == RISING){
            OPL_batDelay ++;
            if (OPL_batDelay >= COUNT_TO_200MS_ON_100Khz) {
                //这里要判断超电没电之后是否超功率，如果超功率+100W，200MS就进入保护。
                sStateBit.Enable_bit  = 0;
                sStateBit.UVP_Cap_bit = 1;
                OPL_batDelay          = 0;
                UVP_CapDelay          = 0;
                PID_Clear_Integral(&sPID_boost_V);
                PID_Clear_Integral(&sPID_buck_V);
                PID_Clear_Integral(&sPID_buck_I);
                PID_Clear_Integral(&sPID_buck_I_UVP);
                PID_Clear_Integral(&sPID_AutomaticCompensation);
            }
        }else if(PbatOPLEdge == FALLING){
            OPL_batDelay = 0;
        }
        if (UVP_CapDelay >= COUNT_TO_500MS_ON_100Khz) {
            // 迟滞500ms才会确认超电已经欠压，防止瞬时电流引起的电压跌落导致误触发欠压保护
            sStateBit.Enable_bit  = 0;
            sStateBit.UVP_Cap_bit = 1;
            UVP_CapDelay          = 0;
            OPL_batDelay          = 0;
            PID_Clear_Integral(&sPID_boost_V);
            PID_Clear_Integral(&sPID_buck_V);
            PID_Clear_Integral(&sPID_buck_I);
            PID_Clear_Integral(&sPID_buck_I_UVP);
            PID_Clear_Integral(&sPID_AutomaticCompensation);
        }
    } else if(VcapUVPEdge == RISING) {
        // 不欠压就清零计数。
        UVP_CapDelay = 0;
        OPL_batDelay = 0;
    }

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
    SuperCap_Disable();
    PID_Clear_Integral(&sPID_buck_V);
    PID_Clear_Integral(&sPID_buck_I);
    PID_Clear_Integral(&sPID_buck_I_UVP);
    PID_Clear_Integral(&sPID_boost_V);
    PID_Clear_Integral(&sPID_AutomaticCompensation);
}


static uint32_t SuperCapDisableDelay = 0;
static uint32_t SuperCapEnableDelay = 0;
static uint32_t SuperCaoDisableFlag  = 0;
static uint32_t SuperCaoEnableFlag  = 0;

uint8_t SuperCap_Disable(void){

#ifdef PLUS
    DISABLE_CAP;
    SuperCapEnableDelay = 0;
    SuperCapDisableDelay++;
    if(SuperCapDisableDelay > COUNT_TO_100MS_ON_100Khz){
        SuperCapDisableDelay = 0;
        return 1; // 失能成功
    }
    return 0;
#endif

#ifdef LITE
    //超级电容控制板失能要先关PWM再关PMOS，避免关掉PMOS之后，电感续流无处可去产生电压尖峰，导致MOS击穿。
    TIM1_PWM2_BREAK;
    SuperCapEnableDelay = 0;
    SuperCapDisableDelay++;
    if(SuperCapDisableDelay > COUNT_TO_100MS_ON_100Khz){
        PMOS_OFF;
        SuperCapDisableDelay = 0;
        return 1; // 失能成功
    }
    return 0;
#endif

}

static uint8_t SuperCap_Enable(void){

#ifdef PLUS
    ENABLE_CAP;
    SuperCapDisableDelay = 0;
    SuperCapEnableDelay++;
    if(SuperCapEnableDelay > COUNT_TO_10MS_ON_100Khz){
        SuperCapEnableDelay = 0;
        return 1; // 使能成功
    }
    return 0;
#endif

#ifdef LITE
    //超级电容控制板使能要先开PMOS再开PWM，避免PMOS开关速度过慢导致烧毁。
    PMOS_ON;
    SuperCapDisableDelay = 0;
    SuperCapEnableDelay++;
    if(SuperCapEnableDelay > COUNT_TO_10MS_ON_100Khz){
        TIM1_PWM2_DISBREAK;
        SuperCapEnableDelay = 0;
        return 1; // 使能成功
    }
    return 0;
#endif
}

/**
 * PMOS关闭函数，在除了充电和放电的时候使用，因为充电和放电的时候需要考虑其他因素，所以不能直接使用这条函数来关PMOS
 */

uint32_t PMOS_OffDelay = 0; //PMOS关断延迟
uint32_t PMOS_OnDelay = 0; //PWM开通延迟
uint8_t PMOS_ReOpen =0; //PMOS重新打开
uint32_t PMOS_ReOpenDelay =0; //PMOS重新打开
void Power_Down(void){

    //如果PWM处于刹车状态，则PID保持初始状态，避免计算错误导致
    if(!TIM1_PWM2_IS_OUT){
        PID_Clear_Integral(&sPID_boost_V);
        PID_Clear_Integral(&sPID_buck_V);
        PID_Clear_Integral(&sPID_buck_I);
        PID_Clear_Integral(&sPID_buck_I_UVP);
        PID_Clear_Integral(&sPID_AutomaticCompensation);
    }

    Hysteresis_Comparator(Ibat, PMOS_ON_CURRENT, PMOS_OFF_CURRENT, &PowerdownEdge);
    if (PowerdownEdge == FALLING && PMOS_ReOpen == 0) {
        PMOS_OnDelay =0;
        PMOS_OffDelay++;
        if (PMOS_OffDelay > COUNT_TO_500MS_ON_100Khz) {
            SuperCaoDisableFlag = SuperCap_Disable();

            if(SuperCaoDisableFlag){
                // 进这里说明PMOS已经关掉了

                if(PMOS_OffDelay > COUNT_TO_2S_ON_100Khz){
                    // 延迟2s，判断底盘是否真正断电，如果没断电，则重新打开超电。

                    if(Vbat > SOFTWARE_UVP_BAT){
                        PMOS_ReOpenDelay ++;
                        if (PMOS_ReOpenDelay > 10) {
                            // 延迟100ms * 10，重新打开PMOS，给底盘供电
                            PMOS_ReOpen = 1;
                            PMOS_ReOpenDelay = 0;
                        }
                    }else {
                        PMOS_ReOpenDelay = 0;
                        PMOS_ReOpen = 0;
                    }

                }

            PID_Clear_Integral(&sPID_boost_V);
            PID_Clear_Integral(&sPID_buck_V);
            PID_Clear_Integral(&sPID_buck_I);
            PID_Clear_Integral(&sPID_buck_I_UVP);
            PID_Clear_Integral(&sPID_AutomaticCompensation);
            }
        }
    }else if(PowerdownEdge == RISING || PMOS_ReOpen == 1){

        PMOS_ReOpenDelay = 0; // 避免在上面累加到一半，电流突然变大，下一次进去会保持上一次的值
        PMOS_OffDelay = 0;
        PMOS_OnDelay++;
        if (PMOS_OnDelay > COUNT_TO_10MS_ON_100Khz) {
            SuperCaoEnableFlag = SuperCap_Enable();

            if(SuperCaoEnableFlag){
                PMOS_ReOpen = 0;
                PMOS_OnDelay = 0;
            }
        }
    }
}


uint32_t LED_Refresh_CNT = 0; // 中间变量，用于同步灯光闪烁
/**
 * LED刷新函数，刷新频率大概为12Hz
 * 如果LED灯没亮，说明程序卡在初始化校准了
 * 但凡有一个LED灯在闪烁，都说明处于保护状态
 */
#ifdef LITE
void LED_Refresh(void){
    LED_Refresh_CNT++;
    if (sStateBit.OTP_CAP_bit || sStateBit.OTP_MOS_bit) {
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
    } else if (sStateBit.CAN_Offline_bit) {
        // CAN掉线的时候，两个灯同时闪烁
        if (LED_Refresh_CNT & 1) {
            LED_CAP_ON;
            LED_CHASSIS_ON;
        } else {
            LED_CAP_OFF;
            LED_CHASSIS_OFF;
        }
    } else  if (sStateBit.UVP_Cap_bit) {
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

#endif

#ifdef PLUS
void LED_Refresh(void){
    LED_Refresh_CNT++;

    if(LED_Refresh_CNT < 50){
        // 这是初始化，会跑马灯五次
        if ((LED_Refresh_CNT & DIV4) == 0){
            LED_RED_OFF;
            LED_GREEN_OFF;
            LED_BLUE_OFF;
        }else if((LED_Refresh_CNT & DIV4) == 1){
            LED_RED_ON;
            LED_GREEN_OFF;
            LED_BLUE_OFF;
        }else if((LED_Refresh_CNT & DIV4) == 2){
            LED_RED_OFF;
            LED_GREEN_ON;
            LED_BLUE_OFF;
        }else{
            LED_RED_OFF;
            LED_GREEN_OFF;
            LED_BLUE_ON;
        }
    }else {
        if(sStateBit.BOOM_bit){
            //这里是爆炸，保险丝熔断了，红、绿、蓝常亮
            LED_RED_ON;
            LED_GREEN_ON;
            LED_BLUE_ON;

        }else if (sStateBit.OTP_CAP_bit || sStateBit.OTP_MOS_bit) {
            // 过温保护，红灯闪烁，蓝绿灯交替闪烁
            LED_RED_BLINK;
            if (LED_Refresh_CNT & 1) {
                LED_GREEN_ON;
                LED_BLUE_OFF;
            } else {
                LED_GREEN_OFF;
                LED_BLUE_ON;
            }
        } else if (sStateBit.UVP_Bat_bit) {
            // 电池没电的时候，红灯常亮，绿、蓝熄灭
            LED_RED_ON;
            LED_GREEN_OFF;
            LED_BLUE_OFF;
        } else if (sStateBit.OCP_bit) {
            // 母线过流的时候，红灯闪烁，蓝灯闪烁，绿灯常亮
            LED_RED_BLINK;
            LED_GREEN_ON;
            LED_BLUE_BLINK;
        } else if (sStateBit.OVP_Bat_bit) {
            // 动能回收母线过压的时候：红灯闪烁，绿灯闪烁，绿灯熄灭
            LED_RED_BLINK;
            LED_GREEN_OFF;
            LED_BLUE_BLINK;
        } else if (sStateBit.CAN_Offline_bit) {
            // CAN掉线的时候，红、绿、蓝同时闪烁
            if (LED_Refresh_CNT & 1) {
                LED_RED_ON;
                LED_GREEN_ON;
                LED_BLUE_ON;
            } else {
                LED_RED_OFF;
                LED_GREEN_OFF;
                LED_BLUE_OFF;
            }
        } else if(sStateBit.Enable_bit == DISABLE){
            // 超级电容模组不使能，红灯熄灭，绿、蓝交替闪烁
            LED_RED_OFF;            
            if (LED_Refresh_CNT & 1) {
                LED_GREEN_ON;
                LED_BLUE_ON;
            } else {
                LED_GREEN_OFF;
                LED_BLUE_OFF;
            }
        }else if (sStateBit.UVP_Cap_bit) {
            // 电容用完电的时候，红灯闪烁，绿灯闪烁，蓝灯熄灭
            // 底盘灯灭
            LED_RED_BLINK;
            LED_GREEN_BLINK;
            LED_BLUE_OFF;
        } else if (SuperCapEnergy_AVG >= 95) {
            // 充满电的时候，红灯熄灭，绿、蓝常亮
            LED_RED_OFF;
            LED_GREEN_ON;
            LED_BLUE_ON;
        }else if (sCAN_TX_data.SuperCapReady) {
            LED_RED_OFF;
            if (sStateBit.Charge_bit) {
                // 充电的时候：红灯熄灭，绿灯常亮，蓝灯熄灭
                LED_GREEN_ON;
                LED_BLUE_OFF;
            } else if (!sStateBit.Charge_bit) {
                // 放电的时候：红灯熄灭，绿灯熄灭，蓝灯常亮
                LED_GREEN_OFF;
                LED_BLUE_ON;
            }
        }
    }
}

#endif



/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
//
void Power_Loop_Parameter_Init(void){

    sStateBit.SoftStart_bit = 1; // 第一次开机必须是软起动
    sStateBit.Enable_bit    = 0; // 上电处于失能状态，等待CAN信息。

    sStateBit.CAN_Offline_bit = 1; // 上电默认CAN掉线，必须收到CAN信息之后才会运行。
    sStateBit.UVP_Bat_bit     = 0;
    sStateBit.UVP_Cap_bit     = 1;  // 上电默认超级电容组没电
    sStateBit.OTP_MOS_bit     = 0;
    sStateBit.OTP_CAP_bit     = 0;
    sStateBit.OCP_bit         = 0;
    sStateBit.OVP_Bat_bit     = 0;

    #ifdef LITE
    TIM1_PWM2_BREAK;
    // 开机需要把半桥关掉，防止初始比较值产生的PWM导致超电爆炸。
    PMOS_OFF;
    // 开机直接先把PMOS关掉。

    sPID_buck_V.Kp                  = PID_BUCK_V_KP;
    sPID_buck_V.Ki                  = PID_BUCK_V_KI;
    sPID_buck_V.OutMax              = MAX_PWM_COMPARE;
    sPID_buck_V.OutMin              = MIN_PWM_COMPARE;
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
    sPID_boost_V.OutMax              = MAX_PWM_COMPARE;
    sPID_boost_V.OutMin              = MIN_PWM_COMPARE;
    sPID_boost_V.ControllerDirection = REVERSE; // 意思是，控制方向与变化方向相反
    // 当设定输出为20V，当前输出为10V，正方向的PID提高电压的方法是增大输出，PID的输出直接作为上管PWM的占空比，当PID输出（上管占空比）变大时，电压变小；故控制反向
    PID_Init(&sPID_boost_V);
    // Boost电压环PID参数初始化

    
    sPID_AutomaticCompensation.Kp                  = PID_AUTOMATIC_COMPENSATION_KP;
    sPID_AutomaticCompensation.Ki                  = PID_AUTOMATIC_COMPENSATION_KI;
    sPID_AutomaticCompensation.OutMax              = MAX_PWM_COMPARE;
    sPID_AutomaticCompensation.OutMin              = MIN_PWM_COMPARE;
    sPID_AutomaticCompensation.ControllerDirection = DIRECT;
    PID_Init(&sPID_AutomaticCompensation);
    #endif

    #ifdef PLUS
    DISABLE_CAP;
    // 开机需要把半桥关掉，防止初始比较值产生的PWM导致超电爆炸。

    sPID_buck_V.Kp                  = PID_BUCK_V_KP;
    sPID_buck_V.Ki                  = PID_BUCK_V_KI;
    sPID_buck_V.OutMax              = MAX_PWM_COMPARE;
    sPID_buck_V.OutMin              = MIN_PWM_COMPARE;
    sPID_buck_V.ControllerDirection = REVERSE; // 意思是，控制方向与变化方向相同
    PID_Init(&sPID_buck_V);
    // Buck电压环PID参数初始化

    sPID_buck_I.Kp                  = PID_BUCK_I_KP;
    sPID_buck_I.Ki                  = PID_BUCK_I_KI;
    sPID_buck_I.OutMax              = MAX_PWM_COMPARE;
    sPID_buck_I.OutMin              = MIN_PWM_COMPARE;
    sPID_buck_I.ControllerDirection = REVERSE; // 意思是，控制方向与变化方向相同
    PID_Init(&sPID_buck_I);
    // Buck电流环PID参数初始化

    sPID_buck_I_UVP.Kp                  = PID_BUCK_I_KP;
    sPID_buck_I_UVP.Ki                  = PID_BUCK_I_KI;
    sPID_buck_I_UVP.OutMax              = MAX_PWM_COMPARE;
    sPID_buck_I_UVP.OutMin              = MIN_PWM_COMPARE;
    sPID_buck_I_UVP.ControllerDirection = REVERSE; // 意思是，控制方向与变化方向相同
    PID_Init(&sPID_buck_I_UVP);

    sPID_boost_V.Kp                  = PID_BOOST_V_KP;
    sPID_boost_V.Ki                  = PID_BOOST_V_KI;
    sPID_boost_V.OutMax              = MAX_PWM_COMPARE;
    sPID_boost_V.OutMin              = MIN_PWM_COMPARE;
    sPID_boost_V.ControllerDirection = DIRECT; // 意思是，控制方向与变化方向相反
    // 当设定输出为20V，当前输出为10V，正方向的PID提高电压的方法是增大输出，PID的输出直接作为上管PWM的占空比，当PID输出（上管占空比）变大时，电压变小；故控制反向
    PID_Init(&sPID_boost_V);
    // Boost电压环PID参数初始化

    sPID_AutomaticCompensation.Kp                  = PID_AUTOMATIC_COMPENSATION_KP;
    sPID_AutomaticCompensation.Ki                  = PID_AUTOMATIC_COMPENSATION_KI;
    sPID_AutomaticCompensation.OutMax              = MAX_PWM_COMPARE;
    sPID_AutomaticCompensation.OutMin              = MIN_PWM_COMPARE;
    sPID_AutomaticCompensation.ControllerDirection = REVERSE;
    PID_Init(&sPID_AutomaticCompensation);

    #endif

}

void Protection_Init(void){

    sOCPbatConfig.SchmittTriggerDirection = RISING;
    sOCPbatConfig.RisingThreshold = SOFTWARE_OCP_IBAT;
    sOCPbatConfig.FallingThreshold = SOFTWARE_OCP_RECOVER_IBAT;
    sOCPbatConfig.Timeout = COUNT_TO_200MS_ON_100Khz;
    Anomaly_Detection_Init(&sOCPbatConfig);
    
    sOCPcapConfig.SchmittTriggerDirection = RISING;
    sOCPcapConfig.RisingThreshold = SOFTWARE_CHARGE_OCP_ICAP;
    sOCPcapConfig.FallingThreshold = SOFTWARE_CHARGE_OCP_RECOVER_ICAP;
    sOCPcapConfig.Timeout = COUNT_TO_200MS_ON_100Khz;
    Anomaly_Detection_Init(&sOCPcapConfig);

    sOVPbatConfig.SchmittTriggerDirection = RISING;
    sOVPbatConfig.RisingThreshold = SOFTWARE_OVP_VBAT;
    sOVPbatConfig.FallingThreshold = SOFTWARE_OVP_RECOVER_VBAT;
    sOVPbatConfig.Timeout = COUNT_TO_200MS_ON_100Khz;
    Anomaly_Detection_Init(&sOVPbatConfig);
    
    sUVPbatConfig.SchmittTriggerDirection = FALLING;
    sUVPbatConfig.RisingThreshold = SOFTWARE_UVP_BAT + 1;
    sUVPbatConfig.FallingThreshold = SOFTWARE_UVP_BAT;
    sUVPbatConfig.Timeout = COUNT_TO_5S_ON_100Khz;
    Anomaly_Detection_Init(&sUVPbatConfig);
    
    sOTPcapConfig.SchmittTriggerDirection = RISING;
    sOTPcapConfig.RisingThreshold = SOFTWARE_OTP_CAP;
    sOTPcapConfig.FallingThreshold = SOFTWARE_OTP_RECOVER_CAP;
    sOTPcapConfig.Timeout = COUNT_TO_5S_ON_100Khz;
    Anomaly_Detection_Init(&sOTPcapConfig);

    sOTPmosConfig.SchmittTriggerDirection = RISING;
    sOTPmosConfig.RisingThreshold = SOFTWARE_OTP_MOS;
    sOTPmosConfig.FallingThreshold = SOFTWARE_OTP_RECOVER_MOS;
    sOTPmosConfig.Timeout = COUNT_TO_5S_ON_100Khz;
    Anomaly_Detection_Init(&sOTPmosConfig);
}


/**
 * 过滤器初始化函数
 */
void FDCAN_Filter_Init(void){
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;       // 标准CANID
    sFilterConfig.FilterIndex  = 0;                       // 过滤器序列0，只配置一个过滤器
    sFilterConfig.FilterType   = FDCAN_FILTER_DUAL;       // 双ID过滤模式，只能接受两个指定ID的数据
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 接收的数据保存到FIFO0
    sFilterConfig.FilterID1    = CAN_BUS_TO_SUPERCAP_ID;         // ID白名单1
    sFilterConfig.FilterID2    = CAN_TEST_ID;             // ID白名单2

    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
    // 全局过滤器配置，与过滤器不匹配的ID数据将会拒绝接收，不会进入中断。
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
}


uint8_t Anomaly_Detection_Init(AnomalyDetectionTypeDef *pAnomalyDetection){
    if(pAnomalyDetection->RisingThreshold < pAnomalyDetection->FallingThreshold){
        return 1;
    }

    pAnomalyDetection->Counter = 0;

    // 初始化滞回比较器状态为触发状态的反。
    if(pAnomalyDetection->SchmittTriggerDirection == RISING){
        pAnomalyDetection->SchmittTriggerState = FALLING;
    }else {
        pAnomalyDetection->SchmittTriggerState = RISING;
    }
    pAnomalyDetection->IsInit = 1;
    return 0;
}

/*
* 异常状态检测函数
*/
void Anomaly_Detection(AnomalyDetectionTypeDef *pAnomalyDetection, float detected_input ,uint8_t *state_output){

    if(pAnomalyDetection->IsInit == 1){
        // 初始化了才会进行检测。
        Hysteresis_Comparator(detected_input, pAnomalyDetection->RisingThreshold, pAnomalyDetection->FallingThreshold, &pAnomalyDetection->SchmittTriggerState);
        if (pAnomalyDetection->SchmittTriggerState == pAnomalyDetection->SchmittTriggerDirection) {
            // 判断滞回比较器的状态与设定的触发状态是否相符。
            pAnomalyDetection->Counter++;
            if (pAnomalyDetection->Counter >= pAnomalyDetection->Timeout) {
                *state_output = 1;
                pAnomalyDetection->Counter = 0;
                // 持续超过设定时间，判定为异常，返回1。
            }
        } else {
            pAnomalyDetection->Counter = 0;
        }
    }
    // 没有异常，返回0。
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


/**
* 温度计算
* NTC:B3950-100K
* Plus：上拉电阻47K
* Lite：下拉电阻12K
*/
void Temperature_Calculations(void){

    #ifdef PLUS
        Tmos = NTC3950_100K_RH_47K_Fitting_A * log(sADC_Value_AVG.Tmos) + NTC3950_100K_RH_47K_Fitting_B;
        Tcap = NTC3950_100K_RH_47K_Fitting_A * log(sADC_Value_AVG.Tcap) + NTC3950_100K_RH_47K_Fitting_B;

    #endif

    #ifdef LITE
        Tmos =(NTC3950_100K_RL_12K_Fitting_A * sADC_Value_AVG.Tmos *sADC_Value_AVG.Tmos *sADC_Value_AVG.Tmos 
        + NTC3950_100K_RL_12K_Fitting_B * sADC_Value_AVG.Tmos *sADC_Value_AVG.Tmos
        + NTC3950_100K_RL_12K_Fitting_C * sADC_Value_AVG.Tmos + NTC3950_100K_RL_12K_Fitting_D );

        Tcap =(NTC3950_100K_RL_12K_Fitting_A * sADC_Value_AVG.Tcap *sADC_Value_AVG.Tcap *sADC_Value_AVG.Tcap 
        + NTC3950_100K_RL_12K_Fitting_B * sADC_Value_AVG.Tcap *sADC_Value_AVG.Tcap
        + NTC3950_100K_RL_12K_Fitting_C * sADC_Value_AVG.Tcap + NTC3950_100K_RL_12K_Fitting_D);

    #endif

}

/*
------------------------------------------------------------------------------------------------------------------------------------------
*/
static float Vbat_AVG =0;
static float Vcap_AVG =0;
static float Ibat_AVG =0;
static float Icap_AVG =0;
static float Vcap_DCR_Compemsion =0;
// 这个函数是将功率计算出来
void Power_Calculations(void){
    Ibat_AVG = sADC_Value_AVG.Ibat * sADC_Fit.Ibat_A + sADC_Fit.Ibat_B;
    Icap_AVG = sADC_Value_AVG.Icap * sADC_Fit.Icap_A + sADC_Fit.Icap_B;

    Vbat_AVG = sADC_Value_AVG.Vbat * sADC_Fit.Vbat_A + sADC_Fit.Vbat_B;
    Vcap_AVG = sADC_Value_AVG.Vcap * sADC_Fit.Vcap_A + sADC_Fit.Vcap_B;


    BatPower_AVG      = Ibat_AVG * Vbat_AVG;
    SuperCapPower_AVG = Vcap_AVG * Icap_AVG;

    if(BatPower_AVG < SuperCapPower_AVG){
        // 限制大小，防止下溢
        ChassisPower_AVG = 0; 
    }else {
        ChassisPower_AVG = BatPower_AVG - SuperCapPower_AVG + PBAT_POWER_LOSS; 
        // 底盘功率 = 电池功率 - 超级电容功率
        // 充电的时候，超级电容功率是正的，与底盘共同消耗电池的功率，所以底盘功率是电池功率 - 超级电容充电功率；
        // 放电的时候，超级电容功率是负的，超级电容给底盘补偿功率，所以底盘功率是电池功率 - 超级电容放电功率（负的），负负的正，总功率就是|电容放电功率| + |电池功率|。
    }

#ifdef SUPERCAP_DCR_COMPENSATION
    Vcap_DCR_Compemsion = Vcap_AVG - (SUPERCAP_DCR * Icap_AVG);
#else
    Vap_DCR_Compemsion = Vcap_AVG;
#endif

    if (Vcap_DCR_Compemsion > SOFTWARE_UVP_VCAP) {
        // 限制大小，防止下溢
        SuperCapEnergy_AVG = (Vcap_DCR_Compemsion - SOFTWARE_UVP_VCAP) * 100.0f / SUPERCAP_AVAILABLE_VOLTAGE;//    根据可用电压范围（6-20V）转换成能量百分比
    } else {
        SuperCapEnergy_AVG = 0;
    }

#ifdef PLUS
    if(sStateBit.BOOM_bit || sStateBit.SoftStart_bit){
        ChassisPower_AVG = BatPower_AVG + PBAT_POWER_LOSS;
    }
#endif

    sCAN_TX_data.ChassisPower = ChassisPower_AVG >> 1; // 发给电控的功率数值不需要太准确，所以右移一位，除以2，将uint8的数值范围变为0-512
    sCAN_TX_data.BatPower = BatPower_AVG;
    sCAN_TX_data.SuperCapEnergy = SuperCapEnergy_AVG;

    if(Vbat_AVG >= 25.5){
        sCAN_TX_data.BatVoltage = 255;
    }else {
        sCAN_TX_data.BatVoltage = Vbat_AVG *10;

    }

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

/**
 * fdcan接收回调函数，定义了这条函数回调就会到这里执行，在HAL_FDCAN_IRQHandler函数中调用了。
 * 其实这条函数也就是把FDCAN的接收缓冲数组赋值给结构体而已，方便使用变量
 * 只有与FDCAN过滤器匹配的ID信息才会触发FDCAN接收中断，产生回调
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
    uint8_t CAN_RX_Buff[8];

    // CAN看门狗计数清零，确保CAN在线
    CAN_WDG_Counter = 0;

    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &sFDCAN1_RxHeader, CAN_RX_Buff); // 接收数据

    if(CAN_ReceiveDataRefresh_Flag){
        //判断数据刷新标志位，限制数据更新速度，避免电控发送动态的数据太快，导致超电环路爆炸

        // 在这里直接把数据分配到结构体中，方便调用
        sCAN_RX_data.Enable      = CAN_RX_Buff[0];
        if(CAN_RX_Buff[2] > 10){
            sCAN_RX_data.PowerLimint = CAN_RX_Buff[2] - PBAT_POWER_LOSS;
        }else {
            sCAN_RX_data.PowerLimint = 10;
        }

        CAN_ReceiveDataRefresh_Flag = 0;
    }

}

/**
 * FDCAN发送函数
 * TX_temp是相关发送数据的结构体，详见CAN_TransmitData
 */
void Can_SendMess(CAN_TransmitDataTypeDef *TX_temp){
    // 这些根据DJI的CAN协议来配。
    sFDCAN1_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;     // 波特率切换关闭
    sFDCAN1_TxHeader.FDFormat      = FDCAN_CLASSIC_CAN; // 经典CAN
    sFDCAN1_TxHeader.Identifier    = CAN_SUPERCAP_TO_BUS_ID;   // 发送ID
    sFDCAN1_TxHeader.IdType        = FDCAN_STANDARD_ID; // 标准CANID
    sFDCAN1_TxHeader.DataLength    = FDCAN_DLC_BYTES_8; // 数据长度
    sFDCAN1_TxHeader.TxFrameType   = FDCAN_DATA_FRAME;  // 数据类型


    uint8_t CAN_TX_BUFF[8] = {0};
    CAN_TX_BUFF[0]         = TX_temp->SuperCapReady;
    CAN_TX_BUFF[1]         = TX_temp->SuperCapState;
    CAN_TX_BUFF[2]         = TX_temp->SuperCapEnergy;
    CAN_TX_BUFF[3]         = TX_temp->ChassisPower;
    CAN_TX_BUFF[4]         = TX_temp->BatVoltage;
    CAN_TX_BUFF[5]         = TX_temp->BatPower;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &sFDCAN1_TxHeader, CAN_TX_BUFF);
}


void DebugOut_UART(void){

    sUART_TxData.ch_data[0] = (float)BatPower_AVG;
    sUART_TxData.ch_data[1] = (float)SuperCapPower_AVG;
    sUART_TxData.ch_data[2] = (float)ChassisPower_AVG;
    sUART_TxData.ch_data[3] = (float)SuperCapEnergy_AVG;
    sUART_TxData.ch_data[4] = (float)Vbat_AVG;
    sUART_TxData.ch_data[5] = (float)Vcap_AVG;
    sUART_TxData.ch_data[6] = (float)Ibat_AVG;
    sUART_TxData.ch_data[7] = (float)Icap_AVG;

    sUART_TxData.ch_data[8] = (float)sCAN_RX_data.Enable;
    sUART_TxData.ch_data[9] = (float)sCAN_RX_data.PowerLimint;
    sUART_TxData.ch_data[10] = (float)sCAN_TX_data.SuperCapReady;
    sUART_TxData.ch_data[11] = (float)sCAN_TX_data.SuperCapState;
    sUART_TxData.ch_data[12] = (float)uwTick;// 发送系统滴答计数，作为运行时间戳，单位ms

    sUART_TxData.tail[0] =0x00;
    sUART_TxData.tail[1] =0x00;
    sUART_TxData.tail[2] =0x80;
    sUART_TxData.tail[3] =0x7f;

    HAL_UART_Transmit(&huart1, (uint8_t *)&sUART_TxData, sizeof(sUART_TxData),sizeof(sUART_TxData)); // 发送数据
}

/**
 * 这个函数将带有96位UID的结构体数组与STM32出厂自带的唯一UID进行对比
 * 如果UID匹配则使用对应的ADC拟合参数对ADC采样值进行转换。
 */
void ADC_Curve_Fitting(void){
    while(1){
        for (uint8_t i = 0; i < 50; i++) {
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

    sADC_Value_SUM.Vbat += (uint32_t)ADC2Value[0];
    sADC_Value_SUM.Vcap += (uint32_t)ADC2Value[1];

    sADC_Value_SUM.Tcap += (uint32_t)ADC1Value[2];
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
