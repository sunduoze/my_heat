#include "OpenT12.h"

//[既然有了nmos 为啥还要pmos]科普:https://www.bilibili.com/video/BV1Mb4y1k7fd?from=search&seid=15411923932488650975

enum MOS_Type{
    PMOS = 0,
    NMOS
};
//PWM
uint16_t PWM_Freq = 16200;   // 频率
uint8_t PWM1_Channel = 0;    // 通道
uint8_t PWM2_Channel = 0;    // 通道
uint8_t PWM_Resolution = 8;  // 分辨率
//基础温控
uint8_t MyMOS = PMOS;
uint8_t POWER = 0;
uint8_t PWM = 0;
uint8_t LastPWM = 0;
uint16_t last_temp = 0;       //LastADC = 0;
double TipTemperature = 0;
double PID_Output = 0;
double PID_Setpoint = 0;
double TempGap = 0;
uint32_t temp_sensor_sampleing_interval = 0; //ADC采样间隔(ms)
//PID
float aggKp = 50.0, aggKi = 0.0, aggKd = 0.5;
float consKp = 30.0, consKi = 1.0, consKd = 0.5;

void oled_init()
{
    //初始化OLED
    Disp.begin();
    Disp.setBusClock(921600);
    Disp.enableUTF8Print();
    Disp.setFontDirection(0);
    Disp.setFontPosTop();
    Disp.setFont(u8g2_font_wqy12_t_gb2312);
    Disp.setDrawColor(1);
    Disp.setFontMode(1);
}

//系统信息
uint64_t ChipMAC;
char ChipMAC_S[19] = {0};
char CompileTime[20];

void heat_hw_init(void)
{
    pinMode(TIP_ADC_PIN, INPUT_PULLDOWN);
    pinMode(CUR_ADC_PIN, INPUT_PULLDOWN);

    ledcAttachPin(PWM1_PIN, PWM1_Channel);             // 绑定PWM1通道
    ledcSetup(PWM1_Channel, PWM_Freq, PWM_Resolution); // 设置PWM1通道

    if (PWM2_PIN != -1)
    {
        ledcAttachPin(PWM2_PIN, PWM2_Channel);             // 绑定PWM2通道
        ledcSetup(PWM2_Channel, PWM_Freq, PWM_Resolution); // 设置PWM2通道
    }
    SetPOWER(0); //关闭功率管输出 

    if (SW_PIN != -1)
    {
        //初始化SW-PIN休眠检测引脚
        pinMode(SW_PIN, INPUT_PULLUP);
        //初始化SW-PIN休眠检测引脚中断 (尽可能减少中断的使用)
        //attachInterrupt(SW_PIN, SW_IRQHandler, CHANGE);
    }
        ////////////////////////////初始化硬件/////////////////////////////
    //获取系统信息
    ChipMAC = ESP.getEfuseMac();
    sprintf(CompileTime, "%s %s", __DATE__, __TIME__);
    for (uint8_t i = 0; i < 6; i++)
        sprintf(ChipMAC_S + i * 3, "%02X%s", ((uint8_t*) &ChipMAC)[i], (i != 5) ? ":" : "");

    Serial.begin(115200);
    beep_init();
    pinMode(POWER_ADC_PIN, INPUT);  //主电压分压检测ADC
    max6675_init();
    heat_ctrl_init();
    sys_RotaryInit();//初始化编码器
    oled_init();

    ////////////////////////////初始化软件/////////////////////////////
    //显示启动信息
    //ShowBootMsg();
    FilesSystemInit();//启动文件系统，并读取存档
    shellInit();//初始化命令解析器
    BLE_Init();//初始化蓝牙（可选）
    SetSound(BootSound); //播放音效
    System_UI_Init();//初始化UI
    sys_Counter_SetVal(BootTemp);//首次启动的时候根据启动温度配置，重新设定目标温度
    LoadTipConfig();//载入烙铁头配置
    //显示Logo
//    EnterLogo();
    //开机密码
    // while (!EnterPasswd())
    // {
    //     Pop_Windows("身份验证失败");
    // }

}

//初始化烙铁头温控系统
void heat_ctrl_init(void)
{
    //初始化烙铁头PID
    MyPID.SetOutputLimits(0, 255); //PID输出限幅
    MyPID.SetMode(AUTOMATIC); //PID控制模式
    MyPID.SetSampleTime(880); //PID采样时间
}

#include "MAX6675Soft.h"

/*
MAX6675Soft(cs, miso, sck)
*/
MAX6675Soft max6675_dev(MAX6675_CS, MAX6675_MISO, MAX6675_SCK); //for ESP8266 change to D3 (fails to BOOT/FLASH if pin LOW), D4 (fails to BOOT/FLASH if pin LOW), D7

void max6675_init(void)
{
    float  temp_val = 0.0f;
    //初始化MAX6675
    max6675_dev.begin();

    while (max6675_dev.getChipID() != MAX6675_ID)
    {
        Serial.println(F("MAX6675 error")); //(F()) saves string to flash & keeps dynamic memory free
        delay(5000);
    }

    if (Use_KFP)
    {
       temp_val = max6675_get_temp(1.0f);
        if (temp_val != MAX6675_ERROR)
        {
            kalman_fast_sattling(&KFP_Temp, temp_val);
        }
    }
}

void max6675_print_temp(void)
{
    float heat_temp = 0.0f;

    Serial.println("S:");
    Serial.println(millis());

    heat_temp = max6675_dev.getTemperature(max6675_dev.readRawData());

    Serial.println(heat_temp, 1);
    Serial.println("E:");
    Serial.println(millis());
}

double max6675_get_temp(double temp_ratio)
{
    return max6675_dev.getTemperature(max6675_dev.readRawData()) * temp_ratio;
}

//PWM输出模块
uint8_t PWMOutput_Lock = true;
void PWMOutput(uint8_t pwm)
{
    PWM_WORKY = true;
    //PWM锁
    if (PWMOutput_Lock || ShutdownEvent || Menu_System_State || ERROREvent)
    {
        PWM_WORKY = false;
        // Log(LOG_INFO,"输出被限制");
        // Serial.printf("输出被限制 PWMOutput_Lock=%d ShutdownEvent=%d Menu_System_State=%d ERROREvent=%d\n", PWMOutput_Lock, ShutdownEvent, Menu_System_State, ERROREvent);
        if (MyMOS == PMOS) pwm = 255;
        else pwm = 0;
        // //软件指示灯
        // digitalWrite(LED_Pin ,LOW);
    }
    // Serial.printf("PWM:%d\n",pwm);
    if (LastPWM != pwm)
    {
        if (pwm == 255) ledcWrite(PWM1_Channel, 256);
        else ledcWrite(PWM1_Channel, pwm);

        LastPWM = pwm;
    }
}

KFP KFP_Temp = {0.02, 0, 0, 0, 0.01, 0.1};
float SamplingRatioWork = 10;           //采样/加热 比率
//获取ADC读数
int GetADC0(void)
{
    static uint32_t ADCSamplingTime = 0; //上次采样时间
    static uint32_t ADCReadCoolTimer = 0;

    if (SYS_Ready)
    {
        temp_sensor_sampleing_interval = millis() - ADCSamplingTime;
        if (temp_sensor_sampleing_interval < ADC_PID_Cycle * (9 / 10.0)) return last_temp; //9/10周期ADC不应该工作，应该把时间留给加热

        //若原输出非关闭，则在关闭输出后等待一段时间，因为电热偶和加热丝是串在一起的，只有不加热的时候才能安全访问热偶温度
        if (PWMOutput_Lock == false)
        {
            ADCReadCoolTimer = millis();
            //锁定功率管输出：必须要关闭MOS管才能成功读取电热偶
            PWMOutput_Lock = true;
        }
        if (millis() - ADCReadCoolTimer <= ADC_PID_Cycle / 10)
        {
            return -1; //数据未准备好
        }
    }

    //读取并平滑滤波经过运算放大器放大后的热偶ADC数据
    uint16_t ADC_RAW = 66;//enzo analogRead(TIP_ADC_PIN);
    uint16_t ADC;

    //卡尔曼滤波器
    if (Use_KFP) ADC = kalmanFilter(&KFP_Temp, (float) ADC_RAW);
    else ADC = ADC_RAW;
    //printf("%d,%d\r\n", ADC_RAW,ADC);

    //解锁功率管输出：前提是没有打开菜单
    if (!Menu_System_State)
        PWMOutput_Lock = false;

    //记录采样间隔时间
    ADCSamplingTime = millis();

    last_temp = ADC;
    return last_temp;
}

//设置输出功率
void SetPOWER(uint8_t power)
{
    POWER = power;
    //MOS管分类处理
    if (MyMOS == PMOS)  PWM = 255 - power;
    else                PWM = power;
    PWMOutput(PWM);
}

float ADC_PID_Cycle_List[3] = {880, 440, 220};
//温度控制循环
void TemperatureControlLoop(void)
{
    Clear();
    double temp_val;

    PID_Setpoint = sys_Counter_Get();

    if (BoostEvent)         PID_Setpoint += BoostTemp;//短时功率加成
    else if (SleepEvent)    PID_Setpoint = SleepTemp;

    PID_Setpoint = constrain(PID_Setpoint, TipMinTemp, TipMaxTemp);
    //尝试访问ADC
    temp_val = max6675_get_temp(1.0f);
    if (temp_val != MAX6675_ERROR)
    {
        if (Use_KFP) TipTemperature = kalmanFilter(&KFP_Temp, (float) temp_val);
        else TipTemperature = temp_val;

        TempGap = abs(PID_Setpoint - TipTemperature);

        //根据温差选择合适的ADC-PID采样周期
        if (TempGap > 150)      ADC_PID_Cycle = ADC_PID_Cycle_List[0];
        else if (TempGap > 50)  ADC_PID_Cycle = ADC_PID_Cycle_List[1];
        else                    ADC_PID_Cycle = ADC_PID_Cycle_List[2];

        /////////////////////////////////////////////////////////////////////////////////////////////
        //控温模式
        if (PIDMode)
        {
            //PID模式

            //根据温度差选择最优的PID配置，PID参数可被Shell实时更改
            if (TempGap < 30)   MyPID.SetTunings(consKp, consKi, consKd);
            else                MyPID.SetTunings(aggKp, aggKi, aggKd);
            //更新PID采样时间：采样时间可被Shell实时更改
            MyPID.SetSampleTime(ADC_PID_Cycle);

            //尝试计算PID
            // Serial.printf("计算PID：%d PID输出:%lf\r\n", MyPID.Compute(), PID_Output);
            MyPID.Compute();
        } else
        {
            //模糊模式
            if (TipTemperature < PID_Setpoint)  PID_Output = 255;
            else                                PID_Output = 0;
        }
        //串口打印温度
        // Serial.printf("Temp:%lf,%lf,%g\r\n", TipTemperature, PID_Setpoint, PID_Output);
        /////////////////////////////////////////////////////////////////////////////////////////////
    }
}

