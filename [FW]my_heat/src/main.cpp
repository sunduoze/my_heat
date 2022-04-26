#include <Arduino.h>
#include "OpenT12.h"

BluetoothSerial SerialBT;
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

OneButton RButton(BUTTON_PIN, true);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C Disp(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/21);
PID MyPID(&TipTemperature, &PID_Output, &PID_Setpoint, aggKp, aggKi, aggKd, DIRECT);
/////////////////////////////////////////////////////////////////

char* TipName = (char*)"文件系统错误：请上报";

float BootTemp =  60;      //开机温度          (°C)
float SleepTemp = 60;      //休眠温度          (°C)
float BoostTemp = 50;      //爆发模式升温幅度   (°C)

float ShutdownTime = 0;    //关机提醒              (分)
float SleepTime = 10;      //休眠触发时间          (分)
float ScreenProtectorTime = 60;     //屏保在休眠后的触发时间(秒)
float BoostTime = 60;      //爆发模式持续时间      (秒)


bool SYS_Ready = false;
//烙铁头事件
bool TipInstallEvent = true;
bool TipCallSleepEvent = false;
//到温提示音播放完成
bool TempToneFlag = false;
//休眠后屏保延迟显示标志
bool SleepScreenProtectFlag = false;
//温控系统状态
bool ERROREvent = false;
bool ShutdownEvent = false;
bool SleepEvent = false;
bool BoostEvent = false;
bool UnderVoltageEvent = false;
//PWM控制状态
bool PWM_WORKY = false;

uint8_t PIDMode = true;
uint8_t Use_KFP = true;
uint8_t PanelSettings = PANELSET_Detailed;
uint8_t ScreenFlip = false;
uint8_t SmoothAnimation_Flag = true;
float ScreenBrightness = 128;
uint8_t OptionStripFixedLength_Flag = false;

uint8_t Volume = true;
uint8_t RotaryDirection = false;
uint8_t HandleTrigger = HANDLETRIGGER_VibrationSwitch;

double SYS_Voltage = 3.3;
float UndervoltageAlert = 3;
char BootPasswd[20] = {0};
uint8_t Language = LANG_Chinese;
uint8_t MenuListMode = false;

float ADC_PID_Cycle = 220;

//面板状态条
uint8_t TempCTRL_Status = TEMP_STATUS_OFF;
uint8_t* C_table[] = {c1, c2, c3, Lightning, c5, c6, c7};
char* TempCTRL_Status_Mes[] = {
    (char*)"错误",
    (char*)"停机",
    (char*)"休眠",
    (char*)"提温",
    (char*)"正常",
    (char*)"加热",
    (char*)"维持",
};
// D:67.50,66.29,41.21,10.00,0.44,2.27,28.97,-0.14  P10 I 2 D
//先初始化硬件->显示LOGO->初始化软件
void setup()
{
    noInterrupts();//关闭中断
    heat_hw_init();
    SYS_Ready = true;
}

void loop()
{
    //获取按键
    sys_KeyProcess();
    // Serial.printf("Temp:%.6fmV,%.6fmV\r\n", analogRead(TIP_ADC_PIN) / 4096.0 * 3300, analogRead(CUR_ADC_PIN) / 4096.0 * 3300);
    if (!Menu_System_State)
    {
        //温度闭环控制
        TemperatureControlLoop();
        //更新系统事件：：系统事件可能会改变功率输出
        TimerEventLoop();
    }
    //更新状态码
    SYS_StateCode_Update();
    // Serial.printf("D:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", PID_Setpoint, TipTemperature, PID_Output, \
    //               analogRead(TIP_ADC_PIN) / 4096.0 * 3300, \
    //               analogRead(CUR_ADC_PIN) / 4096.0 * 3300, \
    //               MyPID.p, MyPID.i, MyPID.d, MyPID.i_item, MyPID.d_item);
        Serial.printf("D:%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\r\n", PID_Setpoint, TipTemperature, PID_Output, \
                  MyPID.p, MyPID.i, MyPID.d, MyPID.p_item, MyPID.i_item, MyPID.d_item);
    SetPOWER((uint8_t)PID_Output);
    //刷新UI
    System_UI();
}
/**
 * @description: 计算主电源电压
 * @param {*}
 * @return 主电源电压估计值
 */
double Get_MainPowerVoltage(void)
{
    //uint16_t POWER_ADC = analogRead(POWER_ADC_PIN);
    // double TipADC_V_R2 = analogReadMilliVolts(POWER_ADC_PIN) / 1000.0;
    // //double   TipADC_V_R2 = ESP32_ADC2Vol(POWER_ADC);
    // double TipADC_V_R1 = (TipADC_V_R2 * POWER_ADC_VCC_R1) / POWER_ADC_R2_GND;

    // SYS_Voltage = TipADC_V_R1 + TipADC_V_R2;

    SYS_Voltage = analogReadMilliVolts(POWER_ADC_PIN) * 0.1f / 1000.0f;
    return SYS_Voltage;
}

void SYS_Reboot(void)
{
    ESP.restart();
}

void About(void)
{
    //播放Logo动画
    EnterLogo();
    //生成项目QRCode
    QRCode qrcode;
    uint8_t qrcodeData[qrcode_getBufferSize(3)];

    switch (Language)
    {
        case LANG_Chinese:
            qrcode_initText(&qrcode, qrcodeData, 3, 0, "https://gitee.com/createskyblue/OpenT12");
            break;

        default:
            qrcode_initText(&qrcode, qrcodeData, 3, 0, "https://github.com/createskyblue/OpenT12");
            break;
    }

    Clear();

    uint8_t x_offset = (SCREEN_COLUMN - qrcode.size * 2) / 2;
    uint8_t y_offset = (SCREEN_ROW - qrcode.size * 2) / 2;

    for (uint8_t y = 0; y < qrcode.size; y++)
        for (uint8_t x = 0; x < qrcode.size; x++)
            if (qrcode_getModule(&qrcode, x, y))
                Draw_Pixel_Resize(x + x_offset, y + y_offset, x_offset, y_offset, 2, 2);

    Disp.setDrawColor(2);
    Disp.drawBox(x_offset - 2, y_offset - 2, qrcode.size * 2 + 4, qrcode.size * 2 + 4);
    Disp.setDrawColor(1);

    while (!sys_KeyProcess())
    {
        Display();
    }

}