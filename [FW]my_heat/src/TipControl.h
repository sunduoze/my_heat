#ifndef TipControl_H
#define TipControl_H

#include "Type.h"

extern uint8_t MyMOS;
extern uint8_t POWER;
extern uint8_t PWM;
extern uint8_t LastPWM;
extern uint16_t last_temp;//LastADC;
extern double TipTemperature;
extern double PID_Output;
extern double PID_Setpoint;
extern double TempGap;

extern float SamplingRatioWork;
extern float ADC_PID_Cycle_List[3];

extern float aggKp, aggKi, aggKd;
extern float consKp, consKi, consKd;

extern uint8_t PWMOutput_Lock;

extern void heat_hw_init(void);
extern void heat_ctrl_init(void);

void PWMOutput(uint8_t pin, uint8_t pwm);
int GetADC0(void);
double max6675_get_temp(double temp_ratio);
extern void SetPOWER(uint8_t power);

void TemperatureControlLoop(void);

#endif