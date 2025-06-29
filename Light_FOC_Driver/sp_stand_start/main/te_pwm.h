#pragma once

//头文件
#include"sp_sys.h"
#include"driver/ledc.h"

//PWM初始化
sp_t te_pwm_init();
//PWM设置
void te_pwm_set(u8 ch,float duty);
