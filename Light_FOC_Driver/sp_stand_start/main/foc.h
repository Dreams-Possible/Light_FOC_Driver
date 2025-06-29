#pragma once

//头文件
#include<math.h>
#include"sp_sys.h"
#include"as5600_driver.h"
#include"te_pwm.h"

//定义
#define FOC_MODE_ANG 0
#define FOC_MODE_SPD 1

//FOC初始化
void foc_init();
//FOC设置模式
void foc_set_mode(uint8_t mode);
//FOC读取角度（°）
float foc_read_ang();
//FOC读取速度（rpm）
float foc_read_spd();
//FOC设置角度（°）
void foc_set_ang(float angle);
//FOC设置速度（rpm）
void foc_set_spd(float speed);
