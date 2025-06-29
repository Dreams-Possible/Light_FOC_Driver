#pragma once

//头文件
#include"sp_sys.h"
#include"driver/i2c_master.h"

//配置
#define SP_IIC_COM I2C_NUM_1
#define SP_IIC_SCL 2
#define SP_IIC_SDA 3
typedef i2c_master_dev_handle_t sp_iic_dev_t;

//IIC初始化
sp_t sp_iic_init();
//IIC添加设备
sp_iic_dev_t sp_iic_add(u8 addr);
//IIC写寄存器
sp_t sp_iic_write(sp_iic_dev_t dev,u8 addr,u8*data,u16 len);
//IIC读寄存器
sp_t sp_iic_read(sp_iic_dev_t dev,u8 addr,u8*data,u16 len);
//IIC检测设备
void sp_iic_check();
