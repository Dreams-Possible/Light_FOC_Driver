#include"as5600_driver.h"

//定义
#define DVE_ADDR 0x36
#define REG_ADDR 0x0c


//IIC读寄存器
static void iic_read_addr(uint8_t dev_addr,uint8_t reg_addr,uint8_t*data,uint8_t len);
//AS5600读取角度
float as5600_read();


//IIC读寄存器
static void iic_read_addr(uint8_t dev_addr,uint8_t reg_addr,uint8_t*data,uint8_t len)
{
    static sp_iic_dev_t dev=NULL;
    if(!dev)
    {
        dev=sp_iic_add(dev_addr);
        // SP_LOG("init");
    }else
    {
        if(sp_iic_read(dev,reg_addr,data,len)==ESP_OK)
        {
            // SP_LOG("succ");
        }else
        {
            // SP_LOG("fail");
        }
    }
    return;
}

//AS5600读取角度
float as5600_read()
{
    // SP_LOG("read");
    uint8_t data[2]={0};
    //IIC读寄存器
    iic_read_addr(DVE_ADDR,REG_ADDR,data,2);
    float angle=data[0]<<8|data[1];
    return angle*0.08789;///4096*360;
}
