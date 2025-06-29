#include"sp_iic.h"

//配置
#define COM SP_IIC_COM
#define SCL SP_IIC_SCL
#define SDA SP_IIC_SDA

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

//IIC初始化
sp_t sp_iic_init()
{
    i2c_master_bus_config_t config={0};
    config.clk_source=I2C_CLK_SRC_DEFAULT;
    config.i2c_port=COM;
    config.scl_io_num=SCL;
    config.sda_io_num=SDA;
    config.glitch_ignore_cnt=7;
    config.flags.enable_internal_pullup=true;
    i2c_master_bus_handle_t handle=NULL;
    if(i2c_new_master_bus(&config,&handle)==ESP_OK)
    {
        SP_LOG("iic init succ");
        return SP_OK;
    }else
    {
        SP_LOG("iic init fail");
        return SP_FAIL;
    }
}

//IIC添加设备
sp_iic_dev_t sp_iic_add(u8 addr)
{
    //设备信息
    i2c_device_config_t config={0};
    config.dev_addr_length=I2C_ADDR_BIT_LEN_7;
    config.device_address=addr;
    config.scl_speed_hz=400*1000;
    //获取端口
    i2c_master_bus_handle_t handle=NULL;
    i2c_master_get_bus_handle(COM,&handle);
    //添加设备
    sp_iic_dev_t dev=NULL;
    i2c_master_bus_add_device(handle,&config,&dev);
    if(!dev)
    {
        SP_LOG("iic add fail");
    }else
    {
        SP_LOG("iic add succ");
    }
    return dev;
}

//IIC写寄存器
sp_t sp_iic_write(sp_iic_dev_t dev,u8 addr,u8*data,u16 len)
{
    i2c_master_transmit_multi_buffer_info_t buf[2];
    //寄存器地址+读写指令
    buf[0].write_buffer=&addr;
    buf[0].buffer_size=1;
    //数据指针
    buf[1].write_buffer=data;
    buf[1].buffer_size=len;    
    //发送
    if(i2c_master_multi_buffer_transmit(dev,buf,2,-1)==ESP_OK)
    {
        return SP_OK;
    }else
    {
        return SP_FAIL;
    }
}

//IIC读寄存器
sp_t sp_iic_read(sp_iic_dev_t dev,u8 addr,u8*data,u16 len)
{
    if(i2c_master_transmit_receive(dev,&addr,1,data,len,-1)==ESP_OK)
    {
        return SP_OK;
    }else
    {
        return SP_FAIL;
    }
}

//IIC检测设备
void sp_iic_check()
{
    u8 addr=0x00;
    //获取端口
    i2c_master_bus_handle_t handle=NULL;
    i2c_master_get_bus_handle(COM,&handle);
    while(1)
    {
        if(i2c_master_probe(handle,addr,-1)==ESP_OK)
        {
            SP_LOG("find addr %x",addr);
        }
        if(addr==255)
        {
            break;
        }else
        {
            ++addr;
        }
    }
}
