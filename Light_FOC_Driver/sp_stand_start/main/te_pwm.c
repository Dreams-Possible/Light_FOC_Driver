#include"te_pwm.h"

//定义
#define IO_1 40
#define IO_2 41
#define IO_3 42

//PWM初始化
sp_t te_pwm_init();
//PWM设置
void te_pwm_set(u8 ch,float duty);

//PWM初始化
sp_t te_pwm_init()
{
    sp_t ret=SP_OK;
    //初始化定时器
    ledc_timer_config_t ledc_timer={0};
    ledc_timer.speed_mode=LEDC_LOW_SPEED_MODE;
    ledc_timer.duty_resolution=LEDC_TIMER_10_BIT;
    ledc_timer.timer_num=LEDC_TIMER_0;
    ledc_timer.freq_hz=20000;
    ledc_timer.clk_cfg=LEDC_AUTO_CLK;
    //初始化通道
    ret=ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel={0};
    ledc_channel.timer_sel=LEDC_TIMER_0;
    ledc_channel.channel=LEDC_CHANNEL_0;
    ledc_channel.gpio_num=IO_1;
    ledc_channel.speed_mode=LEDC_LOW_SPEED_MODE;
    ret=ledc_channel_config(&ledc_channel);
    ledc_channel.channel=LEDC_CHANNEL_1;
    ledc_channel.gpio_num=IO_2;
    ret=ledc_channel_config(&ledc_channel);
    ledc_channel.channel=LEDC_CHANNEL_2;
    ledc_channel.gpio_num=IO_3;
    ret=ledc_channel_config(&ledc_channel);
    if(ret==ESP_OK)
    {
        SP_LOG("init ok");
        return SP_OK;
    }else
    {
        SP_LOG("init fail");
        return SP_FAIL;
    }
}

//PWM设置
void te_pwm_set(u8 ch,float duty)
{
    //限位
    if(duty>1)
    {
        duty=1;
    }
    if(duty<0)
    {
        duty=0;
    }
    //设置占空比
    switch(ch)
    {
        case 1:
            ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,duty*1024);
            ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);
        break;
        case 2:
            ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1,duty*1024);
            ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1);
        break;
        case 3:
            ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_2,duty*1024);
            ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_2);
        break;
        default:
        break;
    }
}
