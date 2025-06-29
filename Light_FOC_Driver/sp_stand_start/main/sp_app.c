#include"sp_app.h"

//头文件
#include"sp_iic.h"
#include"as5600_driver.h"
#include"te_pwm.h"
#include"foc.h"

//用户应用
void sp_app();

//用户应用
void sp_app()
{
    //串口初始化
    while(sp_uart_init()!=SP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // while(1)
    // {
    //     //从串口输入
    //     char*data=sp_uart_rx();
    //     printf("data=%s\n",data);
    // }

    //IIC初始化
    while(sp_iic_init()!=SP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    //PWM初始化
    while(te_pwm_init()!=SP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    //FOC初始化
    foc_init();

    //设置为定角度模式
    foc_set_mode(FOC_MODE_ANG);
    //设置角度为180度
    foc_set_ang(180);

    // //设置为定速度模式
    // foc_set_mode(FOC_MODE_SPD);
    // //设置速度为100转每分钟
    // foc_set_spd(100);

}

//测试
// foc_test();

// void test(void)
// {


//     //串口初始化
//     while(sp_uart_init()!=SP_OK)
//     {
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
//     // while(1)
//     // {
//     //     //从串口输入
//     //     char*data=sp_uart_rx();
//     //     printf("data=%s\n",data);
//     // }
//     //IIC初始化
//     while(sp_iic_init()!=SP_OK)
//     {
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }

//     //PWM初始化
//     while(te_pwm_init()!=SP_OK)
//     {
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }


//     // //FOC服务初始化
//     // foc_service_init();
//     // //FOC设置目标角度
//     // foc_service_set(50);

//     //FOC初始化
//     foc_init();
    
//     while(1)
//     {
//         foc_test();
//         // te_pwm_set(1,50);
//         // te_pwm_set(2,50);
//         // te_pwm_set(3,50);

//         // SP_LOG("ang=%f",as5600_read());

//         // vTaskDelay(pdMS_TO_TICKS(5));
//     }
// }