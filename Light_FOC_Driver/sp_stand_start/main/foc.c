#include"foc.h"

//读取实际角度（360°）
static float angle_real();
//输出PWM（100%）
static void pwm_out(float a,float b,float c);
//延时（ms）
static void delay_ms(uint16_t time);
// //克拉克变换（abc->αβ）
// static void clarke_trans(float a,float b,float c,float*alpha,float*beta);
// //帕克变换（αβ->dq）
// static void park_trans(float alpha,float beta,float theta,float*d,float*q);
//克拉克逆变换（αβ->abc）
static void clarke_invtrans(float alpha,float beta,float*a,float*b,float*c);
//帕克逆变换（dq->αβ）
static void park_invtrans(float d,float q,float theta,float*alpha,float*beta);
//状态更新
static void state_update();
//转动驱动
static void turn_drive(float force);
//FOC服务
static void foc_service(void*arg);
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

//定义
#define ANG_ERR 32.167740//电角度补偿（°）
#define POLE_NUM 7//极对数
#define INT_TIME 5//间隔时间（ms）
#define ANG_KP 0.004//角度控制PID参数
#define SPD_KP 0.0004//速度控制PID参数
#define LOG_PRT printf//日志打印

//FOC数据
typedef struct foc_data_t
{
    uint8_t mode;//控制模式（角度、速度）
    float angle_target;//目标角度（°）
    float speed_target;//目标速度（rpm）
    float angle_real;//真实角度（°）
    float speed_real;//真实速度（rpm）
}foc_data_t;
static foc_data_t foc_data={0};

//读取实际角度（360°）
static float angle_real()
{
    return as5600_read();
}

//输出PWM（100%）
static void pwm_out(float a,float b,float c)
{
    te_pwm_set(1,a);
    te_pwm_set(2,b);
    te_pwm_set(3,c);
    return;
}

//延时（ms）
static void delay_ms(uint16_t time)
{
    vTaskDelay(pdMS_TO_TICKS(time));
    return;
}

// //克拉克变换（abc->αβ）
// static void clarke_trans(float a,float b,float c,float*alpha,float*beta)
// {
//     *alpha=a;
//     *beta=(1/sqrt(3))*(a+2*b);
// }

// //帕克变换（αβ->dq）
// static void park_trans(float alpha,float beta,float theta,float*d,float*q)
// {
//     float cos_theta=cos(theta);
//     float sin_theta=sin(theta);
//     *d=alpha*cos_theta+beta*sin_theta;
//     *q=-alpha*sin_theta+beta*cos_theta;
// }

//克拉克逆变换（αβ->abc）
static void clarke_invtrans(float alpha,float beta,float*a,float*b,float*c)
{
    *a=alpha;
    *b=-0.5*alpha+(sqrt(3)/2)*beta;
    *c=-0.5*alpha-(sqrt(3)/2)*beta;
}

//帕克逆变换（dq->αβ）
static void park_invtrans(float d,float q,float theta,float*alpha,float*beta)
{
    float cos_theta=cos(theta);
    float sin_theta=sin(theta);
    *alpha=d*cos_theta-q*sin_theta;
    *beta=d*sin_theta+q*cos_theta;
}

//FOC初始化
void foc_init()
{
    // //校准电角度（请先借助此函数确定电角度补偿。将电机旋转，电机会依次吸附在最近的电角度零点（吸附次数由磁对数决定），转完一圈，记录传感器最小的角度，即为电角度补偿，填入宏定义中即可）
    // while(1)
    // {
    //     //帕克逆变换（dq->αβ）
    //     float d=0.8;
    //     float q=0;
    //     float theta=0;
    //     float alpha=0;
    //     float beta=0;
    //     park_invtrans(d,q,theta,&alpha,&beta);
    //     //归一化
    //     alpha/=sqrt(2);
    //     //克拉克逆变换（αβ->abc）
    //     float a=0;
    //     float b=0;
    //     float c=0;
    //     clarke_invtrans(alpha,beta,&a,&b,&c);
    //     //归一化
    //     a=(a+1)/2;
    //     b=(b+1)/2;
    //     c=(c+1)/2;
    //     //输出PWM
    //     pwm_out(a,b,c);
    //     sp_uart_tx("show:%f,%f,%f,%f\n",a,b,c,angle_real());
    //     delay_ms(100);
    // }
    //启动FOC服务
    xTaskCreate(foc_service,"foc_service",1024*4,NULL,8,NULL);
}

//状态更新
static void state_update()
{
    static float ang_last=0;
    float angle_now=angle_real();
    // LOG_PRT("ang=%f\n",angle_now);
    float theta=angle_now-ang_last;
    //高转速低频采样单圈补偿
    while(theta>=180)
    {
        theta-=360;
    }
    while(theta<-180)
    {
        theta+=360;
    }
    //速度（rpm）
    float speed=theta/(INT_TIME/1000.0)/360*60;
    //更新实时角度和速度
    foc_data.angle_real=angle_now;
    foc_data.speed_real=speed;
    ang_last=angle_now;
}

//转动驱动
static void turn_drive(float force)
{
    //限幅
    if(force>1)
    {
        force=1;
    }
    if(force<-1)
    {
        force=-1;
    }
    //帕克逆变换（dq->αβ）
    float d=0;
    float q=force;
    float theta=(angle_real()-ANG_ERR)*POLE_NUM*M_PI/180;
    //归一化
    while(theta>=360*POLE_NUM)
    {
        theta-=360*POLE_NUM;
    }
    while(theta<0)
    {
        theta+=360*POLE_NUM;
    }
    float alpha=0;
    float beta=0;
    park_invtrans(d,q,theta,&alpha,&beta);
    //归一化
    alpha/=sqrt(2);
    //克拉克逆变换（αβ->abc）
    float a=0;
    float b=0;
    float c=0;
    clarke_invtrans(alpha,beta,&a,&b,&c);
    //归一化
    a=(a+1)/2;
    b=(b+1)/2;
    c=(c+1)/2;
    //输出PWM
    // LOG_PRT("pwm:%f,%f,%f,%f\n",a,b,c,theta);
    pwm_out(a,b,c);
}

//FOC硬件基本测试
void foc_hd_test()
{
    float theta=0;
    float step=8*M_PI/180;
    while(1)
    {
        float a=0.5+0.5*sin(theta);
        float b=0.5+0.5*sin(theta-2*M_PI/3);
        float c=0.5+0.5*sin(theta+2*M_PI/3);
        pwm_out(a,b,c);
        // LOG_PRT("pwm:%f,%f,%f\n",a,b,c);
        theta+=step;
        if(theta>=2*M_PI)
        {
            theta-=2*M_PI;
        }
        delay_ms(INT_TIME);
    }
}

//FOC服务
static void foc_service(void*arg)
{
    while(1)
    {
        //状态更新
        state_update();
        //控制模式
        switch(foc_data.mode)
        {
            //角度模式
            case 0:
                //计算误差
                float ang_e=foc_data.angle_target-angle_real();
                // //总是选择最短回归方向
                // if(ang_e<-180)
                // {
                //     ang_e+=180;
                //     ang_e=-ang_e;
                // }
                // if(ang_e>180)
                // {
                //     ang_e-=180;
                //     ang_e=-ang_e;
                // }
                //处理临界角度
                if(ang_e<-180)
                {
                    ang_e+=360;
                }
                // LOG_PRT("ang_e=%f\n",ang_e);
                LOG_PRT("ang:%f\n",angle_real());
                //转动驱动
                turn_drive(ANG_KP*ang_e);
            break;
            //速度模式
            case 1:
                //力矩
                static float froce=0;
                //计算误差
                float spd_e=SPD_KP*(foc_data.speed_target-foc_data.speed_real);
                froce+=spd_e;
                //限幅
                if(froce>1)
                {
                    froce=1;
                }
                if(froce<-1)
                {
                    froce=-1;
                }
                LOG_PRT("spd:%f\n",foc_data.speed_real);
                //转动驱动
                turn_drive(froce);
            break;
            default:
            break;
        }
        delay_ms(INT_TIME);
    }
}

//FOC设置模式
void foc_set_mode(uint8_t mode)
{
    switch(mode)
    {
        case FOC_MODE_ANG:
            foc_data.mode=FOC_MODE_ANG;
        break;
        case FOC_MODE_SPD:
            foc_data.mode=FOC_MODE_SPD;
        break;
        default:
        break;
    }
}

//FOC读取角度（°）
float foc_read_ang()
{
    return foc_data.angle_real;
}

//FOC读取速度（rpm）
float foc_read_spd()
{
    return foc_data.speed_real;
}

//FOC设置角度（°）
void foc_set_ang(float angle)
{
    foc_data.angle_target=angle;
}

//FOC设置速度（rpm）
void foc_set_spd(float speed)
{
    foc_data.speed_target=speed;
}




// //测试
// void foc_test()
// {
//     // foc_hd_test();
//     while(1)
//     {
//         //电角度
//         // float angle=(angle_real()-ANG_ERR)*7;
//         // //归一化
//         // while(angle>=360*7)
//         // {
//         //     angle-=360*7;
//         // }
//         // while (angle<0)
//         // {
//         //     angle+=360*7;
//         // }
//         // turn_drive(0.6,angle*M_PI/180);

//         //状态更新
//         state_update();
//         LOG_PRT("spd:%f\n",foc_read_spd());
//         //转动驱动
//         turn_drive(0.8);

//         // pwm_out(0.2,0.4,0.6);
//         delay_ms(INT_TIME);

//     }
// }



    // float ang=0;
    // while(1)
    // {
    //     ang+=0.1;
    //     if(ang>360)
    //     {
    //         ang=0;
    //     }
    //     //限幅
    //     if(force>1)
    //     {
    //         force=1;
    //     }
    //     if(force<-1)
    //     {
    //         force=-1;
    //     }
    //     //帕克逆变换（dq->αβ）
    //     float d=0;
    //     float q=force;
    //     float theta=ang*7*M_PI/180;
    //     float alpha=0;
    //     float beta=0;
    //     park_invtrans(d,q,theta,&alpha,&beta);
    //     //归一化
    //     alpha/=sqrt(2);
    //     //克拉克逆变换（αβ->abc）
    //     float a=0;
    //     float b=0;
    //     float c=0;
    //     clarke_invtrans(alpha,beta,&a,&b,&c);
    //     //归一化
    //     a=(a+1)/2;
    //     b=(b+1)/2;
    //     c=(c+1)/2;
    //     //输出PWM
    //     pwm_out(1,a);
    //     pwm_out(2,b);
    //     pwm_out(3,c);
    //     LOG_PRT("show:%f,%f,%f,%f,%f\n",a,b,c,ang,angle_real());
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }