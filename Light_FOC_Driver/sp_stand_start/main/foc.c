#include"foc.h"

//定义
#define ANG_ERR 32.607189//电角度补偿（°）
#define POLE_NUM 7//极对数
#define INT_TIME 10//间隔时间（ms）
#define ANG_KP 0.008//角度控制P参数
#define ANG_KI 0.00004//角度控制I参数
#define ANG_KD 0.02//角度控制D参数
// #define SPD_KP 0.0008//速度控制P参数
#define SPD_KP 0.004//速度控制P参数
#define SPD_KI 0.0002//速度控制I参数
#define SPD_KD 0.002//速度控制D参数
#define SPD_MAX 300//最大速度
#define LOG_PRT printf//日志打印

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
//FOC硬件基本测试
void foc_hd_test();
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
    //     // LOG_PRT("show:%f,%f,%f,%f\n",a,b,c,angle_real());
    //     LOG_PRT("angle=%f\n",angle_real());
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
    //采样补偿
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
    foc_data.angle_real=0.8*angle_now+0.2*ang_last;
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
    // LOG_PRT("pwm/angle:%f,%f,%f,%f\n",a*100,b*100,c*100,angle_real());
    pwm_out(a,b,c);
}

//FOC硬件基本测试
void foc_hd_test()
{
    float theta=0;
    float step=32*M_PI/180;
    while(1)
    {
        float a=0.5+0.5*sin(theta);
        float b=0.5+0.5*sin(theta-2*M_PI/3);
        float c=0.5+0.5*sin(theta+2*M_PI/3);
        pwm_out(a,b,c);
        LOG_PRT("pwm:%f,%f,%f\n",a*100,b*100,c*100);
        theta+=step;
        if(theta>=2*M_PI)
        {
            theta-=2*M_PI;
        }
        delay_ms(20);
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
                float ang_p=foc_data.angle_target-angle_real();
                //处理临界角度
                if(ang_p<-180)
                {
                    ang_p+=360;
                }
                // LOG_PRT("ang_p=%f\n",ang_p);
                // LOG_PRT("ang:%f\n",angle_real());
                //累计误差
                static float ang_i=0;
                //积分分离
                if(fabs(ang_p)<10)
                {
                    ang_i+=ang_p;
                }
                //积分重置
                if(fabs(ang_p)<0.4)
                {
                    ang_i/=2;
                }
                //预测趋势
                static float ang_p_last=0;
                float ang_d=ang_p-ang_p_last;
                ang_p_last=ang_p;
                // printf("ang_d=%f",ang_d);
                //转动驱动
                // float force=ANG_KP*ang_p+ANG_KI*ang_i;
                float force=ANG_KP*ang_p+ANG_KI*ang_i+ANG_KD*ang_d;
                // float force=ANG_KP*ang_p+ANG_KD*ang_d;
                // float force=ANG_KP*ang_e;
                turn_drive(force);
            break;
            //速度模式
            case 1:
                //计算误差
                float spd_p=foc_data.speed_target-foc_data.speed_real;
                //累计误差
                static float spd_i=0;
                //积分分离
                if(fabs(spd_p)>0.4)
                {
                    spd_i+=spd_p;
                }
                //预测趋势
                static float spd_p_last=0;
                float spd_d=spd_p-spd_p_last;
                spd_p_last=spd_p;
                // float froce=SPD_KP*spd_p;
                // float froce=0;
                float froce=SPD_KP*spd_p+SPD_KI*spd_i+SPD_KD*spd_d;
                // LOG_PRT("spd:%f\n",foc_data.speed_real);
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
    //限位
    while(angle>360)
    {
        angle-=360;
    }
    while(angle<-360)
    {
        angle+=360;
    }
    foc_data.angle_target=angle;
}

//FOC设置速度（rpm）
void foc_set_spd(float speed)
{
    //限位
    if(speed>SPD_MAX)
    {
        speed=SPD_MAX;
    }
    if(speed<-SPD_MAX)
    {
        speed=-SPD_MAX;
    }
    foc_data.speed_target=speed;
}


        //  //角度模式
        //     case 0:
        //         //计算误差
        //         float ang_p=foc_data.angle_target-angle_real();
        //         //处理临界角度
        //         if(ang_p<-180)
        //         {
        //             ang_p+=360;
        //         }
        //         // LOG_PRT("ang_p=%f\n",ang_p);
        //         // LOG_PRT("ang:%f\n",angle_real());
        //         //累计误差
        //         static float ang_i=0;
        //         //积分分离
        //         if(fabs(ang_p)<10)
        //         {
        //             ang_i+=ang_p;
        //         }
        //         //积分重置
        //         if(fabs(ang_p)<0.4)
        //         {
        //             ang_i/=2;
        //         }
        //         //预测趋势
        //         static float ang_p_last=0;
        //         float ang_d_curr=ang_p-ang_p_last;
        //         ang_p_last=ang_p;
        //         //差分滤波
        //         static float last_d=0;
        //         float ang_d=0.8*ang_d_curr+0.2*last_d;
        //         last_d=ang_d_curr;
        //         // printf("ang_d=%f",ang_d);
        //         //转动驱动
        //         // float force=ANG_KP*ang_p+ANG_KI*ang_i;
        //         float force=ANG_KP*ang_p+ANG_KI*ang_i+ANG_KD*ang_d;
        //         // float force=ANG_KP*ang_p+ANG_KD*ang_d;
        //         // float force=ANG_KP*ang_e;
        //         turn_drive(force);
        //     break;

//速度模式
            // case 1:
            //     //力矩
            //     static float froce=0;
            //     //计算误差
            //     float spd_p=SPD_KP*(foc_data.speed_target-foc_data.speed_real);


                
            //     froce+=spd_p;
            //     // LOG_PRT("spd:%f\n",foc_data.speed_real);
            //     //转动驱动
            //     turn_drive(froce);
            // break;


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

                    // //敏感响应调整
                // float force=ANG_KP*ang_e;
                // if(force>FORCE_SENS_MIN&&force<FORCE_SENS_MAX)
                // {
                //     force=FORCE_SENS_MAX;
                // }
                // if(force<-FORCE_SENS_MIN&&force>-FORCE_SENS_MAX)
                // {
                //     force=-FORCE_SENS_MAX;
                // }


                
// //FOC服务
// static void foc_service(void*arg)
// {
//     while(1)
//     {
//         //状态更新
//         state_update();
//         //控制模式
//         switch(foc_data.mode)
//         {
//             //角度模式
//             case 0:
//                 //计算误差
//                 float ang_e=foc_data.angle_target-angle_real();
//                 //处理临界角度
//                 if(ang_e<-180)
//                 {
//                     ang_e+=360;
//                 }
//                 // LOG_PRT("ang_e=%f\n",ang_e);
//                 // LOG_PRT("ang:%f\n",angle_real());
//                 //累计误差
//                 static float err_i=0;
//                 // //积分分离
//                 // if(fabs(ang_e)<90)
//                 // {
//                 //     err_i+=ang_e;
//                 // }
//                 err_i+=ang_e;
//                 //积分限幅
//                 // if(err_i<-ANG_LI)
//                 // {
//                 //     err_i=-ANG_LI;
//                 // }
//                 // if(err_i>ANG_LI)
//                 // {
//                 //     err_i=ANG_LI;
//                 // }
//                 if(err_i<-2000)
//                 {
//                     err_i=-2000;
//                 }
//                 if(err_i>2000)
//                 {
//                     err_i=2000;
//                 }
//                 // //积分重置
//                 // if(fabs(ang_e)<2)
//                 // {
//                 //     err_i/=2;
//                 // }
//                 //预测趋势
//                 static float ang_p_last=0;
//                 float ang_d_curr=ang_e-ang_p_last;
//                 ang_p_last=ang_e;
//                 //差分滤波
//                 static float last_d=0;
//                 float err_d=0.8*ang_d_curr+0.2*last_d;
//                 last_d=ang_d_curr;
//                 // printf("err_d=%f",err_d);
//                 //转动驱动
//                 // float force=ANG_KP*ang_e+ANG_KI*err_i;
//                 float force=ANG_KP*ang_e+ANG_KI*err_i+ANG_KD*err_d;
//                 // float force=ANG_KP*ang_e+ANG_KD*err_d;
//                 // float force=ANG_KP*ang_e;
//                 turn_drive(force);
//             break;
//             //速度模式
//             case 1:
//                 //力矩
//                 static float froce=0;
//                 //计算误差
//                 float spd_p=SPD_KP*(foc_data.speed_target-foc_data.speed_real);
//                 froce+=spd_p;
//                 // LOG_PRT("spd:%f\n",foc_data.speed_real);
//                 //转动驱动
//                 turn_drive(froce);
//             break;
//             default:
//             break;
//         }
//         delay_ms(INT_TIME);
//     }
// }



// //定义
// #define ANG_ERR 32.607189//电角度补偿（°）
// #define POLE_NUM 7//极对数
// #define INT_TIME 5//间隔时间（ms）
// #define ANG_KP 0.008//角度控制P参数
// #define ANG_KI 0.0002//角度控制I参数
// // #define ANG_LI 4000//角度控制I限幅
// // #define ANG_KD 0.08//角度控制D参数
// #define ANG_KD 0.02//角度控制D参数
// // #define SPD_KP 0.0004//速度控制PID参数
// #define SPD_KP 0.0008//速度控制P参数
// // #define FORCE_SENS_MIN 0.1//最小敏感响应力矩阈值
// // #define FORCE_SENS_MAX 0.6//最大敏感响应力矩阈值
// #define LOG_PRT printf//日志打印

