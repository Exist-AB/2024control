#include "main.h"
#include "pid.h"
//TIM1的四路通道PE9,PE11,PE13,PE14控制直流电机驱动
//控制方向的GPIO端口为(motor1:PE7,PE8)(motor2:PE10,PE12)(motor3:PE15,PB10)(motor4:PB11,PB12)
//TIM9的通道1和通道2控制摩擦轮无刷电机
//TIM3的四个通道分别控制四个舵机
//TIM2,TIM4,TIM5,TIM8作为编码器的输入
//TIM7计数中断
#define MOTOR_SPEED_RERATIO 45u    //电机减速比
#define PULSE_PRE_ROUND 11 //一圈多少个脉冲
#define RADIUS_OF_TYRE  //轮胎半径，单位毫米
#define LINE_SPEED_C RADIUS_OF_TYRE * 2 * 3.14
#define COUNTERNUM_1 (short)__HAL_TIM_GetCounter(&htim2)
#define COUNTERNUM_2 (short)__HAL_TIM_GetCounter(&htim3)
#define COUNTERNUM_3 (short)__HAL_TIM_GetCounter(&htim4)
#define COUNTERNUM_4 (short)__HAL_TIM_GetCounter(&htim8)
typedef struct _Motor
{
		int32_t lastCount;   //上一次计数值
    int32_t totalCount;  //总计数值
    int16_t overflowNum; //溢出次数
    float speed;         //电机转速
    uint8_t direct;      //旋转方向
}Motor;

extern Motor motor1,motor2,motor3,motor4;
extern char aRxBuffer[8];
/*
void change_speed(int i1,int i2,int i3,int i4,int i5);
void motor_init(void);
void Motor_1_Go(void);
void Motor_1_Back(void);
void Motor_1_Stop(void);
void Motor_2_Go(void);
void Motor_2_Back(void);
void Motor_2_Stop(void);
void Motor_3_Go(void);
void Motor_3_Back(void);
void Motor_3_Stop(void);
void Motor_4_Go(void);
void Motor_4_Back(void);
void Motor_4_Stop(void);
void Motor_Send(void);
void Forward(int32_t length);
void Back(int32_t length);
void Left(int32_t length);
void Right(int32_t length);
void LF(int32_t length);//左前
void LB(int32_t length);//左后
void RF(int32_t length);//右前
void RB(int32_t length);//右后
void Rotate(int32_t length);//旋转
void STOP(void);
void Forward_1(int32_t length);
void Forward_2(int32_t length);
void Forward_3(int32_t length);
void Forward_4(int32_t length);
void Change_out_kp_1(int num);
void Change_out_ki_1(int num);
void Change_out_kd_1(int num);
void Change_in_kp_1(int num);
void Change_in_ki_1(int num);
void Change_in_kd_1(int num);
void Change_max_speed_1(int num);
void Change_out_kp_2(int num);
void Change_out_ki_2(int num);
void Change_out_kd_2(int num);
void Change_in_kp_2(int num);
void Change_in_ki_2(int num);
void Change_in_kd_2(int num);
void Change_max_speed_2(int num);
void Change_out_kp_3(int num);
void Change_out_ki_3(int num);
void Change_out_kd_3(int num);
void Change_in_kp_3(int num);
void Change_in_ki_3(int num);
void Change_in_kd_3(int num);
void Change_max_speed_3(int num);
void Change_out_kp_4(int num);
void Change_out_ki_4(int num);
void Change_out_kd_4(int num);
void Change_in_kp_4(int num);
void Change_in_ki_4(int num);
void Change_in_kd_4(int num);
void Change_max_speed_4(int num);
*/
