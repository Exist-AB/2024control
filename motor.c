#include "motor.h"
#include "tim.h"
#include "usart.h"
Motor motor1,motor2,motor3,motor4;
char aRxBuffer[8];
int dir;
void motor_init(void)
{
	   motor1.lastCount = 0;                                   //结构体内容初始化
     motor1.totalCount = 0;
     motor1.overflowNum = 0;                                  
     motor1.speed = 0;
     motor1.direct = 0;
		 motor2.lastCount = 0;                                   
     motor2.totalCount = 0;
     motor2.overflowNum = 0;                                  
     motor2.speed = 0;
     motor2.direct = 0;
	   motor3.lastCount = 0;                                   
     motor3.totalCount = 0;
     motor3.overflowNum = 0;                                  
     motor3.speed = 0;
     motor3.direct = 0;
		 motor4.lastCount = 0;                                   
     motor4.totalCount = 0;
     motor4.overflowNum = 0;                                  
     motor4.speed = 0;
     motor4.direct = 0;
	
	
	   HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1|TIM_CHANNEL_2);
	   HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1|TIM_CHANNEL_2);
	   HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_1|TIM_CHANNEL_2);
	   HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_1|TIM_CHANNEL_2);
		 __HAL_TIM_SET_COUNTER(&htim2,32000);
	   __HAL_TIM_SET_COUNTER(&htim4,32000);
	   __HAL_TIM_SET_COUNTER(&htim5,32000);
	   __HAL_TIM_SET_COUNTER(&htim8,32000);//编码器读取初始化
		 
	   HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	   HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	   HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	   HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//直流有刷电机
		 
	   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);//舵机
		 
		 HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
	   HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);//摩擦轮
		 
		 __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,);
	   __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,);
	   __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,);
	   __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,);//舵机姿态初始化
		 
	   PID_Init(&motor1.anglePID.inner, 4, 0.8, 0.01, 3000, 10000);
	   PID_Init(&motor1.anglePID.outer, 20, 0, 0, 100, 200);
	   PID_Init(&motor2.anglePID.inner, 4, 0.8, 0.01, 3000, 10000);
	   PID_Init(&motor2.anglePID.outer, 20, 0, 0, 100, 200);
	   PID_Init(&motor3.anglePID.inner, 4, 0.8, 0.01, 3000, 10000);
	   PID_Init(&motor3.anglePID.outer, 20, 0, 0, 100, 200);
	   PID_Init(&motor4.anglePID.inner, 4, 0.8, 0.01, 3000, 10000);
	   PID_Init(&motor4.anglePID.outer, 20, 0, 0, 100, 200);//PID参数调整
		 
	   HAL_TIM_Base_Start_IT(&htim7);开计数器
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == (&htim7))//50ms一次
	{
		if(COUNTERNUM_1 < 32000-30000 )
			{
			motor1.overflowNum--;
			__HAL_TIM_SetCounter(&htim2,32000);
			}
		else if(COUNTERNUM_1 > 32000+30000 )
			{
			motor1.overflowNum++;
			__HAL_TIM_SetCounter(&htim2,32000);
			}
		if(COUNTERNUM_2 < 32000-30000 )
			{
			motor2.overflowNum--;
			__HAL_TIM_SetCounter(&htim4,32000);
			}
		else if(COUNTERNUM_2 > 32000+30000 )
			{
			motor2.overflowNum++;
			__HAL_TIM_SetCounter(&htim4,32000);
			}
		if(COUNTERNUM_3 < 32000-30000 )
			{
			motor3.overflowNum--;
			__HAL_TIM_SetCounter(&htim5,32000);
			}
		else if(COUNTERNUM_3 > 32000+30000 )
			{
			motor3.overflowNum++;
			__HAL_TIM_SetCounter(&htim5,32000);
			}
		if(COUNTERNUM_4 < 32000-30000 )
			{
			motor4.overflowNum--;
			__HAL_TIM_SetCounter(&htim4,32000);
			}
		else if(COUNTERNUM_4 > 32000+30000 )
			{
			motor4.overflowNum++;
			__HAL_TIM_SetCounter(&htim4,32000);
			}
		motor1.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);//如果向上计数（正转），返回值为0，否则返回值为1
		motor1.totalCount = COUNTERNUM_1 + motor1.overflowNum * 30000;//一个周期内的总计数值等于目前计数值加上溢出的计数值
		motor1.speed = (float)(motor1.totalCount - motor1.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 20;//算得每秒多少转
		//motor1.speed = (float)(motor1.totalCount - motor1.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 20 * LINE_SPEED_C//算得车轮线速度每秒多少毫米
		motor1.lastCount = motor1.totalCount; //记录这一次的计数值
		motor2.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);//如果向上计数（正转），返回值为0，否则返回值为1
		motor2.totalCount = COUNTERNUM_2 + motor2.overflowNum * 30000;//一个周期内的总计数值等于目前计数值加上溢出的计数值
		motor2.speed = (float)(motor2.totalCount - motor2.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 20;//算得每秒多少转
		//motor2.speed = (float)(motor2.totalCount - motor2.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 20 * LINE_SPEED_C//算得车轮线速度每秒多少毫米
		motor2.lastCount = motor2.totalCount; //记录这一次的计数值
		motor3.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5);//如果向上计数（正转），返回值为0，否则返回值为1
		motor3.totalCount = COUNTERNUM_3 + motor3.overflowNum * 30000;//一个周期内的总计数值等于目前计数值加上溢出的计数值
		motor3.speed = (float)(motor3.totalCount - motor3.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 20;//算得每秒多少转
		//motor3.speed = (float)(motor3.totalCount - motor3.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 20 * LINE_SPEED_C//算得车轮线速度每秒多少毫米
		motor3.lastCount = motor3.totalCount; //记录这一次的计数值
		motor4.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim8);//如果向上计数（正转），返回值为0，否则返回值为1
		motor4.totalCount = COUNTERNUM_4 + motor4.overflowNum * 30000;//一个周期内的总计数值等于目前计数值加上溢出的计数值
		motor4.speed = (float)(motor4.totalCount - motor4.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 20;//算得每秒多少转
		//motor4.speed = (float)(motor4.totalCount - motor4.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 20 * LINE_SPEED_C//算得车轮线速度每秒多少毫米
		motor4.lastCount = motor4.totalCount; //记录这一次的计数值
		Motor_Send();
		lwqf =  (uint16_t)GPIOD->IDR & 0x001f; //forward
		lwqb =  ((uint16_t)GPIOB->IDR >> 4)& 0x001f; //back
		lwqlr =  (uint16_t)GPIOF->IDR & 0x03ff; //right
		temp1 = lwqlr;//前五位是左边，后五位是右边
		temp3 = lwqb;//后面
		temp2 = lwqf;//前面
		if(dir == 5)
		{
			motor1.anglePID.outer.maxOutput = 203;
			motor2.anglePID.outer.maxOutput = 200;
			motor3.anglePID.outer.maxOutput = 203;
			motor4.anglePID.outer.maxOutput = 200;
		}
		if(dir == 0)
				{
					motor1.anglePID.outer.maxOutput = 100; //gai
					motor2.anglePID.outer.maxOutput = 100;
					motor3.anglePID.outer.maxOutput = 100;
					motor4.anglePID.outer.maxOutput = 100;
					motor1.targetAngle = motor1.totalAngle;
					motor2.targetAngle = motor2.totalAngle;
				    motor3.targetAngle = motor3.totalAngle;
				    motor4.targetAngle = motor4.totalAngle;
				}
		if(dir == 1)//you
		{
			int hou=(temp1 & 0b0001000010);
			int qian=(temp1 & 0b0100001000);
			int shun=(temp1 & 0b0001001000);
			int ni=(temp1 & 0b0100000010);
			int right1 = (temp1 == 0b1101111011);
			int right2 = (temp1 == 0b1000110001);
		if(right1 == 1 || right2 == 1)
		{
			motor1.anglePID.outer.maxOutput = 100;
			motor2.anglePID.outer.maxOutput = 100;
			motor3.anglePID.outer.maxOutput = 100;
			motor4.anglePID.outer.maxOutput = 100;
		}
		else
		{
			if(hou == 0) {
						change_speed(-10,10,10,-10,2);
						}
					if(qian== 0){
						change_speed(10,-10,-10,10,2);
					}
					if(ni == 0){
						change_speed(-10,-10,10,10,2);
					}
					if(shun == 0){
						change_speed(10,10,-10,-10,2);
					}
		}
		}
		if(dir == 2)//zuo
		{
			int hou=(temp1 & 0b0001000010);
			int qian=(temp1 & 0b0100001000);
			int shun=(temp1 & 0b0001001000);
			int ni=(temp1 & 0b0100000010);
			int right1 = (temp1 == 0b1101111011);
			int right2 = (temp1 == 0b1000110001);
		if(right1 == 1 || right2 == 1)
		{
			motor1.anglePID.outer.maxOutput = 100;
			motor2.anglePID.outer.maxOutput = 100;
			motor3.anglePID.outer.maxOutput = 100;
			motor4.anglePID.outer.maxOutput = 100;
		}
		else
		{
			if(hou == 0) {
						change_speed(10,-10,-10,10,2);
						}
					if(qian== 0){
						change_speed(-10,10,10,-10,2);
					}
					if(ni == 0){
						change_speed(10,10,-10,-10,2);
					}
					if(shun == 0){
						change_speed(-10,-10,10,10,2);
					}
		}
		}
		if(dir == 3)
		{
			int zuo = (temp2 & 0b01000);
			int you = (temp2 & 0b00010);
			int right1 = (temp2 == 0b11011);
			int right2 = (temp2 == 0b10001);
			if(right1 == 1 || right2 == 1)
			{
				motor1.anglePID.outer.maxOutput = 100;//
				motor2.anglePID.outer.maxOutput = 100;
				motor3.anglePID.outer.maxOutput = 100;
				motor4.anglePID.outer.maxOutput = 100;
			}
			else
				{
			if(zuo == 0)
			{
				change_speed(10,-10,-10,10,2);
			}
			if(you == 0)
			{
				change_speed(-10,10,10,-10,2);
			}
				}
		}
		if(dir == 4)
		{
			int zuo = (temp2 & 0b01000);
			int you = (temp2 & 0b00010);
			int right1 = (temp2 == 0b11011);
			int right2 = (temp2 == 0b10001);
			if(right1 == 1 || right2 == 1)
			{
				motor1.anglePID.outer.maxOutput = 100;//
				motor2.anglePID.outer.maxOutput = 100;
				motor3.anglePID.outer.maxOutput = 100;
				motor4.anglePID.outer.maxOutput = 100;
			}
			else
			{
				if(zuo == 0)
				{
					change_speed(10,-10,-10,10,2);
				}
				if(you == 0)
				{
					change_speed(10,-10,-10,10,2);
				}
			}
		}
		if(dir == 6)
		{
			motor1.targetAngle = motor1.totalAngle;
			motor2.targetAngle = motor2.totalAngle;
			motor3.targetAngle = motor3.totalAngle;
			motor4.targetAngle = motor4.totalAngle;
		}
		if(dir == 7)
		{
			motor1.anglePID.outer.maxOutput = 100;
			motor2.anglePID.outer.maxOutput = 100;
			motor3.anglePID.outer.maxOutput = 100;
			motor4.anglePID.outer.maxOutput = 100;
		}
	}
}



void Motor_Send(void)
{
	if( motor1.targetAngle - motor1.totalAngle > 10 || motor1.targetAngle - motor1.totalAngle < -10 )
	{
		float output1 = 0;
			PID_CascadeCalc(&motor1.anglePID, motor1.targetAngle, motor1.totalAngle, motor1.speed);
			//PID_SingleCalc(&motor1.anglePID.inner,motor1.targetSpeed,motor1.speed);//单环调速用
			output1 = motor1.anglePID.inner.output;
			if(output1 > 0 )
			{
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint32_t)output1);
				Motor_1_Go();
			}
			else
			{
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint32_t)(-output1));
				Motor_1_Back();
			}
	}
	else
	{
		stoppoint=0;//clean
		stoppoint1=0;
		Motor_1_Stop();
	}
	if( motor2.targetAngle - motor2.totalAngle > 10 || motor2.targetAngle - motor2.totalAngle < -10 )
		{
			float output2 = 0;
				PID_CascadeCalc(&motor2.anglePID, motor2.targetAngle, motor2.totalAngle, motor2.speed);
				//PID_SingleCalc(&motor2.anglePID.inner,motor2.targetSpeed,motor2.speed);
				output2 = motor2.anglePID.inner.output;
				if(output2 > 0 )
				{
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, (uint32_t)output2);
					Motor_2_Go();
				}
				else
				{
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, (uint32_t)(-output2));
					Motor_2_Back();
				}
		}
		else
		{
			stoppoint=0;
			stoppoint1=0;
			Motor_2_Stop();
		}
	if( motor3.targetAngle - motor3.totalAngle > 10 || motor3.targetAngle - motor3.totalAngle < -10 )
		{
			float output3 = 0;
				PID_CascadeCalc(&motor3.anglePID, motor3.targetAngle, motor3.totalAngle, motor3.speed);
				//PID_SingleCalc(&motor3.anglePID.inner,motor3.targetSpeed,motor3.speed);
				output3 = motor3.anglePID.inner.output;
				if(output3 > 0 )
				{
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, (uint32_t)output3);
					Motor_3_Go();
				}
				else
				{
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, (uint32_t)(-output3));
					Motor_3_Back();
				}
		}
		else
		{
			stoppoint=0;//clean
			stoppoint1=0;
			Motor_3_Stop();
		}
	if( motor4.targetAngle - motor4.totalAngle > 10 || motor4.targetAngle - motor4.totalAngle < -10 )
		{
			float output4 = 0;
				PID_CascadeCalc(&motor4.anglePID, motor4.targetAngle, motor4.totalAngle, motor4.speed);
				//PID_SingleCalc(&motor4.anglePID.inner,motor4.targetSpeed,motor4.speed);
				output4 = motor4.anglePID.inner.output;
				if(output4 > 0 )
				{
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, (uint32_t)output4);
					Motor_4_Go();
				}
				else
				{
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, (uint32_t)(-output4));
					Motor_4_Back();
				}
		}
		else
		{
			stoppoint=0;//clean
			stoppoint1=0;
			Motor_4_Stop();
		}

}

void Motor_1_Go(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
}

void Motor_1_Back(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
}

void Motor_1_Stop(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
}

void Motor_2_Go(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
}

void Motor_2_Back(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
}

void Motor_2_Stop(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
}

void Motor_3_Go(void)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,GPIO_PIN_SET);
}

void Motor_3_Back(void)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_SET);
}

void Motor_3_Stop(void)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,GPIO_PIN_RESET);
}

void Motor_4_Go(void)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_SET);
}

void Motor_4_Back(void)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_SET);
}

void Motor_4_Stop(void)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_RESET);
}

void Forward(int32_t length)
{
	motor1.targetAngle =motor1.totalAngle + length;
	motor2.targetAngle =motor2.totalAngle + length;
	motor3.targetAngle =motor3.totalAngle + length;
	motor4.targetAngle =motor4.totalAngle + length;
}

void Back(int32_t length)
{
	motor1.targetAngle =motor1.totalAngle - length;
	motor2.targetAngle =motor2.totalAngle - length;
	motor3.targetAngle =motor3.totalAngle - length;
	motor4.targetAngle =motor4.totalAngle - length;
}

void Left(int32_t length)
{
	motor1.targetAngle =motor1.totalAngle - length;
	motor2.targetAngle =motor2.totalAngle + length;
	motor3.targetAngle =motor3.totalAngle + length;
	motor4.targetAngle =motor4.totalAngle - length;
}

void Right(int32_t length)
{

	motor1.targetAngle =motor1.totalAngle + length;
	motor2.targetAngle =motor2.totalAngle - length;
	motor3.targetAngle =motor3.totalAngle - length;
	motor4.targetAngle =motor4.totalAngle + length;
}

void LF(int32_t length)
{
	motor1.targetAngle =motor1.totalAngle + length;
	motor2.targetAngle =motor2.totalAngle;
	motor3.targetAngle =motor3.totalAngle;
	motor4.targetAngle =motor4.totalAngle + length;
}

void LB(int32_t length)
{
	motor1.targetAngle =motor1.totalAngle ;
	motor2.targetAngle =motor2.totalAngle - length;
	motor3.targetAngle =motor3.totalAngle - length;
	motor4.targetAngle =motor4.totalAngle ;
}

void RF(int32_t length)
{
	motor1.targetAngle =motor1.totalAngle ;
	motor2.targetAngle =motor2.totalAngle + length;
	motor3.targetAngle =motor3.totalAngle + length;
	motor4.targetAngle =motor4.totalAngle ;
}

void RB(int32_t length)
{
	motor1.targetAngle =motor1.totalAngle ;
	motor2.targetAngle =motor2.totalAngle - length;
	motor3.targetAngle =motor3.totalAngle - length;
	motor4.targetAngle =motor4.totalAngle ;
}

void STOP(void)
{
	if(dir == 1)
	{
		motor1.targetAngle = motor1.totalAngle-20;
		motor2.targetAngle = motor2.totalAngle-20;
		motor3.targetAngle = motor3.totalAngle+20;
		motor4.targetAngle = motor4.totalAngle+20;
	}
	if(dir == 2)
	{
		motor1.targetAngle = motor1.totalAngle+20;
		motor2.targetAngle = motor2.totalAngle+20;
		motor3.targetAngle = motor3.totalAngle-20;
		motor4.targetAngle = motor4.totalAngle-20;
	}
	if(dir == 3)
	{
		motor1.targetAngle = motor1.totalAngle+20;
		motor2.targetAngle = motor2.totalAngle+20;
		motor3.targetAngle = motor3.totalAngle+20;
		motor4.targetAngle = motor4.totalAngle+20;
	}
	if(dir == 4 || dir == 5 )
	{
		motor1.targetAngle = motor1.totalAngle-20;
		motor2.targetAngle = motor2.totalAngle-20;
		motor3.targetAngle = motor3.totalAngle-20;
		motor4.targetAngle = motor4.totalAngle-20;
	}
}

void Rotate(int32_t length)
{
	motor1.targetAngle =motor1.totalAngle + length;
	motor2.targetAngle =motor2.totalAngle - length;
	motor3.targetAngle =motor3.totalAngle + length;
	motor4.targetAngle =motor4.totalAngle - length;
}

void put_the_brakes_on(void)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
}

void put_the_brakes_back(void)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
}

void put_the_brakes_off(void)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
}

void Forward_1(int32_t length)
{
	motor1.targetAngle += length;
}

void Forward_2(int32_t length)
{
	motor2.targetAngle += length;
}

void Forward_3(int32_t length)
{
	motor3.targetAngle += length;
}

void Forward_4(int32_t length)
{
	motor4.targetAngle += length;
}

void Change_out_kp_1(int num)
{
	motor1.anglePID.outer.kp = num ;
}

void Change_out_ki_1(int num)
{
	motor1.anglePID.outer.ki = num ;
}

void Change_out_kd_1(int num)
{
	motor1.anglePID.outer.kd = num ;
}

void Change_in_kp_1(int num)
{
	motor1.anglePID.inner.kp = num ;
}

void Change_in_ki_1(int num)
{
	motor1.anglePID.inner.ki = num ;
}

void Change_in_kd_1(int num)
{
	motor1.anglePID.inner.kd = num ;
}

void Change_max_speed_1(int num)
{
	motor1.anglePID.inner.maxOutput = num ;
}

void Change_out_kp_2(int num)
{
	motor2.anglePID.outer.kp = num ;
}

void Change_out_ki_2(int num)
{
	motor2.anglePID.outer.ki = num ;
}

void Change_out_kd_2(int num)
{
	motor2.anglePID.outer.kd = num ;
}

void Change_in_kp_2(int num)
{
	motor2.anglePID.inner.kp = num ;
}

void Change_in_ki_2(int num)
{
	motor2.anglePID.inner.ki = num ;
}

void Change_in_kd_2(int num)
{
	motor2.anglePID.inner.kd = num ;
}

void Change_max_speed_2(int num)
{
	motor2.anglePID.inner.maxOutput = num ;
}

void Change_out_kp_3(int num)
{
	motor3.anglePID.outer.kp = num ;
}

void Change_out_ki_3(int num)
{
	motor3.anglePID.outer.ki = num ;
}

void Change_out_kd_3(int num)
{
	motor3.anglePID.outer.kd = num ;
}

void Change_in_kp_3(int num)
{
	motor3.anglePID.inner.kp = num ;
}

void Change_in_ki_3(int num)
{
	motor3.anglePID.inner.ki = num ;
}

void Change_in_kd_3(int num)
{
	motor3.anglePID.inner.kd = num ;
}

void Change_max_speed_3(int num)
{
	motor3.anglePID.inner.maxOutput = num ;
}

void Change_out_kp_4(int num)
{
	motor4.anglePID.outer.kp = num ;
}

void Change_out_ki_4(int num)
{
	motor4.anglePID.outer.ki = num ;
}

void Change_out_kd_4(int num)
{
	motor4.anglePID.outer.kd = num ;
}

void Change_in_kp_4(int num)
{
	motor4.anglePID.inner.kp = num ;
}

void Change_in_ki_4(int num)
{
	motor4.anglePID.inner.ki = num ;
}

void Change_in_kd_4(int num)
{
	motor4.anglePID.inner.kd = num ;
}

void Change_max_speed_4(int num)
{
	motor4.anglePID.inner.maxOutput = num ;
}


void change_speed(int i1,int i2,int i3,int i4,int i5)
{
	if(motor1.anglePID.outer.maxOutput<100 +i5*10 && motor1.anglePID.outer.maxOutput>100 - i5*10)
	{
		motor1.anglePID.outer.maxOutput += i1;
	}
	if(motor2.anglePID.outer.maxOutput<100 +i5*10 && motor2.anglePID.outer.maxOutput>100 - i5*10)
	{
		motor2.anglePID.outer.maxOutput += i2;
	}
	if(motor3.anglePID.outer.maxOutput<100 +i5*10 && motor3.anglePID.outer.maxOutput>100 - i5*10)
	{
		motor3.anglePID.outer.maxOutput += i3;
	}
	if(motor4.anglePID.outer.maxOutput<100 +i5*10 && motor4.anglePID.outer.maxOutput>100 - i5*10)
	{
		motor4.anglePID.outer.maxOutput += i4;
	}
	if(motor1.anglePID.outer.maxOutput == 100 +i5*10 && i1<0)
	{
		motor1.anglePID.outer.maxOutput +=i1;
	}
	if(motor1.anglePID.outer.maxOutput == 100 - i5*10 && i1>0)
	{
		motor1.anglePID.outer.maxOutput +=i1;
	}
	if(motor2.anglePID.outer.maxOutput == 100 +i5*10 && i2<0)
	{
		motor2.anglePID.outer.maxOutput +=i2;
	}
	if(motor2.anglePID.outer.maxOutput == 100 - i5*10 && i2>0)
	{
		motor2.anglePID.outer.maxOutput +=i2;
	}
	if(motor3.anglePID.outer.maxOutput == 100 +i5*10 && i3<0)
	{
		motor3.anglePID.outer.maxOutput +=i3;
	}
	if(motor3.anglePID.outer.maxOutput == 100 - i5*10 && i3>0)
	{
		motor3.anglePID.outer.maxOutput +=i3;
	}
	if(motor4.anglePID.outer.maxOutput == 100 +i5*10 && i4<0)
	{
		motor4.anglePID.outer.maxOutput +=i4;
	}
	if(motor4.anglePID.outer.maxOutput == 100 - i5*10 && i4>0)
	{
		motor4.anglePID.outer.maxOutput +=i4;
	}
}


