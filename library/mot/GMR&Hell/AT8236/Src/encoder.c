#include "encoder.h"

Motor motor1;

void Motor_Init(void)
{
    HAL_TIM_Encoder_Start(&ENCODER_TIM, TIM_CHANNEL_ALL);      //开启编码器定时器
    __HAL_TIM_ENABLE_IT(&ENCODER_TIM,TIM_IT_UPDATE);           //开启编码器定时器更新中断,防溢出处理
    HAL_TIM_Base_Start_IT(&GAP_TIM);                       //开启100ms定时器中断
    HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_2);            //开启PWM
    HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_1);            //开启PWM
    __HAL_TIM_SET_COUNTER(&ENCODER_TIM, 10000);                //编码器定时器初始值设定为10000
    motor1.lastCount = 0;                                   //结构体内容初始化
    motor1.totalCount = 0;
    motor1.overflowNum = 0;                                  
    motor1.speed = 0;
    motor1.direct = 0;
}



/*如果发现无法进入编码器中断，可以用这个*/
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器回调函数，用于计算速度
{	
    if(htim->Instance==GAP_TIM.Instance)//间隔定时器中断，是时候计算速度了
    {
        motor1.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&ENCODER_TIM);//如果向上计数（正转），返回值为0，否则返回值为1
        motor1.totalCount = COUNTERNUM_1 + motor1.overflowNum * RELOADVALUE_1;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        
        if(motor1.lastCount - motor1.totalCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motor1.overflowNum++;
            motor1.totalCount = COUNTERNUM_1 + motor1.overflowNum * RELOADVALUE_1;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        else if(motor1.totalCount - motor1.lastCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motor1.overflowNum--;
            motor1.totalCount = COUNTERNUM_1 + motor1.overflowNum * RELOADVALUE_1;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        
        motor1.speed = (float)(motor1.totalCount - motor1.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 3000;//算得每秒多少转,除以4是因为4倍频
        motor1.lastCount = motor1.totalCount; //记录这一次的计数值
}
*/