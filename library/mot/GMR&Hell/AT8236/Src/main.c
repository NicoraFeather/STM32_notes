void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器回调函数，用于计算速度
{
    if(htim->Instance==ENCODER_TIM.Instance)//编码器输入定时器溢出中断，用于防溢出                   
    {      
        if(COUNTERNUM < 10000) motor1.overflowNum++;       //如果是向上溢出
        else if(COUNTERNUM >= 10000) motor1.overflowNum--; //如果是向下溢出
        __HAL_TIM_SetCounter(&ENCODER_TIM, 10000);             //重新设定初始值
    }
    else if(htim->Instance==GAP_TIM.Instance)//间隔定时器中断，是时候计算速度了
    {
        motor1.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&ENCODER_TIM);//如果向上计数（正转），返回值为0，否则返回值为1
        motor1.totalCount = COUNTERNUM + motor1.overflowNum * RELOADVALUE;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        motor1.speed = (float)(motor1.totalCount - motor1.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PER_ROUND) * 10;//算得每秒多少转
        //motor1.speed = (float)(motor1.totalCount - motor1.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PER_ROUND) * 10 * LINE_SPEED_C//算得车轮线速度每秒多少毫米
        motor1.lastCount = motor1.totalCount; //记录这一次的计数值
    }

     /***************************PID速度环**********************************/
        motor_Out = Speed_PID_Realize(&pid_speed,Target_Speed,motor1.speed);
        //Target_Speed是目标速度，自行定义就好
        if(motor_Out >= 0)
    	{
        __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD, 1000);
        __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD, 1000-motor_Out);
    	}
    	else
    	{
        __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_BACKWARD, 1000);
        __HAL_TIM_SetCompare(&MOTOR1_TIM, MOTOR1_CHANNEL_FORWARD, 1000+motor_Out);
    	}
        /**********************************************************************/
}


int main()
{

    Motor_Init();

    while (0)
    {
        /* code */
    }
}