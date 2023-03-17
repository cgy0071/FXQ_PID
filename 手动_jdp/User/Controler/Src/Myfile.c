#include "Headfile.h"
#include "Myfile.h"

/*********myflag*********/
uint8_t Flag1 = 0;
uint8_t Flag2 = 0;
uint8_t Flag3 = 0;

float Value1 = 0;
float Value2 = 0;
float Value3 = 0;

uint16_t PID_count=0;


/*****************
     控制函数
********************/


//以45度为最大的偏航单位
float control_yaw_delta_max(float delta_yaw_target,float delta_max)
{
	 int_least16_t Polarity;
	 uint8_t overflow_flag=0;
	
		//delta_yaw_target为正，则逆时针转动
		//delta_yaw_target为负，则顺时针转动
	if(delta_yaw_target>0)
		Polarity=1;//逆
	else
		Polarity=-1;//顺

		if(ABS(delta_yaw_target)>delta_max)//溢出,控制量就为delta_max
		{
			Total_Controller.Yaw_Angle_Control.Expect+=(delta_max*Polarity);
			overflow_flag=1;
		}
		else
			Total_Controller.Yaw_Angle_Control.Expect+=delta_yaw_target;
			
		//周期化偏航角度
		if(Total_Controller.Yaw_Angle_Control.Expect>=360)
			Total_Controller.Yaw_Angle_Control.Expect-=360;
		if(Total_Controller.Yaw_Angle_Control.Expect<=0)
			Total_Controller.Yaw_Angle_Control.Expect+=360;
		
		if(Total_Controller.Yaw_Angle_Control.Expect==0&&Total_Controller.Yaw_Angle_Control.Expect==360)
			Total_Controller.Yaw_Angle_Control.Expect+=(float)0.05;
	
		if(overflow_flag==1)  //如果有溢出，就返回溢出值
		{
				if(Polarity==1)   //正的就减
				{
					delta_yaw_target-=delta_max;
					return delta_yaw_target;
				}
				if(Polarity==-1)   //负的就加
				{
					delta_yaw_target+=delta_max;
					return delta_yaw_target;
				}	
		}
		else
				 return 0; //没有溢出，就返回0
		return 0;
}

uint8_t control_yaw_delta(float delta_yaw_target)
{
		float overflow_yaw=0;
	  static float set_yaw_flag=1;
	 
	  if(set_yaw_flag)//设置控制偏航角
		{
		overflow_yaw=control_yaw_delta_max(delta_yaw_target,45);
		set_yaw_flag=0;
		}
		//完成设置的偏航角
		if(ABS(Total_Controller.Yaw_Angle_Control.Expect-Yaw)<=5)
		{
			if(overflow_yaw!=0)//若有溢出，则还需要继续设置
				{
					set_yaw_flag=1;
					return 0;
				}
				else           //若有溢出，则还需要继续设置
					return 1;
		}
		return 0;
}

uint8_t control_yaw(float delta_yaw_target,float yaw_vel,float delta,SDK_Status *Status,uint16_t number)
{

		static uint8_t end_flag=0;	
	
		static float end_time=0;
	  Testime dt;
		Test_Period(&dt);
	
		ncq_control_althold();//高度控制依然进行
		OpticalFlow_Control_Pure(0);//完成之后，进行光流悬停
	  if(Status->Status[number].Start_Flag==1
     &&Status->Status[number].Execute_Flag==1
					 &&Status->Status[number].End_flag==1)
		{									
			return 1;
		}
		else
		{		//开始后只配置一次
				if(Status->Status[number].Start_Flag==0) 
				{					
					if(delta_yaw_target==0)//速度控制，一次任务中只设置一次
					{
						end_flag=1;
					}
					else//角度控制
					{
	
					
						end_flag=2;
					}				
						end_time=dt.Now_Time+delta;//单位ms 
						Status->Status[number].Start_Flag=1;				
					
				}

				if(end_flag==1)//角速度控制
				{  
					if(dt.Now_Time>end_time)//结束条件
					{ 
						end_flag=0;
						end_time=0;
						Status->Status[number].Execute_Flag=1;
						Status->Status[number].End_flag=1;
						Status->Transition_Time[number]=2000;//200*5=1秒
						return 1;//返回完成
					}
					Status->Status[number].Execute_Flag=1;
				}
				else if(end_flag==2) //角度控制
				{
						if(ABS(Total_Controller.Yaw_Angle_Control.Expect-Yaw)<=5)
						{
							Status->Status[number].Execute_Flag=1;
							Status->Status[number].End_flag=1;
							Status->Transition_Time[number]=2000;//1000*5ms			
							
							return 1;//返回完成					
						}
						Status->Status[number].Execute_Flag=1;									
				}						
		}
		return 0;		
}

uint8_t control_yaw_rotate(float delta_yaw_target,SDK_Status *Status,uint16_t number)
{

		ncq_control_althold();//高度控制依然进行
		OpticalFlow_Control_Pure(0);//完成之后，进行光流悬停
	  if(Status->Status[number].Start_Flag==1
     &&Status->Status[number].Execute_Flag==1
					 &&Status->Status[number].End_flag==1)
		{									
			return 1;
		}
		else
		{		
				if(Status->Status[number].Start_Flag==0) 
				{
					//delta_yaw_target为正，则逆时针转动
					//delta_yaw_target为负，则顺时针转动
					Total_Controller.Yaw_Angle_Control.Expect+=delta_yaw_target;
					//周期化偏航角度
					if(Total_Controller.Yaw_Angle_Control.Expect>=360)
						Total_Controller.Yaw_Angle_Control.Expect-=360;
					if(Total_Controller.Yaw_Angle_Control.Expect<=0)
						Total_Controller.Yaw_Angle_Control.Expect+=360;
					if(Total_Controller.Yaw_Angle_Control.Expect==0&&Total_Controller.Yaw_Angle_Control.Expect==360)
						Total_Controller.Yaw_Angle_Control.Expect+=(float)0.05;		
					
					Status->Status[number].Start_Flag=1;
				}
				
				if(ABS(Total_Controller.Yaw_Angle_Control.Expect-Yaw)<=5)
				{
					Status->Status[number].Execute_Flag=1;
					Status->Status[number].End_flag=1;
					Status->Transition_Time[number]=2000;//1000*5ms			
					
					return 1;//返回完成					
				}
				else
				{
				Status->Status[number].Execute_Flag=1;				
					return 0;
				}							
		}
				
}

