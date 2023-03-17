/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
/*----------------------------------------------------------------------------------------------------------------------/
                开源并不等于免费，先驱者的历史已经证明，在国内这个环境下，毫无收益的开源，单靠坊间个人爱好者，自发地参与项
								目完善的方式行不通，好的开源项目需要请专职人员做好售后服务、手把手教学、统计用户反馈需求、在实践中完成对产
								品的一次次完善与迭代升级。经过综合考虑，无名飞控开源代码中，程序仅保留公司正版激活功能代码，版本激活无实际
								功能，属于公司产品常规出厂操作，不影响客户学习，其余代码全部开放给客户学习，客户移植和二次开发代码请保留代
								码版权。
-----------------------------------------------------------------------------------------------------------------------/
*               本程序只供购买者学习使用，版权著作权属于无名科创团队，无名科创团队将飞控程序源码提供给购买者，
*               购买者要为无名科创团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载， 
*               更不能以此销售牟利，如发现上述行为，无名科创团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------
*                                                 为什么选择无名创新？
*                                         感动人心价格厚道，最靠谱的开源飞控；
*                                         国内业界良心之作，最精致的售后服务；
*                                         追求极致用户体验，高效进阶学习之路；
*                                         萌新不再孤单求索，合理把握开源尺度；
*                                         响应国家扶贫号召，促进教育体制公平；
*                                         新时代奋斗最出彩，建人类命运共同体。 
-----------------------------------------------------------------------------------------------------------------------
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               无名科创开源飞控QQ群：2号群465082224、1号群540707961（人员已满）
*               CSDN博客：http://blog.csdn.net/u011992534
*               优酷ID：NamelessCotrun无名小哥
*               B站教学视频：https://space.bilibili.com/67803559/#/video
*               客户使用心得、改进意见征集贴：http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               淘宝店铺：https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               百度贴吧:无名科创开源飞控
*               公司官网:www.nameless.tech
*               无名创新国内首款TI开源飞控设计初衷、知乎专栏:https://zhuanlan.zhihu.com/p/54471146
*               修改日期:2021/04/30                    
*               版本：筑梦者PLUS——GankerPilot_V3.0
*               版权所有，盗版必究。
*               Copyright(C) 2019-2025 武汉无名创新科技有限公司 
*               All rights reserved
-----------------------------------------------------------------------------------------------------------------------
*               重要提示：
*               正常淘宝咸鱼转手的飞控、赠送朋友、传给学弟的都可以进售后群学习交流，
*               不得直接在网上销售无名创新资料，无名创新代码有声明版权，他人不得将
*               资料代码传网上供他人下载，不得以谋利为目的销售资料代码，发现有此操
*               作者，公司会提前告知，请1天内及时处理，否则你的学校、单位、姓名、电
*               话、地址信息会被贴出在公司官网、官方微信公众平台、官方技术博客、知乎
*               专栏以及淘宝店铺首页予以公示公告，此种所作所为，会成为个人污点，影响
*               升学、找工作、社会声誉、很快就很在无人机界出名，后果很严重。
*               因此行为给公司造成重大损失者，会以法律途径解决，感谢您的合作，谢谢！！！
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "SYSTEM.h"

/***********************************************************
@函数名：HardWave_Init
@入口参数：无
@出口参数：无
功能描述：对系统芯片资源常规硬件资源进行一系列初始化，具体见
各初始化后的注释
@作者：WUST
@日期：2019年01月27日
*************************************************************/
void HardWave_Init(void)
{
  System_Init();												//系统时钟初始化暂未开放
	USART1_Init();												//前视OPENMV 921600		
	USART2_Init(); 												//主串口初始化(无名创新地面站&山外地面站)波特率115200
	USART0_Init(); 												//光流   19200
	USART3_Init(); 												//俯视OPENMV 921600
	ADC_Init();    												//芯片AD检测初始化
  Butterworth_Parameter_Init();					//滤波器参数初始化
  Bling_Init();													//LED状态指示灯初始化
  Quad_Start_Bling();										//开机LED预显示
	Simulation_PWM_Init();								//模拟PWM初始化
  Key_Init();														//板载按键初始化
  RC_Calibration_Trigger();							//遥控器行程标定检查
  Horizontal_Calibration_Init();				//机架水平标定初始化
  PID_Paramter_Init_With_Flash();				//PID参数初始化
	Other_Parameter_Init();         			//其它参数初始化
	ADRC_Init(&ADRC_Alt_Vel_Controller,2);//自抗扰控制器初始化
	delay_ms(1000);												//初始化延时
  Init_MPU6050();												//MPU6050陀螺仪、加速度计配置初始化
  SPL06_Init();													//SPL06气压计配置初始化
	//US100_Init();													//超声波配置初始化
	GY53L1X_Init();												//激光测距传感器配置初始化
	Compass_Init();                 			//磁力计初始化
  Parameter_Init();											//加速度计校准参数初始化
  WP_Quad_Init();												//根据观测传感器加速度计对姿态四元数初始化
	delay_ms(2500);												//本段延时等待openmv工作就位，用于初始配置openmv工作模式
	SDK_Init();														//SDK模式初始化
	Temperature_Ctrl_Init();							//温度控制器初始化
  Time_init();													//飞控调度定时器初始化
	LCD_CLS();
	
	//*************PID参数初始化**************//
	
	//俯仰角速度
	Total_Controller.Pitch_Gyro_Control.Kp=2.50;//2.50;//俯仰角速度
	Total_Controller.Pitch_Gyro_Control.Ki=0.80;//0.80;
	Total_Controller.Pitch_Gyro_Control.Kd=0.48;//0.50;
	
	//翻滚角速度环
	Total_Controller.Roll_Gyro_Control.Kp=2.60;//2.60;//横滚角速度
	Total_Controller.Roll_Gyro_Control.Ki=1.00;//1.00;
	Total_Controller.Roll_Gyro_Control.Kd=0.43;//0.45
	
	//偏航角速度环 
	Total_Controller.Yaw_Gyro_Control.Kp = 1.50;//1.2
	Total_Controller.Yaw_Gyro_Control.Ki = 1.00;//1.1
	Total_Controller.Yaw_Gyro_Control.Kd = 0.00;	
	
	//角度外环,默认的已经差不多了，就不用微调
	
	Total_Controller.Pitch_Angle_Control.Kp=4.50;//5.00;//俯仰角度
	Total_Controller.Pitch_Angle_Control.Ki=0.00;//0.00;
	Total_Controller.Pitch_Angle_Control.Kd=0.00;//0.00;
	
	Total_Controller.Roll_Angle_Control.Kp=4.50;//5.00;//横滚角度
	Total_Controller.Roll_Angle_Control.Ki=0.00;//0.00
	Total_Controller.Roll_Angle_Control.Kd=0.00;//0.00
	/*
	Total_Controller.Yaw_Angle_Control.Kp=5.00;//5.00;//5.00;//偏航角度
	Total_Controller.Yaw_Angle_Control.Ki=0.00;//0.00;
	Total_Controller.Yaw_Angle_Control.Kd=0.00;//0.00;//0.03;
	*/
	
	//高度速度
	/*
	Total_Controller.High_Speed_Control.Kp=4.745;//3.00;//海拔攀升速度
	Total_Controller.High_Speed_Control.Ki=0.05;//0.05;
	Total_Controller.High_Speed_Control.Kd=0.0551;//0.15;
	
	//高度位置
	Total_Controller.High_Position_Control.Kp=0.50;//2.50;//海拔高度位置
	Total_Controller.High_Position_Control.Ki=0.00;//0.35;
	Total_Controller.High_Position_Control.Kd=0.025;//0.30;
	*/

	//高度加速度
	Total_Controller.High_Acce_Control.Kp = 0.29;//0.17;//高度加速度
	Total_Controller.High_Acce_Control.Ki = 2.20;//2.20;
	Total_Controller.High_Acce_Control.Kd = 0.00;//0.00;	
	
	
	//光流位置环
	/*
	Total_Controller.Optical_Position_Control.Kp=0.35;//0.18//光流位置
	Total_Controller.Optical_Position_Control.Ki=0.01;//0.00
	Total_Controller.Optical_Position_Control.Kd=0.05;//0.00
	*/
	
	//光流速度环
	Total_Controller.Optical_Speed_Control.Kp =5.80;//5.80
	Total_Controller.Optical_Speed_Control.Ki = 0.01;//0.01
	Total_Controller.Optical_Speed_Control.Kd = 1.00;//1.50
	//光流互补权重也会影响前后荡
	
	//SDK
	/*
	Total_Controller.SDK_Roll_Position_Control.Kp=0.35;
	Total_Controller.SDK_Roll_Position_Control.Ki=0.05;//0.05;
	Total_Controller.SDK_Roll_Position_Control.Kd=0.05;//1.00;

	Total_Controller.SDK_Pitch_Position_Control.Kp=0.35;
	Total_Controller.SDK_Pitch_Position_Control.Ki=0.05;//0.05;
	Total_Controller.SDK_Pitch_Position_Control.Kd=0.05;//1.00;
	*/
	
	
}

/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
