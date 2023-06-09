/*----------------------------------------------------------------------------------------------------------------------/
*               本程序只供本校学生学习使用，版权著作权属于武汉科技大学，武汉科技大学将飞控程序源码提供给本校学生，
*               本校学生要为武汉科技大学提供保护，未经学校许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载， 
*               更不能以此销售牟利，如发现上述行为，武汉科技大学将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               无名科创开源飞控QQ群：540707961
*               CSDN博客：http://blog.csdn.net/u011992534
*               优酷ID：NamelessCotrunWUST
*               B站教学视频：https://space.bilibili.com/67803559/#/video
*               客户使用心得、改进意见征集贴：http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               淘宝店铺：https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               百度贴吧:无名科创开源飞控
*               修改日期:2018/8/30
*               版本：慧飞者——V1.0.1
*               版权所有，盗版必究。
*               Copyright(C) 武汉科技大学武汉科技大学 2017-2025
*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
#ifndef _CONTROL_ALTHOLD_H
#define _CONTROL_ALTHOLD_H



//注释以下行开启三环定高模式，即竖直高度位置+竖直速度+竖直加速度
#define YAW_Pos_Control_Accel_Disable  0
//#define ALT_VEL_FEEDFORWARD_ENABLE 		 1 //速度前馈控制使能


typedef enum 
{
  ALTHOLD_MANUAL_CTRL=0,
  ALTHOLD_AUTO_POS_CTRL,
	ALTHOLD_AUTO_VEL_CTRL,
}ALTHOLD_CTRL_MODE;

#define NUL 0


void Flight_Alt_Hold_Control(uint8_t mode,float target_alt,float target_vel);



void Thr_Scale_Set(Vector_RC *rc_date);
void ncq_control_althold(void);
float get_stopping_point_z(Vector3f *stopping_point);
uint8_t Thr_Push_Over_Deadband(void);


extern float ALT_VEL_FEEDFORWARD_ENABLE_Output;//竖直速度前馈控制器输出;
extern float ALT_VEL_FEEDFORWARD_ENABLE_Rate;//竖直速度前馈控制器，APM里面为1、0.45;
extern float ALT_VEL_FEEDFORWARD_ENABLE_Delta;//竖直期望速度变化率;
extern float Last_Yaw_Vel_Target;
extern float Yaw_Vel_Target;
extern Vector3f UAV_Cushion_Stop_Point;

extern uint16_t  Deadband;//油门中位死区
extern uint16_t  Deadzone_Min;
extern uint16_t  Deadzone_Max;
extern uint16_t  Thr_Top;//油门最大上行程
extern uint16_t  Thr_Buttom;//油门最大下行程
extern uint8_t Thr_Push_Over_State;
#endif

