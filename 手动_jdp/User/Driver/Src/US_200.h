#ifndef _US_200_H_
#define _US_200_H_

typedef struct
{
	uint32_t us200_start_time;
	uint32_t us200_end_time;
	uint32_t us200_delta;
	uint8_t us200_update_flag;
	float distance;
	float last_distance;
	float pre_last_distance;
	float vel;
	float acc;
	float last_vel;
	uint32_t us200_rssi_cnt;
}us200_data;


//US200
#define User_Front_Dis_Max  500//用户使用的最大高度值，单位为cm，需要预留一定裕度给气压计/超声波切换过渡



#define US_200_Distance_CMD    0x55
#define US_200_Temperature_CMD 0x50

void US_200_Statemachine(void);
void US200_Init(void);
void US200_IRQHandler(void);

extern float Front_Distance,Last_Front_Distance,Front_Distance_Div,Last_Front_Distance_Div,Front_Distance_Acc;


extern us200_data us200;

extern uint8_t US200_INF_flag;
#endif


