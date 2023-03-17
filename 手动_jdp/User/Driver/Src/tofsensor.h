#ifndef _TOFSENSOR_H_
#define _TOFSENSOR_H_


typedef struct
{
	uint32_t Tofsensor_start_time;
	uint32_t Tofsensor_end_time;
	uint32_t Tofsensor_delta;
	uint8_t Tofsensor_update_flag;
	float distance;
	float last_distance;
	float pre_last_distance;
	float vel;
	float acc;
	float last_vel;
		uint32_t _cnt;
}Tofsensor_Init_data;

extern Tofsensor_Init_data Tofsensor_Init_in1,Tofsensor_Init_in2;
extern void Tofsensor_Data_Update(void);
extern void Tofsensor_Init(void);

#endif


