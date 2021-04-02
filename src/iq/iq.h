#ifndef IQ_H_
#define IQ_H_

#include <stdint.h>
#include <stdbool.h>

#include "ros/ros.h"

typedef struct{
	float velocity_kp;
	float velocity_ki;
	float velocity_kd;
	float velocity_ff0;
	float velocity_ff1;
	float velocity_ff2;
	float timeout;
	uint16_t motor_pole_pairs;
	float motor_Kv;
	float motor_R_ohm;
	float motor_l;
}IQ_ENV;

typedef struct{
	uint8_t id;
	uint8_t channel;
	IQ_ENV env;
}IQ_INSTANCE;

extern void IQ_Init(ros::NodeHandle *nh, uint8_t device_id);

#endif /* IQ_H_ */
