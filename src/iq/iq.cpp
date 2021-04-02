#include "iq.h"
#include "CANOpen.h"

#include <string.h>

#define MAX_RETRY	5


bool IQ_read(IQ_INSTANCE *iq, uint8_t subIndex,
		void *data, uint8_t d_len, int timeOut) {

	int retry = 0;
	uint8_t data_tmp[8];
	uint8_t len;

	while(retry < MAX_RETRY){
		if(CANOpen_readOD(iq->id, 0x2000 + iq->channel - 1, subIndex, data_tmp, &len, timeOut) == CO_OK) break;
		retry ++;
	}

	if(retry < MAX_RETRY){
		memcpy(data, data_tmp, d_len);
		return true;
	}

	return false;
}

bool IQ_write(IQ_INSTANCE *iq, uint8_t subIndex,
		void *data, uint8_t d_len, int timeOut) {

	int retry = 0;
	int i;
	uint8_t data_tmp[8];

	while(retry < MAX_RETRY){
		if(CANOpen_writeOD(iq->id, 0x2000 + iq->channel - 1, subIndex, (uint8_t*)data, d_len, timeOut) == CO_OK)
			return true;
		retry ++;
	}
	return false;
}

bool IQ_setEnv(IQ_INSTANCE *iq, IQ_ENV *env, int timeOut) {

	if(!IQ_write(iq, 0x08, &env->velocity_kp, 4, timeOut)) return false;
	if(!IQ_write(iq, 0x09, &env->velocity_ki, 4, timeOut)) return false;
	if(!IQ_write(iq, 0x0A, &env->velocity_kd, 4, timeOut)) return false;
	if(!IQ_write(iq, 0x0B, &env->velocity_ff0, 4, timeOut)) return false;
	if(!IQ_write(iq, 0x0C, &env->velocity_ff1, 4, timeOut)) return false;
	if(!IQ_write(iq, 0x0D, &env->velocity_ff2, 4, timeOut)) return false;
	if(!IQ_write(iq, 0x10, &env->timeout, 4, timeOut)) return false;
	memcpy(&iq->env, env, sizeof(IQ_ENV));

	return true;
}

bool IQ_getEnv(IQ_INSTANCE *iq, int timeOut) {

	if(!IQ_read(iq, 0x08, &iq->env.velocity_kp, 4, timeOut)) return false;
	if(!IQ_read(iq, 0x09, &iq->env.velocity_ki, 4, timeOut)) return false;
	if(!IQ_read(iq, 0x0A, &iq->env.velocity_kd, 4, timeOut)) return false;
	if(!IQ_read(iq, 0x0B, &iq->env.velocity_ff0, 4, timeOut)) return false;
	if(!IQ_read(iq, 0x0C, &iq->env.velocity_ff1, 4, timeOut)) return false;
	if(!IQ_read(iq, 0x0D, &iq->env.velocity_ff2, 4, timeOut)) return false;
	if(!IQ_read(iq, 0x10, &iq->env.timeout, 4, timeOut)) return false;
	if(!IQ_read(iq, 0x17, &iq->env.motor_pole_pairs, 2, timeOut)) return false;
	if(!IQ_read(iq, 0x18, &iq->env.motor_Kv, 4, timeOut)) return false;
	if(!IQ_read(iq, 0x19, &iq->env.motor_R_ohm, 4, timeOut)) return false;
	if(!IQ_read(iq, 0x1A, &iq->env.motor_l, 4, timeOut)) return false;

	return true;
}

void IQ_printEnv(IQ_ENV *env, char* tag) {

	ROS_INFO("%s- velocity_kp = %f [V/(rad/s)]\n", tag, env->velocity_kp);
	ROS_INFO("%s- velocity_ki = %f [V/(rad)]\n", tag, env->velocity_ki);
	ROS_INFO("%s- velocity_kd = %f [V/(rad/s^2)]\n", tag, env->velocity_kd);
	ROS_INFO("%s- velocity_ff0 = %f\n", tag, env->velocity_ff0);
	ROS_INFO("%s- velocity_ff1 = %f [V/(rad/s)]\n", tag, env->velocity_ff1);
	ROS_INFO("%s- velocity_ff2 = %f [V/(rad/s)^2]\n", tag, env->velocity_ff2);
	ROS_INFO("%s- timeout = %f [s]\n", tag, env->timeout);
	ROS_INFO("%s- motor_pole_pairs = %d\n", tag, env->motor_pole_pairs);
	ROS_INFO("%s- motor_Kv = %f [RPM/V]\n", tag, env->motor_Kv);
	ROS_INFO("%s- motor_R_ohm = %f [Ohm]\n", tag, env->motor_R_ohm);
	ROS_INFO("%s- motor_l = %f [H]\n", tag, env->motor_l);

}

void IQ_Init(ros::NodeHandle *nh, uint8_t device_id){
	
	IQ_INSTANCE iq;
	IQ_ENV iq_env;
	iq.id = device_id;

	std::vector<float> env;
	if(nh->getParam("iq_controller/m1", env)){
		iq.channel = 1;
		iq_env.velocity_kp = env[0];
		iq_env.velocity_ki = env[1];
		iq_env.velocity_kd = env[2];
		iq_env.velocity_ff0 = env[3];
		iq_env.velocity_ff1 = env[4];
		iq_env.velocity_ff2 = env[5];
		iq_env.timeout = env[6];
		IQ_setEnv(&iq, &iq_env, 1000);
	}

	if(nh->getParam("iq_controller/m2", env)){
		iq.channel = 2;
		iq_env.velocity_kp = env[0];
		iq_env.velocity_ki = env[1];
		iq_env.velocity_kd = env[2];
		iq_env.velocity_ff0 = env[3];
		iq_env.velocity_ff1 = env[4];
		iq_env.velocity_ff2 = env[5];
		iq_env.timeout = env[6];
		IQ_setEnv(&iq, &iq_env, 1000);
	}

	if(nh->getParam("iq_controller/m3", env)){
		iq.channel = 3;
		iq_env.velocity_kp = env[0];
		iq_env.velocity_ki = env[1];
		iq_env.velocity_kd = env[2];
		iq_env.velocity_ff0 = env[3];
		iq_env.velocity_ff1 = env[4];
		iq_env.velocity_ff2 = env[5];
		iq_env.timeout = env[6];
		IQ_setEnv(&iq, &iq_env, 1000);
	}


	if(nh->getParam("iq_controller/m4", env)){
		iq.channel = 4;
		iq_env.velocity_kp = env[0];
		iq_env.velocity_ki = env[1];
		iq_env.velocity_kd = env[2];
		iq_env.velocity_ff0 = env[3];
		iq_env.velocity_ff1 = env[4];
		iq_env.velocity_ff2 = env[5];
		iq_env.timeout = env[6];
		IQ_setEnv(&iq, &iq_env, 1000);
	}
}
