#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include "CANOpen.h"
#include "iq.h"

#include <string.h>
#include <stdint.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>

#include <net/if.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

int s_can;

static int can_finish;
static pthread_t p_thread[2];

CO_PDOStruct TxPDO1, TxPDO2;
CO_PDOStruct RxPDO1, RxPDO2;
float iq_targetSpeed[4];
float iq_actualSpeed[4];

/*******************************************************************/
// CAN Thread functions
/*******************************************************************/
void* canopen_checkloop(void *d){
	while(!can_finish){
		CANOpen_timerLoop();
		usleep(1000);
	}

	pthread_exit(NULL);
}

void* canopen_rxloop(void *d){

	struct can_frame frame;

	struct timeval timeout;
	fd_set set;
	int rv;

	while(!can_finish){
		FD_ZERO(&set);
		FD_SET(s_can, &set);

		timeout.tv_sec = 0;
		timeout.tv_usec = 100000;

		rv = select(s_can + 1, &set, NULL, NULL, &timeout);
		if(rv > 0){
			read(s_can, &frame, sizeof(frame));
			CANOpen_addRxBuffer(frame.can_id, frame.data);
		}
	}

	pthread_exit(NULL);
}

/*******************************************************************/
// ROS Callback function
/*******************************************************************/
void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	
	if(msg->data.size() >= 4){
		iq_targetSpeed[0] = msg->data[0];
		iq_targetSpeed[1] = msg->data[1];
		iq_targetSpeed[2] = msg->data[2];
		iq_targetSpeed[3] = msg->data[3];
	}
}

/*******************************************************************/
// main funcion
/*******************************************************************/
int main(int argc, char **argv){

	ros::init(argc, argv, "iq_controller");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	ros::Subscriber sub = n.subscribe<std_msgs::Float32MultiArray>("iq/input", 1000, chatterCallback);
	ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("iq/output", 1000);

	std_msgs::Float32MultiArray iq_msg;
	iq_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	iq_msg.layout.dim[0].label = "Output values from IQ Motors";
	iq_msg.layout.dim[0].size = 4;
	iq_msg.layout.dim[0].stride = 4;
	iq_msg.layout.data_offset = 0;

	iq_msg.data.resize(4);

	// Get ROS Parameters
	int rate;
	if(!nh.getParam("rate", rate)){
		rate = 50;
	}
	if(rate > 50){
		ROS_WARN("Maximum communiation rate is 50Hz.");
		rate = 50;
	}
	ROS_INFO("Set IQ communication rate to %d Hz.", rate);
	ros::Rate loop_rate(rate);

	int device_id;
	if(!nh.getParam("device_id", device_id)){
		device_id = 0x10;
	} 

	// Setting Linux socketCAN ----------------------------------------------->
	int ret;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_filter rfilter[1];

	can_finish = 0;

	// [[ 1.Create socket ]]
	s_can = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if(s_can < 0){
		ROS_ERROR("socketCAN PF_CAN failed.");
		return 1;
	}

	// [[ 2.Specify can0 device ]]
	strcpy(ifr.ifr_name, "can0");
	ret = ioctl(s_can, SIOCGIFINDEX, &ifr);
	if(ret < 0){
		ROS_ERROR("socketCAN ioctl failed.");
		return 1;
	}

	// [[ 3.Bind the socket to can0 ]]
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	ret = bind(s_can, (struct sockaddr *)&addr, sizeof(addr));
	if(ret < 0){
		ROS_ERROR("socketCAN bind failed.");
		return 1;
	}

	// [[ 4. Receive all frame ]]
	rfilter[0].can_id = 0x000;
	rfilter[0].can_mask = 0x000;
	setsockopt(s_can, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

	ROS_INFO("can0 socketCAN Enabled.");

	// Setting Linux socketCAN <-----------------------------------------------

	// Create CAN related threads -------------------------------------------->
	int thr_id;

	thr_id = pthread_create(&p_thread[0], NULL, canopen_checkloop, NULL);
	if(thr_id < 0){
		ROS_ERROR("CAN Check loop thread create error.");
		return 1;
	}

	thr_id = pthread_create(&p_thread[1], NULL, canopen_rxloop, NULL);
	if(thr_id < 0){
		ROS_ERROR("CAN RX loop thread create error.");
		return 1;
	}

	ROS_INFO("CANOpen Threads established.");
	// Create CAN related threads <--------------------------------------------

	// main application ------------------------------------------------------>

	iq_targetSpeed[0] = 0.0f;
	iq_targetSpeed[1] = 0.0f;
	iq_targetSpeed[2] = 0.0f;
	iq_targetSpeed[3] = 0.0f;

	CANOpen_NMT(CO_RESET, 0x00); ros::Duration(0.01).sleep();

	CANOpen_mappingPDO_init(&TxPDO1);
	CANOpen_mappingPDO_float(&TxPDO1, &iq_actualSpeed[0]);
	CANOpen_mappingPDO_float(&TxPDO1, &iq_actualSpeed[1]);

	CANOpen_mappingPDO_init(&TxPDO2);
	CANOpen_mappingPDO_float(&TxPDO2, &iq_actualSpeed[2]);
	CANOpen_mappingPDO_float(&TxPDO2, &iq_actualSpeed[3]);

	CANOpen_mappingPDO_init(&RxPDO1);
	CANOpen_mappingPDO_float(&RxPDO1, &iq_targetSpeed[0]);
	CANOpen_mappingPDO_float(&RxPDO1, &iq_targetSpeed[1]);

	CANOpen_mappingPDO_init(&RxPDO2);
	CANOpen_mappingPDO_float(&RxPDO2, &iq_targetSpeed[2]);
	CANOpen_mappingPDO_float(&RxPDO2, &iq_targetSpeed[3]);

	// Set IQ Parameters
	IQ_Init(&nh, device_id);

	CANOpen_NMT(CO_OP, 0x00);

	while(ros::ok()){

		CANOpen_sendPDO(device_id, 1, &RxPDO1);
		CANOpen_sendPDO(device_id, 2, &RxPDO2);
		CANOpen_sendSync();
		CANOpen_readPDO(device_id, 1, &TxPDO1, 10);
		CANOpen_readPDO(device_id, 2, &TxPDO2, 10);

		iq_msg.data[0] = iq_actualSpeed[0];
		iq_msg.data[1] = iq_actualSpeed[1];
		iq_msg.data[2] = iq_actualSpeed[2];
		iq_msg.data[3] = iq_actualSpeed[3];

		pub.publish(iq_msg);

		ros::spinOnce();

		loop_rate.sleep();
	}

	// main application <------------------------------------------------------

	CANOpen_NMT(CO_RESET, 0x00);
	ROS_INFO("Disable all motors");

	can_finish = 1;

	int status;
	pthread_join(p_thread[0], (void **)&status);
	pthread_join(p_thread[1], (void **)&status);
	close(s_can);

	return 0;
}