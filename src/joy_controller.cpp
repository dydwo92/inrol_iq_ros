#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/ByteMultiArray.h"

ros::Publisher pub_switch;
ros::Publisher pub_inputs;

std_msgs::ByteMultiArray switches;
std_msgs::Float32MultiArray inputs;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg){

	switches.data[0] = msg->buttons[0];
	switches.data[1] = msg->buttons[1];
	switches.data[2] = msg->buttons[2];

	if(msg->axes[0] > 0){
		inputs.data[0] = msg->axes[0] * 300.0;
		inputs.data[2] = -inputs.data[0];
	}else{
		inputs.data[2] = -msg->axes[0] * 300.0;
		inputs.data[0] = -inputs.data[2];
	}

	if(msg->axes[1] > 0){
		inputs.data[1] = msg->axes[1] * 300.0;
		inputs.data[3] = -inputs.data[1];
	}else{
		inputs.data[3] = -msg->axes[1] * 300.0;
		inputs.data[1] = -inputs.data[3];
	}

	pub_switch.publish(switches);
	pub_inputs.publish(inputs);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "joy_controller");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	std::string joy_topic;
	if(!nh.getParam("joy_topic", joy_topic)){
		joy_topic = "joy";
	}
	ROS_INFO("Subscribe [%s] topic", joy_topic.c_str());

	std::string switch_topic;
	if(!nh.getParam("switch_topic", switch_topic)){
		switch_topic = "switch";
	}
	ROS_INFO("Publish [%s] topic", switch_topic.c_str());

	std::string input_topic;
	if(!nh.getParam("input_topic", input_topic)){
		input_topic = "iq/input";
	}
	ROS_INFO("Publish [%s] topic", input_topic.c_str());

	inputs.layout.dim.push_back(std_msgs::MultiArrayDimension());
	inputs.layout.dim[0].size = 4;
	inputs.layout.dim[0].stride = 4;
	inputs.layout.data_offset = 0;
	inputs.data.resize(4);

	switches.layout.dim.push_back(std_msgs::MultiArrayDimension());
	switches.layout.dim[0].size = 3;
	switches.layout.dim[0].stride = 3;
	switches.layout.data_offset = 0;
	switches.data.resize(3);

	ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>(joy_topic, 1000, joyCallback);
	pub_switch = n.advertise<std_msgs::ByteMultiArray>(switch_topic, 1000);
	pub_inputs = n.advertise<std_msgs::Float32MultiArray>(input_topic, 1000);

	ros::spin();

	return 0;
}