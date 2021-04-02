#include "ros/ros.h"
#include "std_msgs/ByteMultiArray.h"

#include<iostream>
#include <vector>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/stat.h>

int fd_gpio[3];

void chatterCallback(const std_msgs::ByteMultiArray::ConstPtr& msg){

	if(fd_gpio[0] > 0 && msg->data.size() > 0){
		lseek(fd_gpio[0], 0, SEEK_SET);
		if(msg->data[0]) write(fd_gpio[0], "1", 1);
		else write(fd_gpio[0], "0", 1);
	}

	if(fd_gpio[1] > 0 && msg->data.size() > 1){
		lseek(fd_gpio[1], 0, SEEK_SET);
		if(msg->data[1]) write(fd_gpio[1], "1", 1);
		else write(fd_gpio[1], "0", 1);
	}

	if(fd_gpio[2] > 0 && msg->data.size() > 2){
		lseek(fd_gpio[2], 0, SEEK_SET);
		if(msg->data[2]) write(fd_gpio[2], "1", 1);
		else write(fd_gpio[2], "0", 1);
	}
}

int main(int argc, char **argv){

	ros::init(argc, argv, "switch_listener");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
	ros::Subscriber sub = n.subscribe<std_msgs::ByteMultiArray>("switch", 1000, chatterCallback);

	std::vector<int> gpios;
	if(nh.getParam("gpios", gpios)){
		if(gpios.size() < 3){
			ROS_ERROR("gpios parameter error");
			return 1;
		}
	}else{
		gpios.resize(3);
		gpios[0] = 26;
		gpios[1] = 13;
		gpios[2] = 6;
	}

	int* gpio_arr = gpios.data();

	ROS_INFO("Setting gpio%d, gpio%d, gpio%d ...", gpio_arr[0], gpio_arr[1], gpio_arr[2]);

	char buf[256];
	int fd;

	int i;
	for(i = 0; i < 3; i++){
		fd = open("/sys/class/gpio/export", O_WRONLY);
		if(fd < 0) ROS_ERROR("Fail to export gpio%d", gpio_arr[i]);
		sprintf(buf, "%d", gpio_arr[i]);
		write(fd, buf, strlen(buf));
		close(fd);

		usleep(100000);

		sprintf(buf, "/sys/class/gpio/gpio%d/direction", gpio_arr[i]);
		fd = open(buf, O_WRONLY);
		if(fd < 0) ROS_ERROR("Fail to set direction gpio%d" ,gpio_arr[i]);
		write(fd, "out", 3);
		close(fd);


		usleep(100000);

		sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio_arr[i]);
		fd_gpio[i] = open(buf, O_WRONLY);
		if(fd_gpio[i] < 0) ROS_ERROR("Fail to open gpio%d value file", gpio_arr[i]);
	}


	ros::spin();

	for(i = 0; i < 3; i++){

		close(gpio_arr[i]);

		fd = open("/sys/class/gpio/unexport", O_WRONLY);
		sprintf(buf, "%d", gpio_arr[i]);
		write(fd, buf, strlen(buf));
		close(fd);
	}

	return 0;
}