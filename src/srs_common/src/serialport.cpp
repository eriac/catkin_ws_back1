#include "ros/ros.h"
  
#include "std_msgs/String.h"

#include <string>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int open_serial(const char *device_name){
	int fd1=open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
	fcntl(fd1, F_SETFL,0);
	//load configuration
	struct termios conf_tio;
	tcgetattr(fd1,&conf_tio);
	//set baudrate
	speed_t BAUDRATE = B1000000;
	cfsetispeed(&conf_tio, BAUDRATE);
	cfsetospeed(&conf_tio, BAUDRATE);
	//non canonical, non echo back
	conf_tio.c_lflag &= ~(ECHO | ICANON);
	//non blocking
	conf_tio.c_cc[VMIN]=0;
	conf_tio.c_cc[VTIME]=0;
	//store configuration
	tcsetattr(fd1,TCSANOW,&conf_tio);
	return fd1;
}
int fd1;
void serial_callback(const std_msgs::String& serial_msg){
	write(fd1,serial_msg.data.c_str(),serial_msg.data.size());
	printf("send:%s\n",serial_msg.data.c_str());
} 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "serialport");
	ros::NodeHandle n;
	char device_name[]="/dev/ttyUSB0";
	fd1=open_serial(device_name);
	if(fd1<0){
		ROS_ERROR("Serial Fail: cound not open %s", device_name);
		printf("Serial Fail\n");
		return 0;
	}
	//Publisher
	ros::Publisher serial_pub = n.advertise<std_msgs::String>("Serial_in", 1000);
	//Subscriber
	ros::Subscriber serial_sub = n.subscribe("Serial_out", 10, serial_callback); 
	
	ros::Rate loop_rate(200); 
	while (ros::ok()){
		char buf[256]={0};
		int recv_data=read(fd1, buf, sizeof(buf));
		if(recv_data!=0){
			printf("recv:%03d %s\n",recv_data,buf);
			std_msgs::String serial_msg;
			serial_msg.data=buf;
			serial_pub.publish(serial_msg);
		}
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

