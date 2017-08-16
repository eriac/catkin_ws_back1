#include "ros/ros.h"

#include "std_msgs/Float64.h"  
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "srs_common/CANCode.h"

#include <sstream>
#include <string>

ros::Publisher  canlink_pub;
void canlink_callback(const srs_common::CANCode& canlink_msg){
	ros::Time subscribe_time=ros::Time::now();
	printf("sub_time c:%d %d,%d\n",canlink_msg.data[0],subscribe_time.sec, subscribe_time.nsec);
} 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "can_test2");
	ros::NodeHandle n;
	//Subscriber
	ros::Subscriber seriallink_sub = n.subscribe("/CANLink_in", 10, canlink_callback); 
	ros::Rate loop_rate(100); 
	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

