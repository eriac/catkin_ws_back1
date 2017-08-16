#include "ros/ros.h"

#include "std_msgs/Float64.h"  
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "srs_common/CANCode.h"

#include <sstream>
#include <string>

ros::Publisher  canlink_pub;
ros::Time publish_time;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "can_test");
	ros::NodeHandle n;
	//publicher
	canlink_pub = n.advertise<srs_common::CANCode>("/CANLink_out", 1000, true);
	/*
	for(int i=0;i<3;i++){
		srs_common::CANLink can_msg;
		can_msg.channel="A";
		can_msg.id=0;
		can_msg.com=1;
		can_msg.remote=true;
		can_msg.length=0;
		canlink_pub.publish(can_msg);
		publish_time=ros::Time::now();
		printf("pub_time %d,%d\n",publish_time.sec, publish_time.nsec);
	}
	*/
	//ros::Duration(2.0).sleep();	
	ros::Rate loop_rate(1); 
	while (ros::ok()){
		static int counter=0;

		srs_common::CANCode can_msg;
		can_msg.channel="A";
		can_msg.id=0;
		can_msg.com=0;
		can_msg.remote=false;
		can_msg.length=1;
		can_msg.data[0]=counter;		
		canlink_pub.publish(can_msg);
		publish_time=ros::Time::now();
		printf("pub_time c:%d %d,%d\n",counter,publish_time.sec, publish_time.nsec);
		
		counter++;

		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

