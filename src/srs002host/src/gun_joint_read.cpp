#include "ros/ros.h"
  
#include "math.h"  
#include "sensor_msgs/JointState.h"
#include "srs_common/CANCode.h"
#include "visualization_msgs/MarkerArray.h"

#include <string>
#include <iostream>
#include <sstream>

float angles[2]={0};

ros::Publisher  joint_pub;
void joint_publish(){
	sensor_msgs::JointState jointstate;
	jointstate.header.stamp = ros::Time::now();
	jointstate.name.resize(2);
	jointstate.position.resize(2);
	jointstate.header.stamp = ros::Time::now();
	jointstate.name[0]="gun0_joint0";
	jointstate.name[1]="gun0_joint1";
	jointstate.position[0]=angles[0];
	jointstate.position[1]=angles[1];
	
	joint_pub.publish(jointstate);
}
void canlink_callback(const srs_common::CANCode& cancode_msg){
	if(cancode_msg.id==8 && cancode_msg.com==1){
		angles[0]=-((float)(((cancode_msg.data[0]<<8)+cancode_msg.data[1])-7500)/4000)*3.141592;//need modify
		angles[1]=-((float)(((cancode_msg.data[2]<<8)+cancode_msg.data[3])-7500)/4000)*3.141592;
	}
} 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "gun_joint_read");
	ros::NodeHandle n;
	
	//publish command message to joints/servos of arm
	joint_pub  = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
	ros::Publisher  canlink_pub = n.advertise<srs_common::CANCode>("CANLink_out", 1000);
	ros::Subscriber canlink_sub = n.subscribe("CANLink_in", 10, canlink_callback); 
	
	ros::Rate loop_rate(100); 
	while (ros::ok()){
		static int counter=0;
		counter++;
		if(counter%5==0){
			srs_common::CANCode cancode_msg;
			cancode_msg.channel="B";
			cancode_msg.id=8;
			cancode_msg.com=1;
			cancode_msg.remote=true;
			cancode_msg.length=0;
			canlink_pub.publish(cancode_msg);
		}
		else if(counter%5==3){
			joint_publish();
		}
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

