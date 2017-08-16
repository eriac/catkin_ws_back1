#include "ros/ros.h"
  
#include "math.h"  
#include "sensor_msgs/JointState.h"
#include "srs_common/CANCode.h"
#include "visualization_msgs/MarkerArray.h"

#include <string>
#include <iostream>
#include <sstream>

float angles[6][3]={{0}};
float loads[6]={0};

ros::Publisher  joint_pub;
ros::Publisher marker_pub;
void joint_publish(){
	sensor_msgs::JointState jointstate;
	jointstate.header.stamp = ros::Time::now();
	jointstate.name.resize(18);
	jointstate.position.resize(18);
	jointstate.header.stamp = ros::Time::now();
	for(int i=0;i<6;i++){
		jointstate.name[i*3+0]="leg"+std::to_string(i)+"_joint0";
		jointstate.name[i*3+1]="leg"+std::to_string(i)+"_joint1";
		jointstate.name[i*3+2]="leg"+std::to_string(i)+"_joint2";
		jointstate.position[i*3+0]=angles[i][0];
		jointstate.position[i*3+1]=angles[i][1];
		jointstate.position[i*3+2]=angles[i][2];
	}
	joint_pub.publish(jointstate);
}
void load_publish(){
	//for marker
	visualization_msgs::MarkerArray markers;
	markers.markers.resize(6);
    	for(int i=0;i<6;i++){
		markers.markers[i].header.frame_id = "/base_link";
		markers.markers[i].header.stamp = ros::Time::now();
		markers.markers[i].ns = "basic_shapes";
		markers.markers[i].id = i;
		markers.markers[i].type = visualization_msgs::Marker::ARROW;
		markers.markers[i].action = visualization_msgs::Marker::ADD;
	
		markers.markers[i].scale.x = 0.02;
		markers.markers[i].scale.y = 0.04;
		markers.markers[i].scale.z = 0.04;
		float px,py=0;
		if(i/3==0)py=0.05;
		else py=-0.05;
		if(i%3==0)px=0.1;
		else if(i%3==1)px=0.0;
		else px=-0.1;
		markers.markers[i].points.resize(2);
		markers.markers[i].points[0].x=px;
		markers.markers[i].points[0].y=py;
		markers.markers[i].points[0].z=0.1;
		markers.markers[i].points[1].x=px;
		markers.markers[i].points[1].y=py;
		markers.markers[i].points[1].z=0.12+loads[i]*0.1;

		markers.markers[i].color.r = 1.0f;
		markers.markers[i].color.g = 0.5f;
		markers.markers[i].color.b = 0.5f;
		markers.markers[i].color.a = 0.5;
		markers.markers[i].lifetime = ros::Duration();
	}
	marker_pub.publish(markers);
}

void canlink_callback(const srs_common::CANCode& cancode_msg){
	if(0<=cancode_msg.id && cancode_msg.id <=5 && cancode_msg.com==1){
		angles[cancode_msg.id][0]=-((float)(((cancode_msg.data[0]<<8)+cancode_msg.data[1])-7500)/4000)*3.141592;//need modify
		angles[cancode_msg.id][1]= ((float)(((cancode_msg.data[2]<<8)+cancode_msg.data[3])-7500)/4000)*3.141592;
		angles[cancode_msg.id][2]= ((float)(((cancode_msg.data[4]<<8)+cancode_msg.data[5])-7500)/4000)*3.141592;
		
		int load=(cancode_msg.data[6]<<8)+cancode_msg.data[7];
		int load2=0;
		if(load<200)load2=200;
		else if(load>600)load2=600;
		else load2=load;
		loads[cancode_msg.id]=2-sqrt((600.0-load2)/100);
		printf("load%d:%04d %f\n", cancode_msg.id, load, loads[cancode_msg.id]);
	}
} 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "leg_joint_read");
	ros::NodeHandle n;
	
	joint_pub  = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
	marker_pub = n.advertise<visualization_msgs::MarkerArray>("LegLoad_marker", 100);
	ros::Publisher  canlink_pub = n.advertise<srs_common::CANCode>("CANLink_out", 1000);
	
	ros::Subscriber canlink_sub = n.subscribe("CANLink_in", 10, canlink_callback); 
	
	
	ros::Rate loop_rate(100); 
	while (ros::ok()){
		static int counter=0;
		counter++;
		if(counter%5==0){
			srs_common::CANCode cancode_msg;
			cancode_msg.channel="A";
			cancode_msg.id=0;
			cancode_msg.com=1;
			cancode_msg.remote=true;
			cancode_msg.length=0;
			for(int i=0;i<6;i++){
				cancode_msg.id=i;
				canlink_pub.publish(cancode_msg);
			}
		}
		else if(counter%5==3){
			joint_publish();
			load_publish();
		}
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

