#include "ros/ros.h"
  
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "srs_common/CANCode.h"

#include <sstream>
#include <string>

float angle_target[2]={0};
float angle_last[2]={0}; 
float angle_pos[2]={0};
int   time_target=0;
int   time_pos=0; 

ros::Publisher canlink_pub;
void canlink_callback(const srs_common::CANCode& cancode_msg){
	std::string recv_channel=cancode_msg.channel;
	int recv_id=cancode_msg.id;
	int recv_com=cancode_msg.com;
	bool recv_remote=cancode_msg.remote;
	if(recv_channel=="B" && recv_id==8 && recv_com==1 && recv_remote==false){//gun
		float angle[2]={0};
		angle[0]=-((float)(((cancode_msg.data[0]<<8)+cancode_msg.data[1])-7500)/4000)*3.141592;//need modify
		angle[1]=-((float)(((cancode_msg.data[2]<<8)+cancode_msg.data[3])-7500)/4000)*3.141592;
		for(int i=0;i<2;i++){	
			angle_target[i]=angle[i];
			angle_last[i]=angle_pos[i];
		}
		time_target=cancode_msg.data[7];
		time_pos=0;
	}
	else if(recv_channel=="B" && recv_id==8 && recv_com==1 && recv_remote==true){//gun
		srs_common::CANCode cancode_msg2;
		cancode_msg2.channel=recv_channel;
		cancode_msg2.id=recv_id;
		cancode_msg2.com=1;
		cancode_msg2.remote=false;
		cancode_msg2.length=8;

		int angles[3]={0};
		angles[0]=((-angle_pos[0]/3.141592)*4000)+7500;
		angles[1]=((-angle_pos[1]/3.141592)*4000)+7500;
		cancode_msg2.data[0]=(angles[0]>>8)&0xff;
		cancode_msg2.data[1]=(angles[0]>>0)&0xff;
		cancode_msg2.data[2]=(angles[1]>>8)&0xff;
		cancode_msg2.data[3]=(angles[1]>>0)&0xff;
		cancode_msg2.data[4]=0;
		cancode_msg2.data[5]=0;
		cancode_msg2.data[6]=0;
		cancode_msg2.data[7]=0;
		canlink_pub.publish(cancode_msg2);
	}
} 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "leg_joint_sim");
	ros::NodeHandle n;
	//Publisher
	canlink_pub = n.advertise<srs_common::CANCode>("CANLink_in", 1000);
	//Subscriber
	ros::Subscriber canlink_sub = n.subscribe("CANLink_out", 10, canlink_callback); 

	ros::Rate loop_rate(100); 
	while (ros::ok()){
		//gun
		float rate=0;
		if(time_target!=0)rate=(float)time_pos/time_target;
		for(int j=0;j<2;j++){
			angle_pos[j]=(1-rate)*angle_last[j]+(rate)*angle_target[j];
		}
		if(time_pos<time_target)time_pos++;
		
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

