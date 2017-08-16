#include "ros/ros.h"

#include "math.h"  
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "srs_common/CANCode.h"
#include "srs002host/LegPoint.h"
#include "tf/transform_listener.h"

#include <string>
#include <sstream>

geometry_msgs::TwistStamped gdir_last;
std_msgs::String gmode_last;
std_msgs::String glight_last;
void gundir_callback(const geometry_msgs::TwistStamped gdir){
	gdir_last=gdir;
}
void gunmode_callback(const std_msgs::String gmode){
	gmode_last=gmode;
}
void gunlight_callback(const std_msgs::String glight){
	glight_last=glight;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "gun_candriver");
	ros::NodeHandle n;
	
	//publish
	ros::Publisher  canlink_pub = n.advertise<srs_common::CANCode>("CANLink_out", 1000);
	//subscriibe
	ros::Subscriber gundir_sub  = n.subscribe("gundir", 10, gundir_callback);
	ros::Subscriber gunmode_sub = n.subscribe("gunmode", 10, gunmode_callback);
	ros::Subscriber gunlight_sub= n.subscribe("gunlight", 10, gunlight_callback);

	ros::Rate loop_rate(10); 
	while (ros::ok()){
		//yaw,pitch
		if(gdir_last.twist.angular.y!=0.0 || gdir_last.twist.angular.z!=0.0){
			static float yaw=0.0;
			static float pit=0.0;
			float yaw_tmp=yaw+gdir_last.twist.angular.z*0.1;
			float pit_tmp=pit-gdir_last.twist.angular.y*0.1*0.2;
			if(-1.5<yaw_tmp && yaw_tmp<1.5)yaw=yaw_tmp;
			if(-0.06<pit_tmp && pit_tmp<0.2)pit=pit_tmp;
			//printf("yaw:%f,pit:%f\n",yaw,pit);
			int data0=(yaw*4000/3.141592)+7500;
			int data1=(pit*4000/3.141592)+7500;
		
			srs_common::CANCode cancode;
			cancode.channel="B";
			cancode.id=8;
			cancode.com=1;
			cancode.length=8;
			cancode.data[0]=(data0>>8)&0xff;
			cancode.data[1]=(data0>>0)&0xff;
			cancode.data[2]=(data1>>8)&0xff;
			cancode.data[3]=(data1>>0)&0xff;
			cancode.data[4]=0;
			cancode.data[5]=0;
			cancode.data[6]=2;
			cancode.data[7]=200/10;
			canlink_pub.publish(cancode);
		}
		//optical
		if(gmode_last.data=="Ready" || gmode_last.data=="Off" || glight_last.data!=""){
			static bool laser=false;
			static bool light=false;
			if(gmode_last.data=="Ready")laser=true;
			else if(gmode_last.data=="Off")laser=false;
			if(glight_last.data=="On")light=true;
			else if(glight_last.data=="Off")light=false;
			gmode_last.data="";
			glight_last.data="";

			srs_common::CANCode cancode;
			cancode.channel="B";
			cancode.id=8;
			cancode.com=4;
			cancode.length=3;
			cancode.data[0]=laser;
			if(light){
				cancode.data[1]=0x80;
				cancode.data[2]=0x80;
			}
			canlink_pub.publish(cancode);
		}
		//gun
		if(gmode_last.data=="Shoot"){
			gmode_last.data="";
			srs_common::CANCode cancode;
			cancode.channel="B";
			cancode.id=8;
			cancode.com=3;
			cancode.length=1;
			cancode.data[0]=3;
			canlink_pub.publish(cancode);
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

