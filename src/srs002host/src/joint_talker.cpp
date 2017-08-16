#include "ros/ros.h"
  
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "crane_urdf/CANLink.h"

#include <sstream>

int   joy_buttons[17]={0};
float joy_axes[4]={0};
void joy_callback(const sensor_msgs::Joy& joy_msg){
	for(int i=0;i<17;i++)joy_buttons[i]=joy_msg.buttons[i];
	for(int i=0;i<4 ;i++)joy_axes[i]   =joy_msg.axes[i];
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "reflect_rviz_in_crane");
	ros::NodeHandle n;
	//publish command message to joints/servos of arm
	ros::Publisher  canlink_pub = n.advertise<crane_urdf::CANLink>("/CANLink_out", 1000);
	ros::Subscriber joy_sub     = n.subscribe("/joy", 10, joy_callback); 
	
	crane_urdf::CANLink canlink;
	   
	ros::Rate loop_rate(10); 
	while (ros::ok()){
		//del
		joy_axes[0]+=0.01;
		joy_axes[1]+=0.01;
		joy_axes[2]+=0.01;

		//canlink.header.stamp = ros::Time::now();
		int data0=(joy_axes[0]*4000/3.141592)+7500;
		int data1=(joy_axes[1]*4000/3.141592)+7500;
		int data2=(joy_axes[2]*4000/3.141592)+7500;
		int data0_0=(data0>>8)&0xff;
		int data0_1=(data0>>0)&0xff;

		printf("raw%d,can0:%d,can1:%d\n",data0,data0_0,data0_1);

		for(int i=0;i<6;i++){
			canlink.id=i;
			canlink.com=1;
			canlink.length=8;
			//canlink.data.resize(8);
			canlink.data[0]=(data0>>8)&0xff;
			canlink.data[1]=(data0>>0)&0xff;
			canlink.data[2]=(data1>>8)&0xff;
			canlink.data[3]=(data1>>0)&0xff;
			canlink.data[4]=(data2>>8)&0xff;
			canlink.data[5]=(data2>>0)&0xff;
			canlink.data[6]=100;
			canlink.data[7]=2;
			canlink_pub.publish(canlink);
		}
			 		
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

