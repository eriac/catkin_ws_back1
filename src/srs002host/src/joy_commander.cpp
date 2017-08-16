#include "ros/ros.h"
  
#include "std_msgs/Int32.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include <sstream>
#include <string>

ros::Publisher twist_pub;
ros::Publisher movemode_pub;
ros::Publisher gundir_pub;
ros::Publisher gunmode_pub;
ros::Publisher gunlight_pub;

std::string movemode_last;
void joy_callback(const sensor_msgs::Joy& joy_msg){
	int joy_buttons[17]={0};
	static int joy_blast[17]={0};
	int joy_bpress[17]={0};
	int joy_brelease[17]={0};

	float joy_axes[4]={0};
	for(int i=0;i<17;i++){
		joy_buttons[i] =joy_msg.buttons[i];
		joy_bpress[i]  =joy_buttons[i]&&(!joy_blast[i]);
		joy_brelease[i]=(!joy_buttons[i])&&joy_blast[i];
		joy_blast[i]   =joy_buttons[i];
	}
	for(int i=0;i<4 ;i++)joy_axes[i]   =joy_msg.axes[i];
	
	//for move
	geometry_msgs::TwistStamped twist;
	twist.header.stamp = ros::Time::now();
	twist.twist.linear.x=joy_axes[0];
	twist.twist.linear.y=-joy_axes[1];
	twist.twist.angular.z=-joy_axes[2];
	twist_pub.publish(twist);
	
	std_msgs::String movemode;
	//for PS3
	if     (joy_buttons[12])movemode_last="Rest";
	else if(joy_buttons[13])movemode_last="Stand";
	else if(joy_buttons[14])movemode_last="Walk";
	else if(joy_buttons[15])movemode_last="4";
	
	//for Elecom
	/*
	if     (joy_buttons[1])movemode_last="Rest";
	else if(joy_buttons[3])movemode_last="Stand";
	else if(joy_buttons[2])movemode_last="Walk";
	else if(joy_buttons[0])movemode_last="4";
	*/
	movemode.data=movemode_last;
	movemode_pub.publish(movemode);

	
	//for gundir
	float gdir[2]={0.0,0.0};
	float delta=0;
	if(joy_buttons[11])delta=0.1;
	else delta=1.0;
	if     (joy_buttons[4])gdir[0]=+delta;
	else if(joy_buttons[6])gdir[0]=-delta;
	if     (joy_buttons[5])gdir[1]=+delta;
	else if(joy_buttons[7])gdir[1]=-delta;
	
	geometry_msgs::TwistStamped guntwist;
	guntwist.header.stamp = ros::Time::now();
	guntwist.twist.angular.y=gdir[0];
	guntwist.twist.angular.z=gdir[1];
	gundir_pub.publish(guntwist);

	//for gunmode ready shoot off
	std_msgs::String gmode;
	if(joy_bpress[11])gmode.data="Ready";
	else if(joy_buttons[11]&&joy_bpress[9])gmode.data="Shoot";
	else if(joy_brelease[11])gmode.data="Off";
	if(gmode.data!="")gunmode_pub.publish(gmode);

	//for gunlight
	std_msgs::String glight;
	static bool gunlight_on=false;
	if(joy_bpress[10]){
		gunlight_on=!gunlight_on;
		if(gunlight_on)glight.data="On";
		else glight.data="Off";
		gunlight_pub.publish(glight);
	}
	

	/*
	printf("press:");
	for(int i=0;i<17;i++)printf("%d,",joy_bpress[i]);
	printf("\n");
	printf("relea:");
	for(int i=0;i<17;i++)printf("%d,",joy_brelease[i]);
	printf("\n");
	*/

}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "joy_commander");
	ros::NodeHandle n;
	//publish
	twist_pub    = n.advertise<geometry_msgs::TwistStamped>("twist", 1000);
	movemode_pub = n.advertise<std_msgs::String>("movemode", 1000);
	gundir_pub   = n.advertise<geometry_msgs::TwistStamped>("gundir", 1000);
	gunmode_pub  = n.advertise<std_msgs::String>("gunmode", 1000);
	gunlight_pub = n.advertise<std_msgs::String>("gunlight", 1000);

	//Subscribe
	ros::Subscriber joy_sub     = n.subscribe("joy", 10, joy_callback); 
	
	   
	ros::Rate loop_rate(10); 
	while (ros::ok()){		 		
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

