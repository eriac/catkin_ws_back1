#include "ros/ros.h"
  
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "srs002host/LegPoint.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_listener.h"

#include "math.h" 
#include <string>
#include <sstream>

float point_home[6][3]={
	{+0.18, +0.17,  -0.07},
	{0.0,   +0.175, -0.07},
	{-0.18, +0.17,  -0.07},
	{+0.18, -0.17,  -0.07},
	{0.0,   -0.175, -0.07},
	{-0.18, -0.17,  -0.07}
};

geometry_msgs::TwistStamped twist_last;
void twist_callback(const geometry_msgs::TwistStamped twist){	
	twist_last.header.seq      = twist.header.seq;
	twist_last.header.stamp    = twist.header.stamp;
	twist_last.header.frame_id = twist.header.frame_id;
	twist_last.twist.linear.x  = twist.twist.linear.x;
	twist_last.twist.linear.y  = twist.twist.linear.y;
	twist_last.twist.linear.z  = twist.twist.linear.z;
	twist_last.twist.angular.x = twist.twist.angular.x;
	twist_last.twist.angular.y = twist.twist.angular.y;
	twist_last.twist.angular.z = twist.twist.angular.z;
}

std::string movemode_last="Rest";
void movemode_callback(const std_msgs::String movemode){	
	movemode_last=movemode.data;
}
/* std::string state
1st char M: A move on, C: A move counter, N: A move default
2nd char F: A move up, G: A stand
3rd char M: B move on, C: B move counter, N: A move default
4th char F: B move up, G: B stand
*/
int walk_set_points(float point_out[6][3], float walk_command[3], std::string state_ab){
	float walk_scale1=-0.05;
	float walk_scale2=-0.02;
	float walk_scale3=-0.2;
	float walk_up=0.04;
	if(fabsf(walk_command[0])+fabsf(walk_command[2])>1.0){
		printf("correct\n");
		float tmp=fmin(fabsf(walk_command[0]),1-fabsf(walk_command[2]));
		if(walk_command[0]>0.0)walk_command[0]=tmp;
		else walk_command[0]=-tmp;
	}
	printf("command:%f,%f,%f\n",walk_command[0],walk_command[1],walk_command[2]);
	if(state_ab.length()==4){
		for(int i=0;i<6;i+=2){//group A
			if(state_ab[0]=='M'){
				float data_x=point_home[i][0]+walk_command[0]*walk_scale1;
				float data_y=point_home[i][1]+walk_command[1]*walk_scale2;
				float theta=walk_command[2]*walk_scale3;
				point_out[i][0]=cos(theta)*data_x-sin(theta)*data_y;
				point_out[i][1]=sin(theta)*data_x+cos(theta)*data_y;
			}
			else if(state_ab[0]=='C'){
				float data_x=point_home[i][0]-walk_command[0]*walk_scale1;
				float data_y=point_home[i][1]-walk_command[1]*walk_scale2;
				float theta=-walk_command[2]*walk_scale3;
				point_out[i][0]=cos(theta)*data_x-sin(theta)*data_y;
				point_out[i][1]=sin(theta)*data_x+cos(theta)*data_y;
			}
			else if(state_ab[0]=='N'){
				point_out[i][0]=point_home[i][0];
				point_out[i][1]=point_home[i][1];
			}
			else return -1;

			if     (state_ab[1]=='F')point_out[i][2]=point_home[i][2]+walk_up;
			else if(state_ab[1]=='G')point_out[i][2]=point_home[i][2];
			else return -1;
		}
		for(int i=1;i<6;i+=2){//group B
			if(state_ab[2]=='M'){
				float data_x=point_home[i][0]+walk_command[0]*walk_scale1;
				float data_y=point_home[i][1]+walk_command[1]*walk_scale2;
				float theta=walk_command[2]*walk_scale3;
				point_out[i][0]=cos(theta)*data_x-sin(theta)*data_y;
				point_out[i][1]=sin(theta)*data_x+cos(theta)*data_y;
			}
			else if(state_ab[2]=='C'){
				float data_x=point_home[i][0]-walk_command[0]*walk_scale1;
				float data_y=point_home[i][1]-walk_command[1]*walk_scale2;
				float theta=-walk_command[2]*walk_scale3;
				point_out[i][0]=cos(theta)*data_x-sin(theta)*data_y;
				point_out[i][1]=sin(theta)*data_x+cos(theta)*data_y;
			}
			else if(state_ab[2]=='N'){
				point_out[i][0]=point_home[i][0];
				point_out[i][1]=point_home[i][1];
			}
			else return -1;

			if     (state_ab[3]=='F')point_out[i][2]=point_home[i][2]+walk_up;
			else if(state_ab[3]=='G')point_out[i][2]=point_home[i][2];
			else return -1;
		}
		
	}
	else return -1;
}
float twist_3d_length(geometry_msgs::TwistStamped twist){
	float sum=0.0;
	sum+=twist.twist.linear.x *twist.twist.linear.x;
	sum+=twist.twist.linear.y *twist.twist.linear.y;
	sum+=twist.twist.angular.z*twist.twist.angular.z;
	return sum;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "leg6_commander");
	ros::NodeHandle n;
	tf::TransformListener tflistener;

	//publish
	ros::Publisher  legpoint_pub = n.advertise<srs002host::LegPoint>("LegPoint_out", 1000);
	ros::Publisher  marker_pub   = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
	//subscriibe
	ros::Subscriber twist_sub    = n.subscribe("twist", 10, twist_callback); 
	ros::Subscriber movemode_sub = n.subscribe("movemode", 10, movemode_callback); 
	
	ros::Duration(1.0).sleep();

	geometry_msgs::PointStamped mypoint;
	geometry_msgs::PointStamped legpoint;
	mypoint.header.stamp=ros::Time(0);
	mypoint.header.frame_id="base_link";
	mypoint.point.x=1.0;
	mypoint.point.y=0.0;
	mypoint.point.z=0.0;

	tflistener.waitForTransform("leg0_link0", "base_link", ros::Time(0), ros::Duration(1.0));	
	tflistener.transformPoint("leg0_link0",ros::Time(0),mypoint,"base_link",legpoint);
	
	printf("my  %f %f %f\n", mypoint.point.x, mypoint.point.y, mypoint.point.z);
	printf("leg %f %f %f\n",legpoint.point.x,legpoint.point.y,legpoint.point.z);
	
	std::string movemode_now="Rest";
	std::string submode="Rest";
	int submode_delay=0;
	float point_publish[6][3];
	int point_delay=1000;
	int point_enable=0;

	ros::Rate loop_rate(20); 
	while (ros::ok()){
		printf("loop\n");
		if(movemode_now=="Rest"){
			if(submode_delay<=0){
				if(submode=="Rest"){
					for(int i=0;i<6;i++){
						point_publish[i][0]=point_home[i][0];
						point_publish[i][1]=point_home[i][1];
						point_publish[i][2]=0;
					}
					point_delay=900;
					point_enable=1;
					submode="Wait";
					submode_delay=1000;
				}
				else if(submode=="Wait"){
					if((movemode_last!="Rest")&&(movemode_last!="")){
						movemode_now=movemode_last;
						submode     =movemode_last;
					}
				}
			}
			else submode_delay-=(1000/20);
		}
		else if(movemode_now=="Stand"){
			if(submode_delay<=0){
				if(submode=="Stand"){
					printf("do Start\n");
					for(int i=0;i<6;i++){
						point_publish[i][0]=point_home[i][0];
						point_publish[i][1]=point_home[i][1];
						point_publish[i][2]=point_home[i][2];
					}
					point_delay=900;
					point_enable=1;
					submode="Wait";
					submode_delay=1000;
				}
				else if(submode=="Wait"){
					if((movemode_last!="Stand")&&(movemode_last!="")){
						movemode_now=movemode_last;
						submode     =movemode_last;
					}
					else{
						submode="Stand2";
						submode_delay=0;
					}
				}
				else if(submode=="Stand2"){
					if((movemode_last!="Stand")&&(movemode_last!="")){
						movemode_now=movemode_last;
						submode     =movemode_last;
					}
					else{
						for(int i=0;i<6;i++){
							//point_publish[i][0]=point_home[i][0]+twist_last.twist.linear.x;
							//point_publish[i][1]=point_home[i][1]+twist_last.twist.linear.y;
							//point_publish[i][2]=point_home[i][2]+twist_last.twist.linear.z;
							point_publish[i][0]=point_home[i][0];
							point_publish[i][1]=point_home[i][1];
							point_publish[i][2]=point_home[i][2];
						}
						point_delay=100;
						point_enable=1;
						submode_delay=0;
					}
				}
			}
			else submode_delay-=(1000/20);
		}
		else if(movemode_now=="Walk"){
			static float walk_command[3]={0.0,0.0,0.0};
			if(submode_delay<=0){
				if(submode=="Walk"){
					if(twist_3d_length(twist_last)>0.1*0.1){
						submode="A0";
						submode_delay=100;
						walk_command[0]=0;
						walk_command[1]=0;
						walk_command[2]=0;
						walk_set_points(point_publish,walk_command,"NFNG");
						point_delay=100;
						point_enable=1;
					}
					else{
						submode="Stop";
						submode_delay=100;
						walk_command[0]=0;
						walk_command[1]=0;
						walk_command[2]=0;
						walk_set_points(point_publish,walk_command,"NGNG");
						point_delay=100;
						point_enable=1;
					}
				}
				else if(submode=="A0"){
					submode="A1";
					submode_delay=500;
					walk_command[0]=+twist_last.twist.linear.y*0.07;
					walk_command[1]=-twist_last.twist.linear.x*0.07;
					walk_command[2]=+twist_last.twist.angular.z*0.07;
					walk_set_points(point_publish,walk_command,"MFCG");
					point_delay=500;
					point_enable=1;
				}
				else if(submode=="A1"){
					submode="A2";
					submode_delay=200;
					walk_set_points(point_publish,walk_command,"MGCG");
					point_delay=200;
					point_enable=1;
				}
				else if(submode=="A2"){
					submode="A3";
					submode_delay=100;
					walk_set_points(point_publish,walk_command,"MGCF");
					point_delay=100;
					point_enable=1;
				}
				else if(submode=="A3"){
					if(twist_3d_length(twist_last)>0.1*0.1){
						submode="B1";
						submode_delay=500;
						walk_command[0]=+twist_last.twist.linear.y;
						walk_command[1]=-twist_last.twist.linear.x;
						walk_command[2]=+twist_last.twist.angular.z;
						walk_set_points(point_publish,walk_command,"CGMF");
						point_delay=500;
						point_enable=1;
					}
					else{
						submode="B4";
						submode_delay=300;
						walk_set_points(point_publish,walk_command,"NGNF");
						point_delay=300;
						point_enable=1;
					}
				}
				else if(submode=="A4"){
					submode="Stop";
					submode_delay=200;
					walk_set_points(point_publish,walk_command,"NGNG");
					point_delay=200;
					point_enable=1;
				}
				else if(submode=="B1"){
					submode="B2";
					submode_delay=200;
					walk_set_points(point_publish,walk_command,"CGMG");
					point_delay=200;
					point_enable=1;
				}
				else if(submode=="B2"){
					submode="B3";
					submode_delay=100;
					walk_set_points(point_publish,walk_command,"CFMG");
					point_delay=100;
					point_enable=1;
				}
				else if(submode=="B3"){
					if(twist_3d_length(twist_last)>0.1*0.1){
						submode="A1";
						submode_delay=500;
						walk_command[0]=+twist_last.twist.linear.y;
						walk_command[1]=-twist_last.twist.linear.x;
						walk_command[2]=+twist_last.twist.angular.z;
						walk_set_points(point_publish,walk_command,"MFCG");
						point_delay=500;
						point_enable=1;
					}
					else{
						submode="A4";
						submode_delay=300;
						walk_set_points(point_publish,walk_command,"NFNG");
						point_delay=300;
						point_enable=1;
					}
				}
				else if(submode=="B4"){
					submode="Stop";
					submode_delay=200;
					walk_set_points(point_publish,walk_command,"NGNG");
					point_delay=200;
					point_enable=1;
				}
				else if(submode=="Stop"){
					if(twist_3d_length(twist_last)>0.1*0.1){
						submode="A0";
						submode_delay=100;
						walk_command[0]=0;
						walk_command[1]=0;
						walk_command[2]=0;
						walk_set_points(point_publish,walk_command,"NFNG");
						point_delay=100;
						point_enable=1;
					}
					else{
						if((movemode_last!="Walk")&&(movemode_last!="")){
							movemode_now=movemode_last;
							submode     =movemode_last;
						}
						else submode_delay=0;
					}
				}
			}
			else submode_delay-=(1000/20);
		}
		else{
			printf("Unknown mode:%s\n",movemode_now.c_str());
		}
		if(point_enable){
			point_enable=0;
			//for marker
			visualization_msgs::MarkerArray markers;
			markers.markers.resize(6);
    				
			for(int i=0;i<6;i++){
				srs002host::LegPoint legpoint;
				legpoint.header.stamp=ros::Time::now();
				legpoint.header.frame_id="base_link";
				legpoint.target_id=i;
				if     (i==0)legpoint.target_frame_id="leg0_link0";
				else if(i==1)legpoint.target_frame_id="leg1_link0";
				else if(i==2)legpoint.target_frame_id="leg2_link0";
				else if(i==3)legpoint.target_frame_id="leg3_link0";
				else if(i==4)legpoint.target_frame_id="leg4_link0";
				else         legpoint.target_frame_id="leg5_link0";
				legpoint.delay=point_delay;

				legpoint.point.x=point_publish[i][0];
				legpoint.point.y=point_publish[i][1];
				legpoint.point.z=point_publish[i][2];
				legpoint_pub.publish(legpoint);

				//for marker
				markers.markers[i].header.frame_id = "/base_link";
    				markers.markers[i].header.stamp = ros::Time::now();
			    	markers.markers[i].ns = "basic_shapes";
    				markers.markers[i].id = i;
    				markers.markers[i].type = visualization_msgs::Marker::SPHERE;
    				markers.markers[i].action = visualization_msgs::Marker::ADD;

				markers.markers[i].pose.position.x = point_publish[i][0];
				markers.markers[i].pose.position.y = point_publish[i][1];
				markers.markers[i].pose.position.z = point_publish[i][2];
				markers.markers[i].pose.orientation.x = 0.0;
				markers.markers[i].pose.orientation.y = 0.0;
				markers.markers[i].pose.orientation.z = 0.0;
				markers.markers[i].pose.orientation.w = 1.0;

				markers.markers[i].scale.x = 0.02;
				markers.markers[i].scale.y = 0.02;
				markers.markers[i].scale.z = 0.02;
				if(i%2==0){
					markers.markers[i].color.r = 1.0f;
					markers.markers[i].color.g = 0.0f;
					markers.markers[i].color.b = 0.0f;
					markers.markers[i].color.a = 0.5;
				}
				else{
					markers.markers[i].color.r = 0.0f;
					markers.markers[i].color.g = 1.0f;
					markers.markers[i].color.b = 0.0f;
					markers.markers[i].color.a = 0.5;
				}
				markers.markers[i].lifetime = ros::Duration();

			}
			marker_pub.publish(markers);

		}
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

