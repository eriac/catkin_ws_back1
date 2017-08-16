#include "ros/ros.h"
  
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"

#include "math.h"
#include <sstream>
#include <string>

sensor_msgs::Joy joy_last;
bool joy_new;
void joy_callback(const sensor_msgs::Joy& joy_msg){
	joy_last=joy_msg;
	joy_new=true;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "omni_commander");
	ros::NodeHandle n;
	//publish
	ros::Publisher serial_pub = n.advertise<std_msgs::String>("/Serial_out", 1000);
	//Subscribe
	ros::Subscriber joy_sub     = n.subscribe("/joy", 10, joy_callback); 
	
	ros::Rate loop_rate(10); 
	while (ros::ok()){
		float joy_in[3]={0.0,0.0,0.0};
		static int joy_push[20]={0};
		static int joy_push2[20]={0};
		int joy_press[20]={0};
		int joy_release[20]={0};
		static std::string mode="none";
		static int mode_counter=0;

		bool sub_enable=false;
		/* move mode
		none: first state
		move: moving
		stop: sent stop
		*/
		if(joy_new){
			printf("new %i\n",mode_counter);
			joy_in[0]=-joy_last.axes[0];
			joy_in[1]=joy_last.axes[1];
			joy_in[2]=-joy_last.axes[2];

			for(int i=0;i<17;i++){
				joy_push2[i]  =joy_push[i];
				joy_push[i]   =joy_last.buttons[i];
				joy_press[i]  =joy_push[i]&(!joy_push2[i]);
				joy_release[i]=(!joy_push[i])&joy_push2[i];
			}

			joy_new=false;
			mode="move";
			mode_counter=0;
			sub_enable=true;
		}
	 	else{
			printf("not new %i\n",mode_counter);
			mode_counter++;
			if(mode_counter>2*10){
				mode="stop";
			}
			else mode="none";
		}



		if(mode=="none"){
			//nothing
			printf("none\n");
		}
		else if(mode=="move"){
			
			double ver1=0.4;
			double ver2=0.2;
			if(joy_push[10]){//highspeed
				ver1=0.7;
				ver2=0.3;				
			}
			double motor_out[4]={0.0,0.0,0.0,0.0};
			double alpha=40.0/180.0*3.141592;
			double right=90.0/180.0*3.141592;
			double motor_normal[4]={alpha+right, -alpha-right, -alpha+right, alpha-right};
			for(int i=0;i<4;i++){
				motor_out[i]=ver1*(cos(motor_normal[i])*joy_in[0]+sin(motor_normal[i])*joy_in[1])-ver2*joy_in[2];
			}
			//printf("0:%f, 1:%f, 2:%f, 3:%f\n",motor_out[0],motor_out[1],motor_out[2],motor_out[3]);
			
			std::string data_out="#SRS1.OMNI:";
			for(int i=0;i<4;i++){
				float motor_mid;
				if     (motor_out[i]>1.0) motor_mid=1.0;
				else if(motor_out[i]<-1.0)motor_mid=-1.0;
				else motor_mid=motor_out[i];
				char tmp_data[8];
				sprintf(tmp_data,"%04X",(int)(motor_mid*2048+4096));
				data_out+=std::string(tmp_data);
			}
			data_out+=";";
			//add
			std_msgs::String serial_msg;
			serial_msg.data=data_out;
			serial_pub.publish(serial_msg);
		}
		else if(mode=="stop"){
			//printf("stop %i\n",mode_counter);
		}
	 	
		if(sub_enable){
			//dribbler
			static bool dribbler_enable=false;
			if(joy_press[13]){
				if(dribbler_enable){
					dribbler_enable=false;
					printf("DRIB off\n");
					std_msgs::String serial_msg;
					serial_msg.data="#SRS1.DRIB:00;";
					serial_pub.publish(serial_msg);
				}
				else{
					dribbler_enable=true;
					printf("DRIB on\n");
					std_msgs::String serial_msg;
					serial_msg.data="#SRS1.DRIB:80;";
					serial_pub.publish(serial_msg);
				}
			}

			//kicker
			if(joy_press[11]){
				printf("KICK charge\n");
				std_msgs::String serial_msg;
				serial_msg.data="#SRS1.KICK:01;";
				serial_pub.publish(serial_msg);
			}
			if(joy_release[11]){
				printf("KICK shoot\n");
				std_msgs::String serial_msg;
				serial_msg.data="#SRS1.KICK:02;";
				serial_pub.publish(serial_msg);
			}
		}		

		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

