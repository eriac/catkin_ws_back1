#include "ros/ros.h"
#include "srs_common/SerialCode.h"
#include "srs_common/CANCode.h"

#include <sstream>
#include <string>

int hex_convert(std::string input){
	int data_size=input.size();
	int output=0;
	if(data_size==0)return -1;
	for(int i=0;i<data_size;i++){
		if     ('0'<=input[i] && input[i]<='9')output|=(input[i]-'0')<<((data_size-i-1)*4);
		else if('A'<=input[i] && input[i]<='F')output|=(input[i]-'A'+10)<<((data_size-i-1)*4);
		else return -1;
	}
	return output;
}

ros::Publisher  canlink_pub;
void seriallink_callback(const srs_common::SerialCode& serialcode_msg){
	if(serialcode_msg.command[0]=="CANLINK"){
		printf("CANLINK_conv\n");
		srs_common::CANCode outdata;
		outdata.channel=serialcode_msg.command[1];
		outdata.id=0;//default?
		outdata.com=0;//default?
		outdata.remote=false;
		for(int i=0;i<4;i++){
			if(serialcode_msg.option[i]=="ID"){
				int ret=hex_convert(serialcode_msg.suboption[i]);
				if(0<=ret && ret<=15)outdata.id=ret;
			}
			else if(serialcode_msg.option[i]=="COM"){
				int ret=hex_convert(serialcode_msg.suboption[i]);
				if(0<=ret && ret<=15)outdata.com=ret;
			}
			else if(serialcode_msg.option[i]=="REMOTE"){
				outdata.remote=true;
			}
		}
		int data_size=serialcode_msg.datanum;
		if(data_size>8)data_size=8;
		for(int i=0;i<data_size;i++){
			outdata.data[i]=serialcode_msg.data[i];
		}
		outdata.length=data_size;
		canlink_pub.publish(outdata);
	}
} 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "seriallink_to_canlink");
	ros::NodeHandle n;
	//publicher
	canlink_pub = n.advertise<srs_common::CANCode>("CANLink_in", 1000);
	//Subscriber
	ros::Subscriber seriallink_sub = n.subscribe("SerialLink_in", 10, seriallink_callback); 
	
	ros::Rate loop_rate(100); 
	while (ros::ok()){ 		
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

