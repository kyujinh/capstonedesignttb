#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"
#include "core_msgs/tf_result.h"


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "opencv2/opencv.hpp"

using namespace std;

#define RAD2DEG(x) ((x)*180./M_PI)

#define PORT 7785
#define IPADDR "172.16.0.1" // myRIO ipadress

float theta;
int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_obs;

int ball_number;
int red_number;
float ball_X[20], ball_X_r[20];
float ball_Y[20], ball_Y_r[20];
float ball_distance[20], red_distance[20];
int near_ball, near_red;
int ch;
int c_socket, s_socket;
struct sockaddr_in c_addr;
int len;
int n;
double data[6];
int ball_get=0;
#define RAD2DEG(x) ((x)*180./M_PI)

void dataInit()
{
	data[0] = 0; //lx*data[3];
	data[1] = 0; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 0; //rx*data[7];
}


void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{

    int count = scan->scan_time / scan->time_increment;
    lidar_size=count;
    for(int i = 0; i < count; i++)
    {
        lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        lidar_distance[i]=scan->ranges[i];
				std::cout<<lidar_distance[i] <<"\n";
    }
}
void camera_Callback(const core_msgs::tf_result::ConstPtr& position)
{

    int count_b = position->b_x.size();
		int count_r=position->r_x.size();
    ball_number=count_b;
		red_number=count_r;
    for(int i = 0; i < count_b; i++)
    {
        ball_X[i] = position->b_x[i];
        ball_Y[i] = position->b_y[i];
        // std::cout << "degree : "<< ball_degree[i];
        // std::cout << "   distance : "<< ball_distance[i]<<std::endl;
		ball_distance[i] = ball_X[i]*ball_X[i]+ball_Y[i]*ball_Y[i];

    }
		for(int i = 0; i < count_r; i++)
    {
        ball_X_r[i] = position->r_x[i];
        ball_Y_r[i] = position->r_y[i];
        // std::cout << "degree : "<< ball_degree[i];
        // std::cout << "   distance : "<< ball_distance[i]<<std::endl;
		red_distance[i] = ball_X_r[i]*ball_X_r[i]+ball_Y_r[i]*ball_Y_r[i];

    }
}
void find_ball()
{
data[0]=-20;data[1]=-20;data[2]=-20;data[3]=-20;
}

void avoid_red()
{
	while(abs(ball_X_r[near_red])<0.05){
	if(ball_X_r[near_red]<=0){
		//slide right
		data[0]=20;
		data[1]=-20;
		data[2]=20;
		data[3]=-20;
	}
	else{
		//slide left
		data[0]=-20;
		data[1]=20;
		data[2]=-20;
		data[3]=20;
	}
	write(c_socket, data, sizeof(data));
	ros::Duration(0.025).sleep();
	ros::spinOnce();
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_integation");
    ros::NodeHandle n;
		ros::start();
	//	printf("Failed to connect\n");

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    ros::Subscriber sub1 = n.subscribe<core_msgs::tf_result>("/ball_platform", 1000, camera_Callback);
    dataInit();

    c_socket = socket(AF_INET, SOCK_STREAM, 0);
    c_addr.sin_addr.s_addr = inet_addr(IPADDR);
    c_addr.sin_family = AF_INET;
    c_addr.sin_port = htons(PORT);

    if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
         printf("Failed to connect\n");
         close(c_socket);
         return -1;
     }
		 	printf("connected\n");
	/*	if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
			printf("Failed to connect\n");
			close(c_socket);
			//return -1;
		}*/
    while(ros::ok){
		/////////////////////////////////////////////////////////////////////////////////////////////////
		// // 각노드에서 받아오는 센서 테이터가 잘 받아 왔는지 확인하는 코드 (ctrl + /)을 눌러 주석을 추가/제거할수 있다.///
		/////////////////////////////////////////////////////////////////////////////////////////////////

	/*  for(int i = 0; i < lidar_size; i++)
   {
	    //std::cout << "degree : "<< lidar_degree[i];
	    //std::cout << "   distance : "<< lidar_distance[i]<<std::endl;
	  }
		for(int i = 0; i < ball_number; i++)
		{
			//std::cout << "ball_X : "<< ball_X[i];
			//std::cout << "ball_Y : "<< ball_Y[i]<<std::endl;
		}
*/while (ball_get<3){


		dataInit();
		// for(int i = 0; i < lidar_size-1; i++)
		// 	    {
		// 		if(lidar_distance[i]<lidar_distance[i+1]){lidar_obs=i;}
		// 		else if(lidar_distance[i]==lidar_distance[i+1]){lidar_obs=i;}
		// 		else {lidar_obs=i+1;}
		// 	    }
		if(ball_number==0 )
		 {
		 		find_ball();
			//	printf("where is the ball????\n");
		 }
		else
		 {


			for(int i = 0; i < ball_number-1; i++)
			    {
				if(ball_distance[i]<ball_distance[i+1]){near_ball=i;}
				else if(ball_distance[i]==ball_distance[i+1]){near_ball=i;}
				else {near_ball=i+1;}}
			if(abs(ball_X[near_ball])>0.03){
				if(ball_X[near_ball]  > 0){data[0]=30;data[1]=30;data[2]=30;data[3]=30;
					printf("turn left\n");}
				else{data[0]=-30;data[1]=-30;data[2]=-30;data[3]=-30;
						printf("turn right\n");}
					}
			else{
					//	printf("here?%f\n",ball_distance[near_ball]);
					int j=0;
	 			 	int interrupt[20];
					for(int i = 0; i < red_number-1; i++)
					    {
								if(abs(ball_X_r[i]<0.1){
									interrupt[j]=i;
									j=j+1;

						}
						else {}
					}
					interrupt.resize(j+1);
					for(int i=0; i<interrupt.size()-1;i++){
						if(red_distance[interrupt[i]]<red_distance[interrupt[i+1]]){near_red=i;}
						else if(red_distance[interrupt[i]]==red_distance[interrupt[i+1]]){near_red=i;}
						else {near_red=interrupt[i+1];}}
					}
						if(abs(ball_X_r[near_red])<0.05){
							avoid_red();
						}
						else{
							if(ball_distance[near_ball]>0.45)
							{
								data[0]=40;data[1]=40;data[2]=-40;data[3]=-40;
								printf("go straight\n");
							}
							else{
								 for (int i=0;i<150;i++)
								 {
									 data[0]=40;data[1]=40;data[2]=-40;data[3]=-40;
									 write(c_socket, data, sizeof(data));
									 ros::Duration(0.025).sleep();
								 }
								 for (int i=0;i<100;i++)
								 {
									 data[0]=10;data[1]=10;data[2]=-10;data[3]=-10;data[4]=90*(ball_get+1);
									 write(c_socket, data, sizeof(data));
									 ros::Duration(0.025).sleep();
								 }
								 data[0]=0;data[1]=0;data[2]=0;data[3]=0;
								 ball_get++;
							}
						}
					}

		printf("distance:%f\n",ball_distance[near_ball]);
	}
		//자율 주행 알고리즘에 입력된 제어데이터(xbox 컨트롤러 데이터)를 myRIO에 송신(tcp/ip 통신)

	//	printf("data0:%f\n",data[0]);
	//	printf("data1:%f\n",data[1]);
	//	printf("data2:%f\n",data[2]);
	//	printf("data3:%f\n",data[3]);
	//printf("??\n");

			write(c_socket, data, sizeof(data));
	    ros::Duration(0.025).sleep();
	    ros::spinOnce();
		//	std::cout << "data : "<<data[0];

}
			data[0]=0;data[1]=0;data[2]=-0;data[3]=-0;
			write(c_socket, data, sizeof(data));
 	    ros::Duration(0.025).sleep();
 	    ros::spinOnce();

}
return 0;

}
