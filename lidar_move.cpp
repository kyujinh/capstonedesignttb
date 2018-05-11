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

#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "opencv2/opencv.hpp"

using namespace std;

#define RAD2DEG(x) ((x)*180./M_PI)

#define PORT 4000
#define IPADDR "10.42.0.133" // myRIO ipadress


int lidar_size;
float lidar_degree[400], scan_angle[181], angle, total_dist;
float lidar_distance[400];
float lidar_obs;

int ball_number;
float ball_X[20];
float ball_Y[20];
float ball_distance[20];
int near_ball;

int c_socket, s_socket;
struct sockaddr_in c_addr;
int len;
int n;
float data[24];
float shortest_obs;
float near_angle;

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
			// if(abs(lidar_degree[i])<90){
			// 		lidar_distance[k] = scan->ranges[i];
			// 		scan_angle[k] = lidar_degree[i];
			// 		k+=1;
				 //printf("ind: %d\n",k);
			 //printf("ang: %f\n",scan_angle[k]);
			}



        // lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        //
        // if(lidar_degree[i]>M_PI/2 && lidar_degree[i]<3*M_PI/4){
        //     lidar_distance[k] = scan->ranges[i];
        //     scan_angle[k] = lidar_degree[i];
        //     ++k;
        // }

        //



}
void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{

    int count = position->size;
    ball_number=count;
    for(int i = 0; i < count; i++)
    {
        ball_X[i] = position->img_b_x[i];
        ball_Y[i] = position->img_b_y[i];
        // std::cout << "degree : "<< ball_degree[i];
        // std::cout << "   distance : "<< ball_distance[i]<<std::endl;
		ball_distance[i] = ball_X[i]*ball_X[i]+ball_Y[i]*ball_X[i];
    }

}
void find_ball()
{
	//data[20]=1;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_integation");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);
    dataInit();

    c_socket = socket(PF_INET, SOCK_STREAM, 0);
    c_addr.sin_addr.s_addr = inet_addr(IPADDR);
    c_addr.sin_family = AF_INET;
    c_addr.sin_port = htons(PORT);

    // if(connect(c_socket, (struct sockaddr*) &c_addr, sizeof(c_addr)) == -1){
    //     printf("Failed to connect\n");
    //     close(c_socket);
    //     return -1;
    // }

		int k = 0;
		while(k< 200){
    // while(ros::ok){
		/////////////////////////////////////////////////////////////////////////////////////////////////
		// // 각노드에서 받아오는 센서 테이터가 잘 받아 왔는지 확인하는 코드 (ctrl + /)을 눌러 주석을 추가/제거할수 있다.///
		/////////////////////////////////////////////////////////////////////////////////////////////////

	 //  for(int i = 0; i < lidar_size; i++)
   // {
	 //
	 //    std::cout << "degree : "<< lidar_degree[i];
	 //    std::cout << "   distance : "<< lidar_distance[i]<<std::endl;
	 //  }
		// for(int i = 0; i < ball_number; i++)
		// {
		// 	std::cout << "ball_X : "<< ball_X[i];
		// 	std::cout << "ball_Y : "<< ball_Y[i]<<std::endl;
   //
		// }



		////////////////////////////////////////////////////////////////
		// // 자율 주행을 예제 코드 (ctrl + /)을 눌러 주석을 추가/제거할수 있다.///
		////////////////////////////////////////////////////////////////
		dataInit();
		float wall_dist = 6.0;
		for(int i = 0; i < 179; i++)
			    {
				// if(lidar_distance[i]<lidar_distance[i+1]){lidar_obs=i;}
				// else if(lidar_distance[i]==lidar_distance[i+1]){lidar_obs=i;}
				// else {lidar_obs=i+1;}
				total_dist = lidar_distance[i] + lidar_distance[i+179];

				if (wall_dist>=total_dist){
					wall_dist = total_dist;
					angle = lidar_degree[i];
				}

			    }
					printf("distance: %f",wall_dist);
					printf(" theta: %f\n",angle);
					//printf(" ind: %d\n",i);

				if (angle < -91.5 && angle > -88.5){
					data[0] = 20;
					data[1] = 20;
					data[2] = 20;
					data[3] = 20;
				}
				else{
					data[0] = 20;
					data[1] = 20;
					data[2] = -20;
					data[3] = -20;
				}
// 		shortest_obs = 6;
// 		near_angle = scan_angle[0];
// 		//printf("ang: %f\n",near_angle);
// 		for(int i = 0; i < 178; i++){
// //printf("ind1: %f\n",near_angle);
// 				if(lidar_distance[i] < 100 && lidar_distance[i+1] < 100){
// 						//printf("ind2: %f\n",lidar_distance[i]-lidar_distance[i+1]);
//
// 	          if(lidar_distance[i]-lidar_distance[i+1]< -0.02){
// 							//printf("ind31: %f\n",near_angle);
// 	            //shortest_obs = lidar_distance[i];
// 	            if (shortest_obs > lidar_distance[i]){
// 	              shortest_obs = lidar_distance[i];
// 	              near_angle = scan_angle[i];
// 	            }
// 	          }
// 	          else if(lidar_distance[i]-lidar_distance[i+1]>0.02){
// 							//printf("ind32: %f\n",near_angle);
// 	            if (shortest_obs > lidar_distance[i+1]){
// 	              shortest_obs = lidar_distance[i+1];
// 	              near_angle = scan_angle[i+1];
// 	            }
// 	          }
// 						//printf("dist: %f\n",lidar_distance[i]);
//
//
//           }
// 	    }
// 			printf("ind31: %f\n",near_angle);



    //  std::cout << " nearest obstacle angle: "<< near_angle <<std::endl;
		// if(ball_number==0 || lidar_obs<0.3)
		// {
		// 		find_ball();
		// }
		// else
		// {
		// 	for(int i = 0; i < ball_number-1; i++)
		// 	    {
		// 		if(ball_distance[i]<ball_distance[i+1]){near_ball=i;}
		// 		else if(ball_distance[i]==ball_distance[i+1]){near_ball=i;}
		// 		else {near_ball=i+1;}
		// 	    }
		// 	if(ball_distance[near_ball]<0.1){data[4]=0; data[5]=0; data[21]=0;}
		// 	else
		// 	{
		// 		data[20]=1;
		// 		if(ball_X[near_ball]>0){data[4]=1;}  else{data[4]=-1;}
		// 		if(ball_Y[near_ball]>0){data[5]=1;}  else{data[5]=-1;}
		// 	}
		// }

		//자율 주행 알고리즘에 입력된 제어데이터(xbox 컨트롤러 데이터)를 myRIO에 송신(tcp/ip 통신)
			k++;
		  write(c_socket, data, sizeof(data));
	    ros::Duration(0.025).sleep();
	    ros::spinOnce();
    }

    return 0;
}
