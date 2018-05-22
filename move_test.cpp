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
float lidar_degree[400], scan_angle[181], angle, total_dist,ref1,ref2;
float lidar_distance[400],lidar_dist_new[400];
float lidar_obs;
float home_x;
int ball_number;
int red_number;
int green_number;
float ball_X[20], ball_X_r[20], ball_X_g[20];
float ball_Y[20], ball_Y_r[20], ball_Y_g[20];
float ball_distance[20];
float red_distance[20];
float green_distance[20];
int near_ball, near_red;
int ch;
int c_socket, s_socket;
struct sockaddr_in c_addr;
int len;
int n;
int state=0;
double data[6];
int ball_get=0;
#define RAD2DEG(x) ((x)*180./M_PI)
int slide_dist;
float shortest=10.0;
float shortest_angle;
int finish=0;


void dataInit()
{
	data[0] = 0; //lx*data[3];
	data[1] = 0; //ly*data[3];
	data[2] = 0; //GamepadStickAngle(_dev, STICK_LEFT);
	data[3] = 0; //GamepadStickLength(_dev, STICK_LEFT);
	data[4] = 45; //rx*data[7];
}


void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{

    //int count = scan->lidar_distance.size();
		int count = scan->scan_time / scan->time_increment;
		lidar_size=count;
    for(int i = 0; i < count; i++)
    {
        //lidar_degree[i] = scan->lidar_angle[i];
        //lidar_distance[i]=scan->lidar_distance[i];
				lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);
				lidar_distance[i]=scan->ranges[i];
				float x = lidar_distance[i]*sin(scan->angle_min + scan->angle_increment * i)+0.165;
				float y = lidar_distance[i]*cos(scan->angle_min + scan->angle_increment * i);
				lidar_dist_new[i]= sqrt(y*y+x*x);


				//std::cout<<lidar_distance[i] <<"\n";
		//		printf("lidar_distance %f",lidar_distance[i]);
		//		printf(" new_distance %f\n",lidar_dist_new[i]);
		//		printf(" angle %f\n",lidar_degree[i]);

				if(shortest>lidar_dist_new[i]){
					shortest=lidar_dist_new[i];
					shortest_angle=lidar_degree[i];
				}
    }
}
void camera_Callback(const core_msgs::tf_result::ConstPtr& position)
{
    int count_b = position->b_x.size();
		int count_r=position->r_x.size();
		int count_g=position->g_x.size();
		ball_number=count_b;
		red_number=count_r;
		green_number=count_g;
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
		for(int i=0;i<count_g;i++)
		{
			ball_X_g[i]=position->g_x[i];
			ball_Y_g[i]=position->g_y[i];
		}
	//	printf("ball_number %d\n",ball_number);
}
/*void find_ball()
{
	for(int i=0;i<50;i++)
	{
		data[0]=-20;data[1]=-20;data[2]=-20;data[3]=-20;
		write(c_socket, data, sizeof(data));
		ros::Duration(0.025).sleep();
		ros::spinOnce();
		if (ball_number!=0){
			break;
		}
	}
	for(int i=0;i<50;i++)
	{
		data[0]=20;data[1]=20;data[2]=-20;data[3]=-20;
		write(c_socket, data, sizeof(data));
		ros::Duration(0.025).sleep();
		ros::spinOnce();
		if (ball_number!=0){
			break;
		}
	}
}*/
void find_ball()
{
data[0]=-20;data[1]=-20;data[2]=-20;data[3]=-20;
}
void avoid_wall()
{
	if (shortest_angle>0 && shortest_angle<=90){
		data[0]=40;
		data[1]=0;
		data[2]=-40;
		data[3]=0;
	}
	else if(shortest_angle>90 && shortest_angle<=180){
		data[0]=0;
		data[1]=-40;
		data[2]=0;
		data[3]=40;
	}
	else if(shortest_angle<=0 && shortest_angle>-90){
		data[0]=-40;
		data[1]=0;
		data[2]=40;
		data[3]=0;
	}
	else{
		data[0]=0;
		data[1]=40;
		data[2]=0;
		data[3]=-40;

	}

}
void avoid_red(int slide_dist)
{

	if(ball_X_r[near_red]>0){
		//slide right

		for(int i=0; i<int(slide_dist);i++)
		{
			data[0]=30;
			data[1]=30;
			data[2]=-20;
			data[3]=-20;
		//	printf("slide_right\n");
		//	printf("data0:%f\n",data[0]);
		//	printf("data1:%f\n",data[1]);
		//	printf("data2:%f\n",data[2]);
		//	printf("data3:%f\n",data[3]);


			write(c_socket, data, sizeof(data));
			ros::Duration(0.025).sleep();
		}

	}
	else{
		//slide left
		for(int i=0; i<int(slide_dist);i++)
		{
		data[0]=-30;
		data[1]=-30;
		data[2]=20;
		data[3]=20;
	//	printf("slide_left\n");
	//	printf("data0:%f\n",data[0]);
	//	printf("data1:%f\n",data[1]);
	//	printf("data2:%f\n",data[2]);
	//	printf("data3:%f\n",data[3]);


		write(c_socket, data, sizeof(data));
		ros::Duration(0.025).sleep();
	}
	}

	//printf("data0:%f\n",data[0]);
	//printf("data1:%f\n",data[1]);
	//printf("data2:%f\n",data[2]);
	//printf("data3:%f\n",data[3]);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_test");
    ros::NodeHandle n;
		ros::start();
	//	printf("Failed to connect\n");

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    ros::Subscriber sub1 = n.subscribe<core_msgs::tf_result>("/ball_platform", 1, camera_Callback);
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
//		int k =0;
	//	while(k<500){
    while(ros::ok){
// 		/////////////////////////////////////////////////////////////////////////////////////////////////
// 		// // 각노드에서 받아오는 센서 테이터가 잘 받아 왔는지 확인하는 코드 (ctrl + /)을 눌러 주석을 추가/제거할수 있다.///
// 		/////////////////////////
// 		////////////////////////////////////////////////////////////////////////
//
// 	  /*for(int i = 0; i < lidar_size; i++)
//    {
// 	    //std::cout << "degree : "<< lidar_degree[i];
// 	    //std::cout << "   distance : "<< lidar_distance[i]<<std::endl;
// 	  }
// 		for(int i = 0; i < ball_number; i++)
// 		{
// 			//std::cout << "ball_X : "<< ball_X[i];
// 			//std::cout << "ball_Y : "<< ball_Y[i]<<std::endl;
// 		}*/
while (ball_get<3){
	/*	if(shortest<0.28){
			avoid_wall();
		}*/
	//	else{
			if(ball_number==0 )
		 {
			 find_ball();
				printf("find ball\n");
		 }
		else
		 {
			 std::vector<int> interrupt;

			 near_red=0;
			 if(red_number!=0 && state==0){
		 printf("red_number %d\n",red_number);
				 for(int i=0; i<red_number-1;i++){
				 	if(red_distance[i]<red_distance[i+1]){near_red=i;}
			 	else if(red_distance[i]==red_distance[i+1]){near_red=i;}
			 	else {near_red=i+1;}}

				printf("ball_X_r %f",ball_X_r[near_red]);
			 	if(abs(ball_X_r[near_red])<0.095 && red_distance[near_red]<0.4){
					slide_dist = int(1200*(0.1-abs(ball_X_r[near_red])));
			 		avoid_red(slide_dist);
					state=1;
			 		printf("avoid\n");
			 	}
				else{state=1;}
			}

			else{
				near_ball=0;
				state=0;
				for(int i = 0; i < ball_number-1; i++)
 					 {
 				 if(ball_distance[i]<ball_distance[i+1]){near_ball=i;}
 				 else if(ball_distance[i]==ball_distance[i+1]){near_ball=i;}
 				 else {near_ball=i+1;}}
				printf("ball_number %d\n",ball_number);
				printf("near %d\n",near_ball);
			//	printf("near_ball %d\n",near_ball);
			//	printf("how many balls? %d\n",ball_number[]);
			printf("ball distance of first ball %f\n",ball_distance[0]);
			printf("ball distance of second ball %f\n",ball_distance[1]);
			if(abs(ball_X[near_ball])>0.03){
				if(ball_X[near_ball]  > 0){data[0]=15;data[1]=15;data[2]=15;data[3]=15;
					printf("turn left\n");
					printf("nearest_ball_distance %f\n",ball_distance[near_ball]);
		//			printf(" ball_x%f\n",ball_X[near_ball]);
				}
				else{data[0]=-15;data[1]=-15;data[2]=-15;data[3]=-15;
						printf("turn right\n");
						printf("nearest_ball_distance %f\n",ball_distance[near_ball]);
			//			printf(" ball_x%f\n",ball_X[near_ball]);
					}

					}

				else{
						if(ball_distance[near_ball]>0.3)
							{

								data[0]=-40;data[1]=40;data	[2]=40;data[3]=-40;
							printf("go straight\n");
							printf("nearest_ball_distance %f\n",ball_distance[near_ball]);

							}
						else{
								 for (int i=0;i<200;i++)
								 {
									 data[0]=-30;data[1]=30;data[2]=30;data[3]=-30;
									 write(c_socket, data, sizeof(data));
									 ros::Duration(0.025).sleep();
									 printf("let's go\n");
									 printf("collector motor %f\n",data[4]);
								 }
								 ros::Duration(0.025).sleep();
								 for (int i=0;i<100;i++)
								 {
									 data[0]=-20;data[1]=20;data[2]=20;data[3]=-20;data[4]=45+90*(ball_get+1);
									 write(c_socket, data, sizeof(data));
									 ros::Duration(0.025).sleep();
									 printf("pick\n");
									 printf("collector motor %f\n",data[4]);
								 }
								 data[0]=0;data[1]=0;data[2]=0;data[3]=0;
								 ball_get++;
							}
						}
					}
				}

	//	printf("distance:%f\n",ball_distance[near_ball]);

		//자율 주행 알고리즘에 입력된 제어데이터(xbox 컨트롤러 데이터)를 myRIO에 송신(tcp/ip 통신)

//		printf("data0:%f\n",data[0]);
//		printf("data1:%f\n",data[1]);
//		printf("data2:%f\n",data[2]);
//		printf("data3:%f\n",data[3]);
	//printf("??\n");
//}
	write(c_socket, data, sizeof(data));
	ros::Duration(0.025).sleep();
	ros::spinOnce();
}
data[0]=0;data[1]=0;data[2]=-0;data[3]=-0;
while(green_number<2){
	// if(shortest<0.28){
	// 	avoid_wall();
	// }
//	else{

float wall_dist = 6.0;
int k=0;
for(int i = 0; i < 179; i++)
	{
		// if(lidar_distance[i]<lidar_distance[i+1]){lidar_obs=i;}
		// else if(lidar_distance[i]==lidar_distance[i+1]){lidar_obs=i;}
		// else {lidar_obs=i+1;}
		total_dist = lidar_distance[i] + lidar_distance[i+179];

		if (wall_dist>=total_dist){
			wall_dist = total_dist;
			angle = lidar_degree[i];
			if ( i > 90.5){
				//turn right
				ref2 = lidar_distance[i+89];
				ref1 = lidar_distance[i-89];
				k=i;

			}
			else if( i < 90.5){
				//turn left
				ref1 = lidar_distance[i+269];
				ref2 = lidar_distance[i+89];
				k=i;
			}
		}

			}
	//	printf("distance: %f\n",wall_dist);
	//		printf(" theta: %f\n",angle);
			//printf(" ind: %d\n",i);
for(int i=0;i<1;i++){
if(ref1==std::numeric_limits<float>::infinity()||ref2==std::numeric_limits<float>::infinity()||lidar_distance[0]==std::numeric_limits<float>::infinity()||lidar_distance[179]==std::numeric_limits<float>::infinity()){
write(c_socket, data, sizeof(data));
ros::Duration(0.025).sleep();
printf("here?\n");

break;
}
else{
printf(" ref1: %f\n", ref1);
printf(" ref2: %f\n", ref2);
printf("k %d\n",k);
printf("angle %f\n",angle);


//	while(green_number<2){
	if(ref1<ref2){
		if(  angle<-15){
			data[0]=20;data[1]=20;data[2]=20;data[3]=20;
			printf("turn left\n");
			printf("angle %f\n",angle);

		}
		else
		{
			data[0]=-20;
			data[1]=20;
			data[2]=20;
			data[3]=-20;
			printf("go_straight\n");
		}
	}
	else {
		if(-165<angle){
			data[0]=-20;data[1]=-20;data[2]=-20;data[3]=-20;
			printf("turn right\n");
			printf("angle %f\n",angle);

		}
		else
		{
		data[0]=-20;
		data[1]=20;
		data[2]=20;
		data[3]=-20;
		printf("go_straight\n");


	}


}
write(c_socket, data, sizeof(data));
ros::Duration(0.025).sleep();
printf("data[0] %f\n",data[0]);
printf("data[1] %f\n",data[1]);
printf("data[2] %f\n",data[2]);
printf("data[3] %f\n",data[3]);
}
}

ros::spinOnce();
//}
//data[0]=0;
//data[1]=0;
//data[2]=0;
//data[3]=0;
write(c_socket, data, sizeof(data));
ros::Duration(0.025).sleep();

}
	home_x=(ball_X_g[0]+ball_X_g[1])/2;
	if(abs(home_x)>0.02){
		if(home_x>0){
			data[0]=30;
			data[1]=30;
			data[2]=-30;
			data[3]=-30;
		}
		else{
			data[0]=-30;
			data[1]=-30;
			data[2]=30;
			data[3]=30;
		}}
	else{
			if(abs(ball_Y_g[0])>0.2){
				data[0]=-40;
				data[1]=40;
				data[2]=40;
				data[3]=-40;
			}
			else{
				for(int i=0;i<100;i++){
					data[0]=-40;
					data[1]=40;
					data[2]=40;
					data[3]=-40;
					write(c_socket, data, sizeof(data));
					ros::Duration(0.025).sleep();
				}
				for(int i=0;i<3;i++){
					for(int j=0;j<50;j++){
						data[0]=0;
						data[1]=0;
						data[2]=0;
						data[3]=0;
						data[4]=315-90*(i+1);
						write(c_socket, data, sizeof(data));
						ros::Duration(0.025).sleep();
						printf("finish!!!!\n");

					}
					if(i==2){
						ros::shutdown();
					}
				}
				}
}



write(c_socket, data, sizeof(data));
//k++;
ros::Duration(0.025).sleep();
ros::spinOnce();

		//	std::cout << "data : "<<data[0];


}

return 0;
}
