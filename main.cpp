#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include "core_msgs/ball_position.h"
#include "core_msgs/tf_result.h"

ros::Publisher pub;

/*struct ball{
	double x, y, z;
};//struct ball will have 3 variables x, y, and z.

struct ball ball1; //declare ball1
struct ball ball2; //declare ball2
*/
/*void callback(const visualization_msgs::Marker& msg){  //subscribe message from fake_ball_in_rviz node
	ball1.x = msg.points.at(0).x;
	ball1.y = msg.points.at(0).y;
	ball1.z = msg.points.at(0).z;ball_cx = msg.img_b_x
	ball_cy = msg.img_b_y
	ball_cz = m
	ball2.x = msg.points.at(1).x;
	ball2.y = msg.points.at(1).y;
	ball2.z = msg.points.at(1).z;
	//ROS_INFO("balls: %f %f %f %f %f %f", ball1.x, ball1.y, ball1.z, ball2.x, ball2.y, ball2.z);
}*/
std::vector<float>  b_ball_cx, b_ball_cy, b_ball_cz;
std::vector<float>    r_ball_cx, r_ball_cy, r_ball_cz;
std::vector<float>    g_ball_cx, g_ball_cy, g_ball_cz;



void callback(const core_msgs::ball_position::ConstPtr& msg)
{


	b_ball_cx.resize(msg->img_b_x.size());
	b_ball_cy.resize(msg->img_b_x.size());
	b_ball_cz.resize(msg->img_b_x.size());
	r_ball_cx.resize(msg->img_r_x.size()), r_ball_cy.resize(msg->img_r_x.size()), r_ball_cz.resize(msg->img_r_x.size());
	g_ball_cx.resize(msg->img_g_x.size()), g_ball_cy.resize(msg->img_g_x.size()), g_ball_cz.resize(msg->img_g_x.size());

//	ROS_INFO("%d %d %d", msg->img_r_x.size(), msg->img_b_x.size(), msg->img_g_x.size());
//	ROS_INFO("%f %f %f", msg->img_r_x[0], msg->img_r_y[0], msg->img_r_z[0]);


	for (size_t i=0;i<msg->img_b_x.size();i++){
		b_ball_cx[i] = msg->img_b_x[i];
		b_ball_cy[i] = msg->img_b_y[i];
		b_ball_cz[i] = msg->img_b_z[i];
//e(r_ball_cx.size());
	//	msgs.r_z.resize

	}

	for (size_t i=0;i<msg->img_r_x.size();i++){
		r_ball_cx[i] = msg->img_r_x[i];
		r_ball_cy[i] = msg->img_r_y[i];
		r_ball_cz[i] = msg->img_r_z[i];
	}

	for (size_t i=0;i<msg->img_g_x.size();i++){
		g_ball_cx[i] = msg->img_g_x[i];
		g_ball_cy[i] = msg->img_g_y[i];
		g_ball_cz[i] = msg->img_g_z[i];
	}
}

int main(int argc, char** argv){
        ros::init(argc, argv, "compute_position_in_other_frame");  //init ROS node
	ros::NodeHandle nh;
	ros::Rate r(10); // 10 hz
  ros::Subscriber sub = nh.subscribe<core_msgs::ball_position>("/position",1,callback);  //declare subscriber
	pub = nh.advertise<core_msgs::tf_result>("/ball_platform",100);



	double x, y, z, qx, qy, qz, qw;  //variables to save transformation variables. it is just for printing.
	tf::TransformListener listener;  //declare tf listener

  	while (ros::ok()){

		tf::StampedTransform transform; //declare transform
		core_msgs::tf_result msgs;
		    //about try&catch, read README.md file for detail
		    try{
			 listener.lookupTransform("/mobile_platform", "/camera_link", ros::Time(0), transform); //this command will receive transformation between "world frame" and "camera_link frame"
		    }
		    catch (tf::TransformException &ex) {
		      ROS_ERROR("%s",ex.what());
		      ros::Duration(1.0).sleep();
		      continue;
		    }


	    x = transform.getOrigin().x();
	    y = transform.getOrigin().y();
	    z = transform.getOrigin().z();

	    qx = transform.getRotation().x();
	    qy = transform.getRotation().y();
	    qz = transform.getRotation().z();
	    qw = transform.getRotation().w();

ros::spinOnce();

		ROS_INFO("transform: %f %f %f %f %f %f %f", x, y, z, qx, qy, qz, qw);  //print the tf information between "world" and "camera_link"
		msgs.b_x.resize(b_ball_cx.size());
		msgs.b_y.resize(b_ball_cx.size());
		msgs.b_z.resize(b_ball_cx.size());
		for (size_t i=0;i<b_ball_cx.size();i++){


		tf::Vector3 output;

  		tf::Vector3 input(b_ball_cx[i],b_ball_cy[i],b_ball_cz[i]); //declare tf::Vector3. this input will have values of position of ball1 before trnasformation


			output = transform*input; // apply transformation.

			msgs.b_x[i]=output[0];
			msgs.b_y[i]=output[1];
			msgs.b_z[i]=output[2];

		}
		msgs.r_x.resize(r_ball_cx.size());
		msgs.r_y.resize(r_ball_cx.size());
		msgs.r_z.resize(r_ball_cx.size());

		ROS_INFO("ball size: %d" , r_ball_cx.size());

		for (size_t i=0;i<r_ball_cx.size();i++){
			tf::Vector3 input(r_ball_cx[i],r_ball_cy[i],r_ball_cz[i]); //declare tf::Vector3. this input will have values of position of ball1 before trnasformation
			tf::Vector3 output;


			ROS_INFO("before T: %f %f %f", r_ball_cx[i], r_ball_cy[i], r_ball_cz[i]); 
			output = transform*input; // apply transformation.
			msgs.r_x[i]=output[0];
			msgs.r_y[i]=output[1];
			msgs.r_z[i]=output[2];

			ROS_INFO("after T: %f %f %f", output[0],output[1], output[2]);

			if (r_ball_cx[i]<0.01&&r_ball_cx[i]>-0.01)
			{
				printf("input %f\n",r_ball_cy[i]);
				printf("output %f\n",msgs.r_y[i]);
			}
		}
		msgs.g_x.resize(g_ball_cx.size());
		msgs.g_y.resize(g_ball_cx.size());
		msgs.g_z.resize(g_ball_cx.size());
		for (size_t i=0;i<g_ball_cx.size();i++){
			tf::Vector3 input(g_ball_cx[i],g_ball_cy[i],g_ball_cz[i]); //declare tf::Vector3. this input will have values of position of ball1 before trnasformation
			tf::Vector3 output;
			output = transform*input; // apply transformation.
			msgs.g_x[i]=output[0];
			msgs.g_y[i]=output[1];
			msgs.g_z[i]=output[2];

		}
msgs.size =msgs.g_x.size()+msgs.b_x.size()+msgs.r_x.size();
				pub.publish(msgs);

		r.sleep();
	}


  return 0;
}
