#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "static_tf_example");  //init ROS node
  ros::NodeHandle nh;
  ros::Rate r(10);  //10 hz
  static tf::TransformBroadcaster br;  //set transformbroadcaster (similar to publisher)
  static tf::TransformBroadcaster ls;
  while(ros::ok()){

	tf::Transform transform;  //define transform
  tf::Transform transform1;
	transform.setOrigin( tf::Vector3(0, -0.255, 0.2) );  //set the translation related variables
  transform1.setOrigin(tf::Vector3(0.1,-0.1,1.0));
  tf::Quaternion q;
  tf::Quaternion q1;
	q.setRPY(-1.57-0.7687, 0, 3.14);  //set the rotation related variables here
  //q.setRPY(0,0,0);
  q1.setRPY(0,1.57,0);
  transform.setRotation(q);
  transform1.setRotation(q1);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "mobile_platform", "camera_link"));
  ls.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "mobile_platform", "laser_frame"));  //publish tf. parent link is "world" and child link is "camera_link"
	r.sleep();
  }


  return 0;
}
