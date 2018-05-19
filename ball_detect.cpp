#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <opencv2/imgproc.hpp>//
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>

using namespace cv;
using namespace std;

Mat buffer(640,360,CV_8UC1);
ros::Publisher pub;
ros::Publisher pub_markers;
ros::Publisher pub_markers_r;
ros::Publisher pub_markers_g;

int low_h_b=90, low_s_b=200, low_v_b=100;//
int high_h_b=110, high_s_b=255, high_v_b=255;//
int low_h2_r=166, high_h2_r=180;//
int low_h_r=0, low_s_r=81, low_v_r=129;//
int high_h_r=1, high_s_r=255, high_v_r=255;//
int lowThreshold_r = 100;
int ratio_r = 3;
int kernel_size_r = 3;
int lowThreshold_b = 100;
int ratio_b = 3;
int kernel_size_b = 3;

int low_h_g=45, low_s_g=100, low_v_g=100;//
int high_h_g=70, high_s_g=255, high_v_g=255;//
int lowThreshold_g = 100;
int ratio_g = 3;
int kernel_size_g = 3;

string intToString(int n);
string floatToString(float f);
void morphOps(Mat &thresh);

// Declaration of functions that calculates the ball position from pixel position: Using the pixel positions from the image observed by the camera, this function calculates the position of the ball, which is extremely important in our course goal. The vector function stores series of elements with the same variable name, in float data type.
vector<float> pixel2point(Point center, int radius);
float fball_radius = 0.0734 ; // meter: The unit which is used in the initialization.

// Initialization of variable for camera calibration paramters: Like we did in our second class, we have to calibrate our main camera, and obtain the intrinsic and distortion parameters in order to undistort the images seen.
Mat distCoeffs;
float intrinsic_data[9] = {715.032256, 0, 347.884084, 0, 715.316998, 231.873067, 0, 0, 1};
float distortion_data[5] = {0.044202, -0.152627, 0.001762, 0.001029, 0};
int iMin_tracking_ball_size = 20; // This is the minimum tracking ball size, in pixels.

// Here, we start our main function.
void ball_detect()
{
    core_msgs::ball_position msg;  //create a message for ball positions
    //core_msgs::msgblue msgb;
    Mat frame, bgr_frame, hsv_frame, hsv_frame_red, hsv_frame_red1, hsv_frame_red2, hsv_frame_blue, hsv_frame_red_blur, hsv_frame_blue_blur, hsv_frame_red_canny, hsv_frame_blue_canny, result;
    Mat hsv_frame_green, hsv_frame_green_blur, hsv_frame_green_canny;
    Mat calibrated_frame;
    Mat intrinsic = Mat(3,3, CV_32FC1);
    Mat distCoeffs;
    intrinsic = Mat(3, 3, CV_32F, intrinsic_data);
    distCoeffs = Mat(1, 5, CV_32F, distortion_data);

    vector<Vec4i> hierarchy_r;
    vector<Vec4i> hierarchy_b;// Color is converted from BGR color space to HSV color space.
    vector<Vec4i> hierarchy_g;

    vector<vector<Point> > contours_r;
    vector<vector<Point> > contours_b;
    vector<vector<Point> > contours_g;


    //end of marker definitions
    // Here, we start the video capturing function, with the argument being the camera being used. 0 indicates the default camera, and 1 indicates the additional camera. Also, we make the 6 windows which we see at the results.
  //  VideoCapture cap(1);
    while((char)waitKey(1)!='q'){
    //    cap>>frame;


        if(buffer.size().width==320){
            cv::resize(buffer, frame, cv::Size(640, 360));
        }
        else{
            frame = buffer;
        }
        if(frame.empty())
            break;
        undistort(frame, calibrated_frame, intrinsic, distCoeffs);
        result = calibrated_frame.clone();


        medianBlur(calibrated_frame, calibrated_frame, 3);// Median blur function.

        cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);// Color is converted from BGR color space to HSV color space.


        // Detect the object based on RGB and HSV Range Values: In the following lines, the camera uses the BGR to HSV converted colors to detect the different colored objects.

        inRange(hsv_frame,Scalar(low_h_r,low_s_r,low_v_r),Scalar(high_h_r,high_s_r,high_v_r),hsv_frame_red1);

        inRange(hsv_frame,Scalar(low_h2_r,low_s_r,low_v_r),Scalar(high_h2_r,high_s_r,high_v_r),hsv_frame_red2);

        inRange(hsv_frame,Scalar(low_h_b,low_s_b,low_v_b),Scalar(high_h_b,high_s_b,high_v_b),hsv_frame_blue);

        inRange(hsv_frame,Scalar(low_h_g,low_s_g,low_v_g),Scalar(high_h_g,high_s_g,high_v_g),hsv_frame_green);

        addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0, hsv_frame_red);

        morphOps(hsv_frame_red);
        morphOps(hsv_frame_blue);
        morphOps(hsv_frame_green);

        // Gaussian blur function.minEnclosingCircle
        GaussianBlur(hsv_frame_red, hsv_frame_red_blur, cv::Size(9, 9), 2, 2);
        GaussianBlur(hsv_frame_blue, hsv_frame_blue_blur, cv::Size(9, 9), 2, 2);
        GaussianBlur(hsv_frame_green, hsv_frame_green_blur, cv::Size(9, 9), 2, 2);

        //Canny edge function.
        //cout << "work" <<endl;
        Canny(hsv_frame_red_blur, hsv_frame_red_canny, 100,300);
        Canny(hsv_frame_blue_blur, hsv_frame_blue_canny, 100,300);
        Canny(hsv_frame_green_blur, hsv_frame_green_canny, 100,300);

        //Find contour function.
        findContours(hsv_frame_red_canny, contours_r, hierarchy_r, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
        findContours(hsv_frame_blue_canny, contours_b, hierarchy_b, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
        findContours(hsv_frame_green_canny, contours_g, hierarchy_g, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));


        vector<vector<Point> > contours_r_poly( contours_r.size() );
        vector<vector<Point> > contours_b_poly( contours_b.size() );
        vector<Point2f>center_r( contours_r.size() );
        vector<Point2f>center_b( contours_b.size() );
        vector<float>radius_r( contours_r.size() );
        vector<float>radius_b( contours_b.size());
        vector<vector<Point> > contours_g_poly( contours_g.size() );
        vector<Point2f>center_g( contours_g.size() );
        vector<float>radius_g( contours_g.size());
        vector<int>check_r( contours_r.size() );
        vector<int>check_b( contours_b.size() );

        //cv::imshow("view", frame);  //show the image with a window
      //cv::waitKey(1rs_b.size() );

        for( size_t i = 0; i < contours_r.size(); i++ ){approxPolyDP( contours_r[i], contours_r_poly[i], 3, true );
            minEnclosingCircle( contours_r_poly[i], center_r[i], radius_r[i] );
        }

        for( size_t i = 0; i < contours_b.size(); i++ ){approxPolyDP( contours_b[i], contours_b_poly[i], 3, true );
            minEnclosingCircle( contours_b_poly[i], center_b[i], radius_b[i] );
        }

        for( size_t i = 0; i < contours_g.size(); i++ ){approxPolyDP( contours_g[i], contours_g_poly[i], 3, true );
            minEnclosingCircle( contours_g_poly[i], center_g[i], radius_g[i] );
        }

      	visualization_msgs::Marker ball_list;  //declare marker
      	ball_list.header.frame_id = "/camera_link";  //set the frame
      	ball_list.header.stamp = ros::Time::now();   //set the header. without it, the publisher may not publish.
      	ball_list.ns = "balls";   //name of markers
      	ball_list.action = visualization_msgs::Marker::ADD;
      	ball_list.pose.position.x=0; //the transformation between the frame and camera data, just set it (0,0,0,0,0,0) for (x,y,z,roll,pitch,yaw)
      	ball_list.pose.position.y=0;
      	ball_list.pose.position.z=0;
      	ball_list.pose.orientation.x=0;
      	ball_list.pose.orientation.y=0;
      	ball_list.pose.orientation.z=0;
      	ball_list.pose.orientation.w=1.0;
      	ball_list.id = 0; //set the marker id. if you use another markers, then make them use their own unique ids
      	ball_list.type = visualization_msgs::Marker::SPHERE_LIST;  //set the type of marker
      	ball_list.scale.x=0.10; //set the radius of marker   1.0 means 1.0m, 0.001 means 1mm
      	ball_list.scale.y=0.10;
      	ball_list.scale.z=0.10;

      	visualization_msgs::Marker ball_list_r;  //declare marker
      	ball_list_r.header.frame_id = "/camera_link";  //set the frame
      	ball_list_r.header.stamp = ros::Time::now();   //set the header. without it, the publisher may not publish.
      	ball_list_r.ns = "red_balls";   //name of markers
      	ball_list_r.action = visualization_msgs::Marker::ADD;
      	ball_list_r.pose.position.x=0; //the transformation between the frame and camera data, just set it (0,0,0,0,0,0) for (x,y,z,roll,pitch,yaw)
      	ball_list_r.pose.position.y=0;
      	ball_list_r.pose.position.z=0;
      	ball_list_r.pose.orientation.x=0;
      	ball_list_r.pose.orientation.y=0;
      	ball_list_r.pose.orientation.z=0;
      	ball_list_r.pose.orientation.w=1.0;
      	ball_list_r.id = 1; //set the marker id. if you use another markers, then make them use their own unique ids
      	ball_list_r.type = visualization_msgs::Marker::SPHERE_LIST;  //set the type of marker
      	ball_list_r.scale.x=0.10; //set the radius of marker   1.0 means 1.0m, 0.001 means 1mm
      	ball_list_r.scale.y=0.10;
      	ball_list_r.scale.z=0.10;

      	visualization_msgs::Marker ball_list_g;  //declare marker
      	ball_list_g.header.frame_id = "/camera_link";  //set the frame
      	ball_list_g.header.stamp = ros::Time::now();   //set the header. without it, the publisher may not publish.
      	ball_list_g.ns = "green_balls";   //name of markers
      	ball_list_g.action = visualization_msgs::Marker::ADD;
      	ball_list_g.pose.position.x=0; //the transformation between the frame and camera data, just set it (0,0,0,0,0,0) for (x,y,z,roll,pitch,yaw)
      	ball_list_g.pose.position.y=0;
      	ball_list_g.pose.position.z=0;
      	ball_list_g.pose.orientation.x=0;
      	ball_list_g.pose.orientation.y=0;
      	ball_list_g.pose.orientation.z=0;
      	ball_list_g.pose.orientation.w=1.0;
      	ball_list_g.id = 2; //set the marker id. if you use another markers, then make them use their own unique ids
      	ball_list_g.type = visualization_msgs::Marker::SPHERE_LIST;  //set the type of marker
      	ball_list_g.scale.x=0.10; //set the radius of marker   1.0 means 1.0m, 0.001 means 1mm
      	ball_list_g.scale.y=0.10;
      	ball_list_g.scale.z=0.10;
        int kr=0;
        int kb=0;
        int kg=0;
        for( size_t i = 0; i< contours_r.size(); i++ ){
          if(radius_r[i] > iMin_tracking_ball_size){
            kr++;
          }
        }

        for( size_t i = 0; i< contours_b.size(); i++ ){
          if(radius_b[i] > iMin_tracking_ball_size){
            kb++;
          }
        }

        for( size_t i = 0; i< contours_g.size(); i++ ){
          if(radius_g[i] > iMin_tracking_ball_size){
            kg++;
          }
        }


        msg.size =kb+kg+kr; //adjust the size 87:of message. (*the size of message is varying depending on how many circles are detected)

     int alphar=0;
     float x, y, r, l;

     for( int i = 0; i< radius_r.size(); i++ ){ //draw the circles
         for(int j = 0; j <radius_r.size(); j++ ){
             x = center_r[i].x - center_r[j].x;
             y = center_r[i].y - center_r[j].y;
             r = sqrt(pow(x,2.0) + pow(y,2.0));
             l = radius_r[i] +radius_r[j];
             if(r < l && radius_r[i] < radius_r[j] ){ // check i-th ball is in the j-th ball
                 check_r[i] = 1; //
                 break; // we aready know this circle is in the other circle, so we don't need to check anymore
             }
         }
     }

    kr=0;
    std::cout<<"# of contours: "<<contours_r.size()<<std::endl;
    for(int i=0;i<contours_r.size();i++){
	if(check_r[i]==0&&radius_r[i] > iMin_tracking_ball_size){
		std::cout<<i<<": "<<check_r[i]<<" //";
		kr++;
	}
    }
	std::cout<<std::endl;
	std::cout<<"the number of red balls: "<<kr<<std::endl;

        msg.img_r_x.resize(kr);  //adjust the size of array
        msg.img_r_y.resize(kr);  //adjust the size of array
        msg.img_r_z.resize(kr);//gblue’ is not a member of ‘core_msgs’
     //core_msgs::msgblue msgb;

//	std::cout<<"temp block===="<<std::endl;
      for( size_t i = 0; i< contours_r.size(); i++ ){
          if (radius_r[i] > iMin_tracking_ball_size && check_r[i] !=1){

            Scalar color = Scalar( 0, 0, 255);
                drawContours( hsv_frame_red_canny, contours_r_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
                vector<float> ball_position_r;
                ball_position_r = pixel2point(center_r[i], radius_r[i]);
                float isx = ball_position_r[0];
                float isy = ball_position_r[1];
                float isz = ball_position_r[2];

		std::cout<<ball_position_r[0]<<", "<<ball_position_r[1]<<", "<<ball_position_r[2]<<std::endl;


                /*string sx = floatToString(isx);
                string sy = floatToString(isy);
                string sz = floatToString(isz);*/
                //text = "Red ball:" + sx + "," + sy + "," + sz;
                /*putText(result, text, center_r[i],2,1,Scalar(0,255,0),2);
                circle( result, center_r[i], (int)radius_r[i], color, 2, 8, 0 );*/
                msg.img_r_x[alphar]=isx;  //input the x povtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);// Color is converted from BGR color space to HSV color space.
                msg.img_r_y[alphar]=isy;
                msg.img_r_z[alphar]=isz;


                geometry_msgs::Point p;
                p.x=isx;
                p.y=isy;
                p.z=isz;
                ball_list_r.points.push_back(p);
                std_msgs::ColorRGBA c;
                c.r = 1.0;  //set the color of the balls. You can set it respectively.
                c.g = 0.0;
                c.b = 0.0;
                c.a = 1.0;
                ball_list_r.colors.push_back(c);
                alphar++;
                if (isx<0.01&&isx>-0.01){
                //  printf("isx %f\n",isx);
                  printf("isy %f %f %f\n",isx, isy, isz);
                //  printf("isz %f\n",isz);
                }
              }
            }


       int alphab=0;
       for( int i = 0; i< radius_b.size(); i++ ){ //draw the circles
           for(int j = 0; j < radius_b.size(); j++ ){
               x = center_b[i].x - center_b[j].x;
               y = center_b[i].y - center_b[j].y;
               r = sqrt(pow(x,2.0) + pow(y,2.0));
               l = radius_b[i] +radius_b[j];
               if(r < l && radius_b[i] < radius_b[j] ){ // check i-th ball is in the j-th ball
                   check_b[i] = 1; //
                   break; // we aready know this circle is in the other circle, so we don't need to check anymore
               }
           }
       }
       kb=0;
       std::cout<<"# of contours: "<<contours_b.size()<<std::endl;
       for(int i=0;i<contours_b.size();i++){
   	if(check_b[i]==0&&radius_b[i] > iMin_tracking_ball_size){
   		std::cout<<i<<": "<<check_b[i]<<" //";
   		kb++;
   	}
       }
       std::cout<<std::endl;
     	std::cout<<"the number of blue balls: "<<kb<<std::endl;

       msg.img_b_x.resize(kb);  //adjust the size of array
        msg.img_b_y.resize(kb);  //adjust the size of array
        msg.img_b_z.resize(kb);
        printf("size %lu\n",msg.img_b_x.size());
      for( size_t i = 0; i< contours_b.size(); i++ ){
        if(radius_b[i] > iMin_tracking_ball_size && check_b[i]==0){
          Scalar color = Scalar( 255, 0, 0);
          drawContours( hsv_frame_blue_canny, contours_b_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
          vector<float> ball_position_b;// Color is converted from BGR color space to HSV color space.
          ball_position_b = pixel2point(center_b[i], radius_b[i]);
          float isx = ball_position_b[0];
          float isy = ball_position_b[1];
          float isz = ball_position_b[2];
          msg.img_b_x[alphab]=isx;  //input the x povtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);// Color is converted from BGR color space to HSV color space.
          msg.img_b_y[alphab]=isy;
          msg.img_b_z[alphab]=isz;
          //printf("isx %f\n",isx);
          //printf("isy %f\n",isy);
          printf("alphab %d\n",alphab);
          geometry_msgs::Point p;
          p.x=isx;
          p.y=isy;
          p.z=isz;
          ball_list.points.push_back(p);
          std_msgs::ColorRGBA c;
          c.r = 0.0;  //set the color of the balls. You can set it respectively.
          c.g = 0.0;
          c.b = 1.0;
          c.a = 1.0;
          ball_list.colors.push_back(c);

          alphab++;

            }
          }


  //      pub.publish(msg);

      //  msg.size =contours_b.size(); //adjust the size 87:of message. (*the size of message is varying depending on how many circles are detected)


          msg.img_g_x.resize(kg);  //adjust the size of array
          msg.img_g_y.resize(kg);  //adjust the size of array
          msg.img_g_z.resize(kg);
          int alphag=0;

        for( size_t i = 0; i< contours_g.size(); i++ ){
          if(radius_g[i] > iMin_tracking_ball_size){
            Scalar color = Scalar( 0, 255, 0);
            //drawContours( hsv_frame_green_canny, contours_g_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            vector<float> ball_position_g;// Color is converted from BGR color space to HSV color space.
            ball_position_g = pixel2point(center_g[i], radius_g[i]);
            float isx = ball_position_g[0];
            float isy =  ball_position_g[1];
            float isz = ball_position_g[2];
            msg.img_g_x[alphag]=isx;  //input the x povtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);// Color is converted from BGR color space to HSV color space.
            msg.img_g_y[alphag]=isy;
            msg.img_g_z[alphag]=isz;
            geometry_msgs::Point p;
            p.x=isx;
            p.y=isy;
            p.z=isz;
            ball_list_g.points.push_back(p);
            std_msgs::ColorRGBA c;
            c.r = 0.0;  //set the color of the balls. You can set it respectively.
            c.g = 1.0;
            c.b = 0.0;
            c.a = 1.0;
            ball_list_g.colors.push_back(c);
            alphag++;




              }
            }
pub.publish(msg);
pub_markers.publish(ball_list);  //publish a marker message
pub_markers_g.publish(ball_list_g);
pub_markers_r.publish(ball_list_r);
    break;
        }




}
void morphOps(Mat &thresh){//create structuring element that will be used to "dilate" and "erode" image.: These are morphological operations, they process input image based on shape and generate an output image. Erosion and dilation remove noise, isolate individual elements, join disparate elements, , and find intensity bumps or holes.
    //the element chosen here is a 3px by 3px rectangle
    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));
    erode(thresh,thresh,erodeElement);
    erode(thresh,thresh,erodeElement);
    dilate(thresh,thresh,dilateElement);
    dilate(thresh,thresh,dilateElement);
}

vector<float> pixel2point(Point center, int radius){vector<float> position;
    float x, y, u, v, Xc, Yc, Zc;
    x = center.x;//.x;// .at(0);
    y = center.y;//.y;//
    u = (x-intrinsic_data[2])/intrinsic_data[0];
    v = (y-intrinsic_data[5])/intrinsic_data[4];

    Zc = (intrinsic_data[0]*fball_radius)/(2*(float)radius) ;
    Xc = u*Zc ;
    Yc = v*Zc ;
    Xc = roundf(Xc * 1000) / 1000;
    Yc = roundf(Yc * 1000) / 1000;
    Zc = roundf(Zc * 1000) / 1000;

    position.push_back(Xc);
    position.push_back(Yc);
    position.push_back(Zc);

    return position;

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{


  if(msg->height==480&&buffer.size().width==320){  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
	std::cout<<"resized"<<std::endl;
	cv::resize(buffer,buffer,cv::Size(640,360));
}
   else{
	//do nothing!
   }

   try
   {
     buffer = cv_bridge::toCvShare(msg, "bgr8")->image;  //transfer the image data into buffer
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
   ball_detect(); //proceed ball detection
}

int main(int argc, char **argv)
{
  cout << "main" << endl;
   ros::init(argc, argv, "ball_track"); //init ros noddimageCallback
   ros::NodeHandle nh; //create node handler
   image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
   image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback); //create subscriber
   pub = nh.advertise<core_msgs::ball_position>("/position",1);
//   pubblue = nh.advertise<core_msgs::msgblue>("/position",100); //setting publisher
   pub_markers = nh.advertise<visualization_msgs::Marker>("/balls",1);
   pub_markers_r = nh.advertise<visualization_msgs::Marker>("/red_balls",1);
   pub_markers_g = nh.advertise<visualization_msgs::Marker>("/green_balls",1);


   ros::spin(); //spin.
   return 0;
}
