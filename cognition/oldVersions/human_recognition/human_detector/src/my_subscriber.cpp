#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "human_detector/detectedobjectsMsg.h"
#include "human_detector/objectMsg.h"


#include "std_msgs/String.h"

#include <iostream>
#include <stdio.h>
#include <ros/package.h>

using namespace std;
using namespace cv;
ros::Publisher face_pub;
ros::Publisher body_pub;
CascadeClassifier face_cascade;
HOGDescriptor hog;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char **argv)
{
  ROS_INFO("main");
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  namedWindow("view");
  startWindowThread();

  namedWindow("grayview");
  startWindowThread();

  namedWindow("histview");
  startWindowThread();

  std::string path = ros::package::getPath("human_detector");
  if( !face_cascade.load( path +"/xml/haarcascade_frontalface_alt2.xml" ) ){ ROS_INFO("--(!)Error loading\n");}
  hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
  face_pub = nh.advertise<human_detector::detectedobjectsMsg>("face_detection_result", 1);
  body_pub = nh.advertise<human_detector::detectedobjectsMsg>("body_detection_result", 1);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
  ROS_INFO("end_main");
  ros::spin();

  destroyWindow("view");
  destroyWindow("grayview");
  destroyWindow("histview");
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    
    if( !frame.empty() ){
       ROS_INFO("!frame empty");
       Mat gray_frame;
       cvtColor( frame, gray_frame, CV_BGR2GRAY );
       imshow("grayview", gray_frame);
       equalizeHist( gray_frame, gray_frame );
       imshow("histview", gray_frame);
       vector<Rect> faces;
       face_cascade.detectMultiScale( gray_frame, faces, 1.1, 3, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

       vector<Rect> bodies;
       hog.detectMultiScale(gray_frame, bodies, 0.0, cv::Size(8, 8), cv::Size(0,0), 1.05, 4); 
       
       human_detector::detectedobjectsMsg fsMsg;
       for(int i=0; i<faces.size(); i++){
	  human_detector::objectMsg fMsg;
	  fMsg.x = faces[i].x;
	  fMsg.y = faces[i].y;
	  fMsg.width = faces[i].width;
	  fMsg.height = faces[i].height;
          fsMsg.objectsArray.push_back(fMsg);
	  ROS_INFO("----FMSGX%d" , fMsg.x);
       }
       face_pub.publish(fsMsg);
       human_detector::detectedobjectsMsg bsMsg;
       for(int i=0; i<bodies.size(); i++){
	  human_detector::objectMsg bMsg;
	  bMsg.x = bodies[i].x;
	  bMsg.y = bodies[i].y;
	  bMsg.width = bodies[i].width;
	  bMsg.height = bodies[i].height;
          bsMsg.objectsArray.push_back(bMsg);
	  ROS_INFO("----BMSGX%d" , bMsg.x);
       }
       body_pub.publish(bsMsg);

       for(int i=0; i<faces.size(); i++){
          rectangle(frame, faces[i].tl(), faces[i].br(), Scalar(255),3);
       }
       for(int i=0; i<bodies.size(); i++){
          rectangle(frame, bodies[i].tl(), bodies[i].br(), Scalar(64, 255, 64),3); 
       }
       imshow("view", frame);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
