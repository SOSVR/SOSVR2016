#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "human_detector/facesMsg.h"
#include "human_detector/faceMsg.h"


#include "std_msgs/String.h"

#include <iostream>
#include <stdio.h>
#include <ros/package.h>

#include <fstream>
#include <ios>

using namespace std;
using namespace cv;
ros::Publisher human_pub;
HOGDescriptor body_cascade;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);

static void loadDescriptorVector(vector<float>& descriptorVector, string fileName) {
    string separator = " ";
    fstream File;
    File.open(fileName.c_str(), ios::in);
    if (File.good() && File.is_open()) {

	for(string line; getline(File, line); )
	{
	    istringstream in(line);
	    string buf;
    	    while (in >> buf){		
       		descriptorVector.push_back(atof(buf.c_str()));
	    }
	}

    }
}

int main(int argc, char **argv)
{
  ROS_INFO("main");
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  std::string path = ros::package::getPath("human_detector");
  if( !body_cascade.load( path +"/xml/cvHOGClassifier.yaml" ) ){ ROS_INFO("--(!)Error loading\n");}
  vector<float> descriptorVector;
  loadDescriptorVector(descriptorVector, path +"/xml/descriptorvector.dat");
  body_cascade.setSVMDetector(descriptorVector);
  human_pub = nh.advertise<human_detector::facesMsg>("human_detection_result", 1);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
  ROS_INFO("end_main");
  ros::spin();
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    
    if( !frame.empty() ){
       cvtColor( frame, frame, CV_BGR2GRAY );
       vector<Rect> faces;
       body_cascade.detectMultiScale( frame, faces, -0.649049, Size(8, 8), Size(8, 8) );
       
       human_detector::facesMsg fsMsg;
       for(int i=0; i<faces.size(); i++){
	  human_detector::faceMsg fMsg;
	  fMsg.x = faces[i].x;
	  fMsg.y = faces[i].y;
	  fMsg.width = faces[i].width;
	  fMsg.height = faces[i].height;
          fsMsg.facesArray.push_back(fMsg);
	  ROS_INFO("----FMSGX%d" , fMsg.x);
       }
       human_pub.publish(fsMsg);       
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

