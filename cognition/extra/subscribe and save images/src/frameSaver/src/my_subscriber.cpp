#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"


#include "std_msgs/String.h"

#include <iostream>
#include <stdio.h>
#include <ros/package.h>

using namespace std;
using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);

int counter = 0;
int tresh = 5;
static string pobTop = "camera/rgb/image_raw";
string path = ros::package::getPath("frameSaver");

int main(int argc, char **argv)
{
  ROS_INFO("main");
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh; 

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(pobTop, 1, imageCallback);
  ROS_INFO("end_main");
  ros::spin();
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {    
    if(tresh==0){
	    Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	    counter++;
	    ostringstream ss;
	    ss << counter;
	    imwrite(path +"/images/imagee" +ss.str() +".png", frame);
	    printf("image %d\n", counter);
    }
    tresh++;
    tresh %= 5;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

