#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "human_detector/detectedobjectsMsg.h"


#include "std_msgs/String.h"

#include <iostream>
#include <stdio.h>
#include <ros/package.h>

using namespace std;
using namespace cv;
//static string subscribeTopicName = "camera/camera";
//static string subscribeTopicName = "usb_cam/image_raw";
static string humanPubTopicName = "human_detection_result";
const double nmsTresh = 0.65;

ros::Publisher humanPub;
string path = ros::package::getPath("human_detector");
//HOGDescriptor sleephog = *new HOGDescriptor(path+ "/xml/cvHOGClassifier.yaml");
//HOGDescriptor pdestrianhog = *new HOGDescriptor(path+ "/xml/cvHOGClassifier2.yaml");
HOGDescriptor pdestrianhog;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);

static void resizeToDetectSize(Mat &imageData, Mat &resizedImage, int maxD) {
    double scale =maxD / (double) (max(imageData.rows, imageData.cols));
    resize(imageData, resizedImage, Size(imageData.cols * scale, imageData.rows * scale), 0, 0, INTER_LINEAR);
}

/*static vector<Rect> non_max_suppression(vector<Rect> rects, int overlapThresh) {
    vector<vector<int> > boxes;
    for (int i = 0; i < rects.size(); i++) {
        vector<int> box;
        box.push_back(rects[i].x);
        box.push_back(rects[i].y);
        box.push_back(rects[i].x + rects[i].width);
        box.push_back(rects[i].y + rects[i].height);
        boxes.push_back(box);
    }
    vector<Rect> pick;
    if (boxes.size() == 0)return pick;
    while (boxes.size() > 0) {
        int maxy2idx = 0;
        vector<int> maxy2box = boxes[maxy2idx];

        for (int i = 1; i < boxes.size(); i++) {
            vector<int> box = boxes[i];
            if (box[3] > maxy2box[3]) {
                maxy2idx = i;
                maxy2box = box;
            }
        }

        boxes.erase(boxes.begin() + maxy2idx);

        pick.push_back(Rect(maxy2box[0], maxy2box[1], maxy2box[2]-maxy2box[0], maxy2box[3]-maxy2box[1]));

        for (int i = 0; i < boxes.size(); i++) {
            vector<int> box = boxes[i];

            int xx1 = max(maxy2box[0], box[0]);
            int yy1 = max(maxy2box[1], box[1]);
            int xx2 = min(maxy2box[2], box[2]);
            int yy2 = min(maxy2box[3], box[3]);

            int w = max(0, xx2 - xx1 + 1);
            int h = max(0, yy2 - yy1 + 1);

            double overlap = ((double) (w * h)) / ((box[0] - box[2]) * (box[1] - box[3]));

            if (overlap > overlapThresh)boxes.erase(boxes.begin() + i);
        }
    }
    return pick;
}*/

int main(int argc, char **argv)
{
  //ROS_INFO("main");
  ros::init(argc, argv, "image_listener");
  static string subscribeTopicName(argv[1]);
  pdestrianhog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
  ros::NodeHandle nh;

  //namedWindow("view");
  //startWindowThread();

  humanPub = nh.advertise<human_detector::detectedobjectsMsg>(humanPubTopicName, 1);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(subscribeTopicName, 1, imageCallback);
  //ROS_INFO("end_main");
  ros::spin();

  //destroyWindow("view");
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    
    if( !frame.empty() ){
	resizeToDetectSize(frame,frame, 400);
       Mat gray_frame;
       cvtColor( frame, gray_frame, CV_BGR2GRAY );
       //equalizeHist( gray_frame, gray_frame );
       //vector<Rect> objs1;
       vector<Rect> objs2;
       //sleephog.detectMultiScale(gray_frame, objs1, 0.995770812034607, Size(8, 8), Size(8, 8), 1.05, 2); 
       //pdestrianhog.detectMultiScale(gray_frame, objs2, 0.817382812500000, Size(8, 8), Size(8, 8), 1.05, 2);
       pdestrianhog.detectMultiScale(gray_frame, objs2, 0.0, cv::Size(8, 8), cv::Size(0,0), 1.05, 4);  
	//objs1 = non_max_suppression(objs1, nmsTresh);
	//objs2 = non_max_suppression(objs2, nmsTresh);
       
       human_detector::detectedobjectsMsg osMsg;
	osMsg.found = 0;
	//int aria=0;
	//int target=0;
	//if(objs1.size() > 0 || objs2.size() > 0){
	if(objs2.size() > 0){
		osMsg.found = 1;
	       /*for(int i=0; i<objs1.size(); i++){
		  human_detector::objectMsg oMsg;
		  oMsg.x = objs1[i].x;
		  oMsg.y = objs1[i].y;
		  oMsg.width = objs1[i].width;
		  oMsg.height = objs1[i].height;	  
		  osMsg.objectsArray.push_back(oMsg);
		  
		  int oaria = objs1[i].width * objs1[i].height;
		  if(oaria > aria){
		  	aria = oaria;
		  	target = objs1[i].x + (objs1[i].width/2);
		  }
		  rectangle(frame, objs1[i].tl(), objs1[i].br(), Scalar(64, 255, 64),3); 
	       }*/
	}
	/*if(objs2.size() > 0){
		osMsg.found = true;
	       for(int i=0; i<objs2.size(); i++){
		  human_detector::objectMsg oMsg;
		  oMsg.x = objs2[i].x;
		  oMsg.y = objs2[i].y;
		  oMsg.width = objs2[i].width;
		  oMsg.height = objs2[i].height;	  
		  osMsg.objectsArray.push_back(oMsg);
		  
		  int oaria = objs2[i].width * objs2[i].height;
		  if(oaria > aria){
		  	aria = oaria;
		  	target = objs2[i].x + (objs2[i].width/2);
		  }
		  rectangle(frame, objs2[i].tl(), objs2[i].br(), Scalar(64, 255, 64),3); 
	       }
	}*/
	/*osMsg.lr = 0;
	if(osMsg.found == true){
		if(target < (frame.cols)/3){
			osMsg.lr = 1;
		}else if(target > (frame.cols*2)/3){
			osMsg.lr = 2;
		}
	}*/
	//ROS_INFO("%d" , target);
	//ROS_INFO("%d" , osMsg.lr);
       humanPub.publish(osMsg);
       //imshow("view", frame);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
