#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

volatile sig_atomic_t flag = 0;

void my_function(int sig) { // can be called asynchronously
    flag = 1; // set flag
}

int main(int argc, char **argv) {
    signal(SIGINT, my_function);
    VideoCapture cap(0); //capture the video from webcam

    if (!cap.isOpened())  // if not success, exit program
    {
        cout << "Cannot open the web cam" << endl;
        return -1;
    }

    int iLowH = 175;
    int iHighH = 179;

    int iLowS = 151;
    int iHighS = 250;

    int iLowV = 130;
    int iHighV = 255;

    //Create trackbars in "Control" window
    /*createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    createTrackbar("HighH", "Control", &iHighH, 179);

    createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    createTrackbar("HighS", "Control", &iHighS, 255);

    createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
    createTrackbar("HighV", "Control", &iHighV, 255);

    int iLastX = -1;
    int iLastY = -1;*/

    //Capture a temporary image from the camera
    Mat imgTmp;
    cap.read(imgTmp);

    //Create a black image with the size as the camera output


    while (true) {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video



        if (!bSuccess) //if not success, break loop
        {
            cout << "Cannot read a frame from video stream" << endl;
            break;
        }

        Mat imgHSV;

        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        Mat imgThresholded;

        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV),
                imgThresholded); //Threshold the image

        //morphological opening (removes small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

        //morphological closing (removes small holes from the foreground)
        dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

        GaussianBlur(imgThresholded, imgThresholded, Size(5, 5), 0);
        threshold(imgThresholded, imgThresholded, 0, 255, THRESH_BINARY+THRESH_OTSU);

        //Calculate the moments of the thresholded image
        Moments oMoments = moments(imgThresholded);

        double dM01 = oMoments.m01;
        double dM10 = oMoments.m10;
        double dArea = oMoments.m00;

        // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
        if (dArea > 10000) {
            //calculate the position of the ball
            int posX = dM10 / dArea;
            int posY = dM01 / dArea;

            if (posX >= 0 && posY >= 0) {
                //Draw a red line from the previous point to the current point
                rectangle(imgOriginal, Rect(posX-10, posY-10, 20, 20), Scalar(64, 255, 64),5);
            }
        }

        imshow("Thresholded Image", imgThresholded); //show the thresholded image

        imshow("Original", imgOriginal); //show the original image
        waitKey(30);
        if (flag) { // my action when signal set it 1
            cap.release();
            return 0;
        }
    }

    return 0;
}