 #include "opencv2/objdetect/objdetect.hpp"
 #include "opencv2/highgui/highgui.hpp"
 #include "opencv2/imgproc/imgproc.hpp"


 #include <iostream>
 #include <stdio.h>

 using namespace std;
 using namespace cv;

 /** Function Headers */
 void detectAndDisplay( Mat frame );

 /** Global variables */
 String body_cascade_name = "haarcascade_fullbody.xml";
 CascadeClassifier body_cascade;
 string window_name = "Capture - body detection";
 RNG rng(12345);

 /** @function main */
 int main( int argc, const char** argv )
 {
   VideoCapture capture(-1);
   Mat frame;

   //-- 1. Load the cascades
   if( !body_cascade.load( body_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };

   //-- 2. Read the video stream
   if (capture.isOpened())
   {
     while( true )
     {
   capture.read(frame);

   //-- 3. Apply the classifier to the frame
       if( !frame.empty() )
       { detectAndDisplay( frame ); }
       else
       { printf(" --(!) No captured frame -- Break!"); break; }

       int c = waitKey(10);
       if( (char)c == 'c' ) { break; }
      }
   }
   return 0;
 }

/** @function detectAndDisplay */
void detectAndDisplay( Mat frame )
{
  std::vector<Rect> bodys;
  Mat frame_gray;

  cvtColor( frame, frame_gray, CV_BGR2GRAY );
  equalizeHist( frame_gray, frame_gray );

  //-- Detect bodys
  body_cascade.detectMultiScale( frame_gray, bodys, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

  for( size_t i = 0; i < bodys.size(); i++ )
  {
    Point center( bodys[i].x + bodys[i].width*0.5, bodys[i].y + bodys[i].height*0.5 );
    ellipse( frame, center, Size( bodys[i].width*0.5, bodys[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
  }
  //-- Show what you got
  imshow( window_name, frame );
 }
