#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <map>
#include <ctime>
#include <sstream>
#include <algorithm>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "math_helper.h"
#include "string_convertor.h"
#include "transformation2D.h"
#include "signature_visualization/pathData.h"

using namespace cv;
using namespace std;

vector<vector<Point> > vPtSignature;
int keyPress;
Point transPoint(280,100);//point for aligntment test, left click to set the translation point.
double scale=1;//middle click to set the scale.
double scaleIncremental=0.1;
double rotation=0;//rotation, defined in degrees. click
double rotationIncremental=1;
//bool forward=true;
//bool rotationMode=false;
bool published=false;
cv::Mat recvImg;

void CallBackFunc2(int event, int x, int y, int flags, void* userdata);

void printOutDebugging()
{
  cout<<"translation:"<<transPoint<<endl;
  cout<<"scale:"<<scale<<endl;
  cout<<"rotation:"<<rotation<<endl;
}

void resetTransformations()
{
  transPoint.x=280;
  transPoint.y=100;
  scale=1.0;
  rotation=0;
}

cv::Mat processOperateImg()
{
   ros::spinOnce();
   Mat tmp_img = recvImg.clone();
   size_t strokesNum=vPtSignature.size();
   if(strokesNum>0)
   {
     transformation2D tf2D;
     vector< vector<Point2d> > tfPoints=tf2D.doTransformation(transPoint, vPtSignature, scale, rotation);
     for(int m=0;m<tfPoints.size();m++)
       for(int n=0;n<tfPoints[m].size();n++)
         if(!published)
             cv::circle( tmp_img, tfPoints[m][n], 2.0, cv::Scalar( 0, 0, 255), 1, 2 );//mark the desired point
        else
             cv::circle( tmp_img, tfPoints[m][n], 2.0, cv::Scalar( 0, 255, 0), 1, 2 );//mark the desired point
   }
   //resetTransformations();
   return tmp_img;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
   try
    {
      recvImg=cv_bridge::toCvShare(msg, "bgr8")->image;//using for simulation display
      namedWindow("Alignment",CV_WINDOW_NORMAL);
      cv::Mat operateImg=processOperateImg();
      imshow("Alignment", operateImg);
      setMouseCallback("Alignment", CallBackFunc2, NULL);
      // cv::waitKey(30);
    }//end of try
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
 }

 void signature_data_callback(const std_msgs::String::ConstPtr& msg)
 {
     if(msg->data=="reset")
     {
        vPtSignature.clear();
        return ;
     }

     vector<string> strokeStrs=string_convertor::split(msg->data, ';');//get different strokes.
     size_t strokesNum=strokeStrs.size();
     if(strokesNum>0)
     {
         vPtSignature.clear();
         vector<vector<Point> >().swap(vPtSignature);
         for(int i=0;i<strokesNum;i++)
         {
           string thisStroke=strokeStrs[i];
           vector<double> points=string_convertor::fromString2Array(thisStroke);
           size_t pointNum=points.size()/2;
           vector<Point> strokePoints;
           for(int j=0;j<pointNum;j++)
               strokePoints.push_back(Point(points[2*j],points[2*j+1]));
           vPtSignature.push_back(strokePoints);
         }
         published=false;
         //resetTransformations();
     }
 }



//===========================MAIN FUNCTION START===========================

int main(int argc, char* argv[]){

  ros::init(argc, argv, "signature_vis");
  ros::NodeHandle nh;
  ros::Publisher pubTask = nh.advertise<signature_visualization::pathData>("/path_data", 1, true);//task will be only published once
  ros::Publisher pubTask2 = nh.advertise<std_msgs::String>("/chris/strokes", 1, true);//task will be only published once
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber subImage = it.subscribe("/camera/rgb/image_rect_color", 1, imageCallback);///camera/rgb/image_rect_color  /usb_cam/image_raw
  ros::Subscriber sub2 = nh.subscribe("/chris/targetPoints_2D", 1000, signature_data_callback);
  ros::spinOnce();
  cout << "Press any key to continue..." << endl;

    for(;;){
       ros::spinOnce();
        keyPress = cvWaitKey(1)&255;
      //  cout<<(keyPress)<<endl;
        if(27 == keyPress){//"Esc"
           break;
         }
         else if(105==keyPress){//i zoom in
            scale +=scaleIncremental;
            ROS_INFO_STREAM("zoom in");
         }
         else if(111==keyPress){//o zoom out
            scale -=scaleIncremental;
            ROS_INFO_STREAM("zoom out");
         }
         else if(114==keyPress){//r reset scale
           scale =1;
           ROS_INFO_STREAM("reset scale");
         }
         else if(122==keyPress){//z reset rotation
           rotation=0;
           ROS_INFO_STREAM("reset rotation");
         }
         else if(49==keyPress){//1 increase rotation
           rotation +=rotationIncremental;
           ROS_INFO_STREAM("rotation anti clockwise");
         }
         else if(50==keyPress){//2 decrease rotation
           rotation -=rotationIncremental;
           ROS_INFO_STREAM("rotation clockwise");
         }
        else if(32==keyPress){//"space" //send out the points

            size_t strokesNum=vPtSignature.size();
            if(strokesNum>0)
            {
              transformation2D tf2D;
              vector< vector<Point2d> > tfPoints=tf2D.doTransformation(transPoint, vPtSignature, scale, rotation);
              std_msgs::String msg;
              msg.data = string_convertor::constructPubStr(tfPoints);
              pubTask2.publish(msg);
              published=true;
              ROS_INFO_STREAM("strokes topic published!");

              signature_visualization::pathData myPathData;
              myPathData.x=0;
              myPathData.y=0;
              myPathData.a=0;
              myPathData.theta=0;
              myPathData.targetx=0;
              myPathData.targety=0;
              myPathData.savingFlag=1;
              //do transformation.
              myPathData.u_path=math_helper::getU_Path(tfPoints);
              myPathData.v_path=math_helper::getV_Path(tfPoints);
              pubTask.publish(myPathData);
              continue;
            }
            else
              ROS_INFO_STREAM("no strokes to publish.");
         }
    }
    //cv::namedWindow("view");
    //cv::startWindowThread();

      //declaration of function
  	//cv::destroyWindow("view");
    return 0;
}


void CallBackFunc2(int event, int x, int y, int flags, void* userdata)
{

  if  ( event == EVENT_LBUTTONDOWN )
  {
    transPoint.x = x;
    transPoint.y = y;
    ROS_INFO_STREAM("left click down");
    printOutDebugging();
  }
  else if  ( event == EVENT_LBUTTONUP )
  {
    ROS_INFO_STREAM("left click up");
    //printOutDebugging();
  }
  else if  ( event == EVENT_RBUTTONDOWN )
  {
      //rotation +=rotationIncremental;
      //ROS_INFO_STREAM("right click down");
  }
  else if  ( event == EVENT_RBUTTONUP )
  {

    // ROS_INFO_STREAM("right click off");
  }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
        //rotation -=rotationIncremental;
        // ROS_INFO_STREAM("middle click down");
     }
      else if ( event ==EVENT_MBUTTONUP)
      {
        //ROS_INFO_STREAM("middle click up");
      }
     else if ( event == EVENT_MOUSEMOVE )
     {
          //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

     }
}
