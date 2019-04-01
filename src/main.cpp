#define CMT_DISPLAY

#include "CMT.h"
#include "gui.h"

#include <string>
#include <ctime>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Polygon.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using cmt::CMT;
using cv::namedWindow;
using cv::Scalar;
using cv::VideoCapture;
using cv::waitKey;
using cv::Size;
using cv_bridge::CvImagePtr;
using cv_bridge::toCvCopy;
using std::min_element;
using std::max_element;
using ros::Publisher;
using ros::Subscriber;
using geometry_msgs::Polygon;
using geometry_msgs::Point32;
using sensor_msgs::ImageConstPtr;

const string WIN_NAME = "CMT Node";
const float scale = 0.5;

CMT* cmt_ptr = nullptr;
Publisher pub;

#ifdef CMT_DISPLAY
void display(Mat im, CMT & cmt){
    /* Visualize the output */
    //It is ok to draw on im itself, as CMT only uses the grayscale image
    for(size_t i = 0; i < cmt.points_active.size(); i++)
    {
        circle(im, cmt.points_active[i], 2, Scalar(255,0,0));
    }

    Point2f vertices[4];
    cmt.bb_rot.points(vertices);
    for (int i = 0; i < 4; i++)
    {
        line(im, vertices[i], vertices[(i+1)%4], Scalar(255,0,0));
    }

    imshow(WIN_NAME, im);
    waitKey(20);
}
#endif

void sendtoROS(CMT& cmt){
    Point2f vertices[4];
    cmt.bb_rot.points(vertices);
    
    Polygon pol;
    Point32 p;
    for (int i = 0; i < 4; i++)
    {
        p.x = vertices[i].x;
        p.y = vertices[i].y;
        pol.points.push_back(p);
    }
    pub.publish(pol);
}

void bluefoxCallback(const ImageConstPtr& im){
    if(cmt_ptr == nullptr){
        /* Initialize cmt */
        cmt_ptr = new CMT();

        /* Get the initial bounding box */
        CvImagePtr im0_ptr;
        Rect rect;
        im0_ptr = toCvCopy(im,"mono8");
        resize(im0_ptr->image,im0_ptr->image,Size(),scale,scale);
        rect = getRect(im0_ptr->image,WIN_NAME);
        ROS_INFO("Using bounding box (%d,%d,%d,%d)",rect.x,rect.y,rect.width,rect.height);

        // cmt_ptr->consensus.estimate_rotation = true;
        cmt_ptr->initialize(im0_ptr->image,rect);
#ifndef CMT_DISPLAY
        cv::destroyWindow(WIN_NAME);
#endif
    }else{
        clock_t begin,end;
        int time_elapsed;
        CvImagePtr im_ptr = toCvCopy(im,"mono8");
        resize(im_ptr->image,im_ptr->image,Size(),0.5,0.5);
        
        begin = clock();
        cmt_ptr->processFrame(im_ptr->image);
        end = clock();
        time_elapsed = (end-begin)*1.0/CLOCKS_PER_SEC*1000;
        ROS_INFO("Time: %dms, active features: %lu",time_elapsed,cmt_ptr->points_active.size());

#ifdef CMT_DISPLAY
        /* Display the image */
        display(im_ptr->image,*cmt_ptr);
#endif
        sendtoROS(*cmt_ptr);
    }   
}

int main(int argc, char *argv[]){
    namedWindow(WIN_NAME);
    
    /* Initialize ROS */
    ros::init(argc,argv,"cmt_node");
    ros::NodeHandle n("~");
    pub = n.advertise<Polygon>("cmt_output",10);
    Subscriber sub = n.subscribe("/camera/image_raw",10,bluefoxCallback);
    ros::spin();
}
