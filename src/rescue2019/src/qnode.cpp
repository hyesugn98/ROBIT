/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/rescue2019/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rescue2019 {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {}

void QNode::Img1_Callback(const sensor_msgs::ImageConstPtr &msg_img)
{
    if(img_qnode1 == NULL)
    {
        img_qnode1 = new cv::Mat(cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::RGB8)->image);
        if(img_qnode1 != NULL)
            Q_EMIT recvImg1();
    }
}

void QNode::Img2_Callback(const sensor_msgs::ImageConstPtr &msg_img)
{
    if(img_qnode2 == NULL)
    {
        img_qnode2 = new cv::Mat(cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::RGB8)->image);
        if(img_qnode2 != NULL)
            Q_EMIT recvImg2();
    }
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"rescue2019");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_sub1 = it.subscribe("/camera1/usb_cam1/image_raw",1,&QNode::Img1_Callback, this);
    image_sub2 = it.subscribe("/camera2/usb_cam2/image_raw",1,&QNode::Img2_Callback, this);
    
	// Add your ros communications here.
	start();
	return true;
}

void QNode::run() {
    ros::Rate loop_rate(100);
	while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
}
