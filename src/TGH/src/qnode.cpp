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
#include "../include/TGH/qnode.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace TGH {


/*****************************************************************************
** Implementation
*****************************************************************************/

bool isRecved = false;

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

QNode::~QNode() {}


bool QNode::init() {
    ros::init(init_argc,init_argv,"TGH");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    com = it.subscribe("/usb_cam/image_raw",1,&QNode::imageCallback,this);
    //cam = n.advertise<TGH::TGH_msg>("TGH", 1);
    start();
    return true;
}

void QNode::imageCallback(const sensor_msgs::ImageConstPtr &msg_img)
{
    if(TGH_Mat == NULL && !isRecved)
    {
        TGH_Mat = new cv::Mat(cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8)->image);
        if(TGH_Mat != NULL)
        {
            Original = TGH_Mat->clone();
            Change_to_Binary(*TGH_Mat, Binary_img_free, save_HSV_free);
            Change_to_Binary(*TGH_Mat, Binary_img_black, save_HSV_black);
            Find_Cone(Original, save_HSV_yellow, blobs_yellow,0);
            Find_Cone(Original, save_HSV_green, blobs_green,1);
            updateHoughLine();

            Q_EMIT recvImg();
            isRecved = true;
        }
    }
}


void QNode::run() {
    ros::Rate loop_rate(300);
    while ( ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    Q_EMIT rosShutdown();
}

void QNode::Change_to_Binary(Mat &input_img, Mat &output_img, int value[])
{
    Mat mask = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));
    output_img = input_img.clone();
    medianBlur(output_img , output_img , 9);
    GaussianBlur(output_img , output_img , Size(15,15) , 2.0);
    cvtColor(output_img , output_img , CV_RGB2HSV);
    inRange(output_img, Scalar(value[3],value[4],value[5]),Scalar(value[0],value[1],value[2]), output_img);
    erode(output_img, output_img, mask, Point(-1, -1), 2);
    dilate(output_img, output_img, mask, Point(-1, -1), 2);
}

void QNode::updateHoughLine(void)
{
    Canny(Binary_img_black,Canny_img,50,200,3);
    int width = Canny_img.cols;
    int height = Canny_img.rows;
    points[0] = Point((width * (1 - trap_bottom_width)) / 2, height);
    points[1] = Point((width * (1 - trap_top_width)) / 2, height- height * trap_height);
    points[2] = Point(width - (width * (1- trap_top_width)) /2 , height - height * trap_height);
    points[3] = Point(width - (width * (1- trap_bottom_width)) /2 , height);
    Canny_img = region_of_interest(Canny_img, points);

    vector<Vec4i> black_line;
    HoughLinesP(Canny_img,black_line,1,CV_PI/180,50,30,50);

    for(size_t i=0; i<black_line.size();i++)
    {
        Vec4i L = black_line[i];
        line(Original, Point(L[0],L[1]),Point(L[2],L[3]), Red,3,8);

        if(L[0]>L[2])
        {
            if(L[0]>past_right) right=L[0];
            if(L[2]<past_left) left=L[2];
        }
        else
        {
            if(L[2]>past_right) right=L[2];
            if(L[0]<past_left) left=L[0];
        }
        past_right=right;
        past_left=left;
    }
}

Mat QNode::region_of_interest(Mat img_egdes, Point *points)
{
    Mat img_mask = Mat::zeros(img_egdes.rows, img_egdes.cols, CV_8UC1);
    const Point* ppt[1] = { points };
    int npt[] = { 4 };

    fillPoly(img_mask, ppt, npt, 1, Scalar(255,255,255), LINE_8);

    show=img_mask;

    Mat img_masked;
    bitwise_and(img_egdes, img_mask, img_masked);
    return img_masked;
}

void QNode::Find_Cone(Mat &input_img, int type[], vector<cv::Rect>&blobs, int _color)
{
    color=_color;
    past_left=10000;
    past_right=0;

    cv:: Scalar typecolor;
    Cone_img = input_img.clone();
    Change_to_Binary(Cone_img, Cone_img, type);
    labeling(Cone_img, 1300);
    do_labeling(blobs);

    if(type==save_HSV_yellow)
    {
        Binary_img_yellow=Cone_img;
        typecolor =Yellow;
    }
    if(type==save_HSV_green)
    {
        Binary_img_green=Cone_img;
        typecolor =Green;
    }

    if(blobs.size() > 0)
    {
        int max_x, min_x, max_num = 0, min_num = 0 , y_gap;
        max_x = blobs[0].x;
        min_x = blobs[0].x;
        for(int i = 0; i < blobs.size() ; i++)
        {
            if(blobs[i].x > max_x)
                max_num = i;
            if(blobs[i].x < min_x)
                min_num = i;
        }
        for(int i = 0 ; i < blobs.size() ; i++)
        {
            rectangle(input_img, blobs[i], typecolor, 2);
        }
        Where_is_Cone();
    }
    else
    {
        position[color]=3;
    }
}

void QNode::labeling(Mat &binary_img, int threshold)
{
    img_binary = binary_img.clone();
    pixel_threshold = threshold;
}

void QNode::do_labeling(vector<cv::Rect>&_blobs)
{
    pixel_num = 0;
    num_of_labels = connectedComponentsWithStats(img_binary,img_labels,stats,centroids,8,CV_32S);

    for(int i = 1 ; i < num_of_labels ; i++)
    {
        int area =  stats.at<int>(i,CC_STAT_AREA);
        int left = stats.at<int>(i,CC_STAT_LEFT);
        int top = stats.at<int>(i,CC_STAT_TOP);
        int width = stats.at<int>(i,CC_STAT_WIDTH);
        int height = stats.at<int>(i,CC_STAT_HEIGHT);

        cone_left[color]=left;
        cone_right[color]=left + width;

        for(int y = top ; y < top + height ; y++)
        {
            uchar *pointer_binary_img  = img_binary.ptr<uchar>(y);
            for(int x = left ; x < left + width ; x++)
            {
                if(pointer_binary_img[x] == 255)
                    pixel_num++;
            }
        }

        if(clear_state==1)
        {
            if(pixel_num > pixel_threshold)
            {
                _blobs.clear();
                _blobs.push_back(cv::Rect(left,top,width,height));
                clear_state=1;
            }
            else
            {
                _blobs.clear();
                clear_state=0;
            }
        }
        else
        {
            _blobs.clear();
            clear_state=1;
        }

    }
}


void QNode::Where_is_Cone(void)
{
    if(cone_right[color]<left)
    {
        position[color]=0; //the cone's position is left of the line
    }
    else if(cone_left[color]>right)
    {
        position[color]=1; //the cone's position is right of the line
    }
    else
    {
        position[color]=2; //the cone is on the line
    }
}

}  // namespace HR
