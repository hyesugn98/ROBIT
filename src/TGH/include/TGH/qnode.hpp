/**
 * @file /include/TGH/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef TGH_QNODE_HPP_
#define TGH_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <QThread>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>



#define Red cv::Scalar(0,0,255)
#define Green cv::Scalar(0,255,0)
#define Yellow cv::Scalar(0,255,255)

#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace TGH {
using namespace cv;
using namespace Qt;
using namespace std;

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:


    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    void run();
    int save_HSV_free[6]={0,};
    int save_HSV_black[6]={179,225,152, 0, 0, 0};
    int save_HSV_yellow[6]={117, 225, 225, 91, 155, 111};
    int save_HSV_green[6]={87, 225, 225, 64, 19, 111};

    cv::Mat *TGH_Mat, Original, Canny_img;
    cv::Mat Binary_img_free;
    cv::Mat Binary_img_black;
    cv::Mat Binary_img_yellow;
    cv::Mat Binary_img_green;
    void Change_to_Binary(Mat &input_img, Mat &output_img, int value[]);
    void updateHoughLine(void);
    ///////////////////////
    float trap_bottom_width = 0.45;  //사다리꼴의 하단 가장자리의 너비(백분율)
    float trap_top_width = 0.07;    //사다리꼴의 최상단 너비(백분율)
    float trap_height = 0.7;        //사다리꼴 높이
    Point points[4];
    Mat region_of_interest(Mat img_egdes, Point *points);
    ///////////////////////
    int pixel_threshold, num_of_labels, max_num, pixel_num, max_pixel;
    int clear_state=1;
    vector<cv::Rect> blobs_yellow, blobs_green;
    cv::Mat Cone_img, img_gray, img_color, img_binary, img_labels, stats, centroids;
    void labeling(Mat &binary_img, int threshold);
    void do_labeling(vector<cv::Rect>&blobs_);
    void Find_Cone(Mat &input_img, int type[], vector<cv::Rect>&blobs, int _color);
    ///////////////////////
    int past_left=10000, past_right=0; // left<->right
    int left, right;
    int cone_right[2], cone_left[2]; // 0 : yellow, 1 : green
    int color;
    int position[2];
    void Where_is_Cone(void);

    Mat show;

Q_SIGNALS:
    void recvImg();
    void rosShutdown();

private:
    int init_argc;
    char** init_argv;
    ros::Publisher cam;
    image_transport::Subscriber com;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg_img);
};

}  // namespace TGH

#endif /* TGH_QNODE_HPP_ */
