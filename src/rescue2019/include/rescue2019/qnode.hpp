/**
 * @file /include/rescue2019/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef rescue2019_QNODE_HPP_
#define rescue2019_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <string>
#include <QThread>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core/core.hpp"

#endif
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rescue2019 {

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
    cv::Mat *img_qnode1, *img_qnode2;
Q_SIGNALS:
    void rosShutdown();
    void recvImg1();
    void recvImg2();

private:
	int init_argc;
	char** init_argv;
    image_transport::Subscriber image_sub1, image_sub2;
    void Img1_Callback(const sensor_msgs::ImageConstPtr& msg_img);
    void Img2_Callback(const sensor_msgs::ImageConstPtr& msg_img);
};

}  // namespace rescue2019

#endif /* rescue2019_QNODE_HPP_ */
