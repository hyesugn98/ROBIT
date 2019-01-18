/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/rescue2019/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
QHostAddress UdpSocket::ARRIBAL_IP = QHostAddress("192.168.0.8");
namespace rescue2019 {

using namespace Qt;
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    ui.setupUi(this);
    qnode.init();
    mUdpsocket1 = new UdpSocket(8888);
    mUdpsocket2 = new UdpSocket(8887);
    mUdpsocket3 = new UdpSocket(8886);
    connect(&qnode, SIGNAL(recvImg1()), this, SLOT(Update_Img1()));
    connect(&qnode, SIGNAL(recvImg2()), this, SLOT(Update_Img2()));
    connect(mUdpsocket1, SIGNAL(recvBuffer()), this, SLOT(Recieve_Data()));
    connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    ui.lineEdit->setText(mUdpsocket1->getIPAddresses().toString());
    ui.lineEdit_2->setText(mUdpsocket2->getIPAddresses().toString());
    ui.lineEdit_3->setText(mUdpsocket3->getIPAddresses().toString());
    if(qnode.img_qnode1 != NULL) qnode.img_qnode1 = NULL;
    if(qnode.img_qnode2 != NULL) qnode.img_qnode2 = NULL;
    ui.widget->acceptDrops();
}

void MainWindow::Update_Img1()
{
    Img1 = QImage((const unsigned char*)(qnode.img_qnode1->data), qnode.img_qnode1->cols, qnode.img_qnode1->rows,  QImage::Format_RGB888);
    ui.label->setPixmap(QPixmap::fromImage(Img1));
    QPixmap map = QPixmap::fromImage(Img1);
    QBuffer buff1(&buffer1);
    map.save(&buff1,"JPG");
    mUdpsocket1->sendData(&buffer1);
    buffer1.clear();
    qnode.img_qnode1 = NULL;
}

void MainWindow::Update_Img2()
{
    Img2 = QImage((const unsigned char*)(qnode.img_qnode2->data), qnode.img_qnode2->cols, qnode.img_qnode2->rows,  QImage::Format_RGB888);
    ui.label_2->setPixmap(QPixmap::fromImage(Img2));
    QPixmap map = QPixmap::fromImage(Img2);
    QBuffer buff2(&buffer2);
    map.save(&buff2,"JPG");
    mUdpsocket2->sendData(&buffer2);
    buffer2.clear();
    qnode.img_qnode2 = NULL;
}

void MainWindow::Recieve_Data()
{
//    QString data = QString::fromAscii(mUdpsocket1->getBuffer().data());
//    ui.lineEdit_4->setText(data);
}

MainWindow::~MainWindow() {}

}  // namespace rescue2019

