/**
 * @file /include/rescue2019/main_window.hpp
 *
 * @brief Qt based gui for rescue2019.
 *
 * @date November 2010
 **/
#ifndef rescue2019_MAIN_WINDOW_H
#define rescue2019_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include <QtNetwork/QUdpSocket>
#include <QPainter>
#include <QGraphicsScene>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "udpsocket.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace rescue2019 {

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

public Q_SLOTS:
    void Update_Img1();
    void Update_Img2();
    void Recieve_Data();

private:
    UdpSocket *mUdpsocket1, *mUdpsocket2, *mUdpsocket3 ;
	Ui::MainWindowDesign ui;
    QNode qnode;
    QByteArray buffer1, buffer2;
    QGraphicsScene *scene;
    QImage Img1, Img2;
};

}  // namespace rescue2019

#endif // rescue2019_MAIN_WINDOW_H
