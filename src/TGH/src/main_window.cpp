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
#include "../include/TGH/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace TGH {

extern bool isRecved;


/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this);
    qnode.init();
    QObject::connect(&qnode, SIGNAL(recvImg()), this, SLOT(updateImg()));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    if(qnode.TGH_Mat != NULL) qnode.TGH_Mat = NULL;
}



void MainWindow::updateImg(void)
{
    updateBinaryImg_free();
    updateBinaryImg_black();
    updateBinaryImg_yellow();
    updateBinaryImg_green();
    position();


    QImage raw_image((const unsigned char*)(qnode.Original.data), qnode.Original.cols, qnode.Original.rows, QImage::Format_RGB888);
    ui.label_1->setPixmap(QPixmap::fromImage(raw_image.rgbSwapped()));
    delete qnode.TGH_Mat;
    if(qnode.TGH_Mat!=NULL) qnode.TGH_Mat=NULL;
    isRecved = false;
}



void MainWindow::updateBinaryImg_free(void)
{
    //QImage refined_img((const unsigned char*)(qnode.Binary_img_free.data), qnode.Binary_img_free.cols, qnode.Binary_img_free.rows,  QImage::Format_Indexed8);
   //ui.label_free->setPixmap(QPixmap::fromImage(refined_img.rgbSwapped()));
    QImage refined_img((const unsigned char*)(qnode.show.data), qnode.show.cols, qnode.show.rows,  QImage::Format_Indexed8);
    ui.label_free->setPixmap(QPixmap::fromImage(refined_img.rgbSwapped()));
}
void MainWindow::updateBinaryImg_black(void)
{
    QImage refined_img((const unsigned char*)(qnode.Binary_img_black.data), qnode.Binary_img_black.cols, qnode.Binary_img_black.rows,  QImage::Format_Indexed8);
    ui.label_2->setPixmap(QPixmap::fromImage(refined_img.rgbSwapped()));
}
void MainWindow::updateBinaryImg_yellow(void)
{
    QImage refined_img((const unsigned char*)(qnode.Binary_img_yellow.data), qnode.Binary_img_yellow.cols, qnode.Binary_img_yellow.rows,  QImage::Format_Indexed8);
    ui.label_3->setPixmap(QPixmap::fromImage(refined_img.rgbSwapped()));
}
void MainWindow::updateBinaryImg_green(void)
{
    QImage refined_img((const unsigned char*)(qnode.Binary_img_green.data), qnode.Binary_img_green.cols, qnode.Binary_img_green.rows,  QImage::Format_Indexed8);
    ui.label_4->setPixmap(QPixmap::fromImage(refined_img.rgbSwapped()));
}

MainWindow::~MainWindow() {}

}  // namespace TGH

void TGH::MainWindow::on_slider_U_H_valueChanged(int value)
{
   qnode.save_HSV_free[0] = value;
   ui.box_U_H->setText(QString::number(value));
}

void TGH::MainWindow::on_slider_U_S_valueChanged(int value)
{
    qnode.save_HSV_free[1] = value;
    ui.box_U_S->setText(QString::number(value));
}

void TGH::MainWindow::on_slider_U_V_valueChanged(int value)
{
    qnode.save_HSV_free[2] = value;
    ui.box_U_V->setText(QString::number(value));
}

void TGH::MainWindow::on_slider_L_H_valueChanged(int value)
{
    qnode.save_HSV_free[3] = value;
    ui.box_L_H->setText(QString::number(value));
}

void TGH::MainWindow::on_slider_L_S_valueChanged(int value)
{
    qnode.save_HSV_free[4] = value;
    ui.box_L_S->setText(QString::number(value));
}

void TGH::MainWindow::on_slider_L_V_valueChanged(int value)
{
    qnode.save_HSV_free[5] = value;
    ui.box_L_V->setText(QString::number(value));
}

void TGH::MainWindow::position(void)
{
    switch (qnode.position[0]) //yellow
    {
    case 0:
        ui.position_Y->setText("Yellow : Left");
        break;
    case 1:
        ui.position_Y->setText("Yellow : Right");
        break;
    case 2:
        ui.position_Y->setText("Yellow : Line");
        break;
    default:
        ui.position_Y->setText("-");
        break;
    }

    switch (qnode.position[1]) //green
    {
    case 0:
        ui.position_G->setText("Green : Left");
        break;
    case 1:
        ui.position_G->setText("Green : Right");
        break;
    case 2:
        ui.position_G->setText("Green : Line");
        break;
    default:
         ui.position_G->setText("-");
        break;
    }
}
