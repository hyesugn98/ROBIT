/**
 * @file /include/TGH/main_window.hpp
 *
 * @brief Qt based gui for HR.
 *
 * @date November 2010
 **/
#ifndef TGH_MAIN_WINDOW_H
#define TGH_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace TGH {
using namespace cv;
using namespace Qt;
using namespace std;
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    void updateBinaryImg_free(void);
    void updateBinaryImg_black(void);
    void updateBinaryImg_yellow(void);
    void updateBinaryImg_green(void);
    void updateBinaryImg_edge(void);
    void position(void);

    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

public Q_SLOTS:
    void updateImg(void);

    void on_slider_U_H_valueChanged(int value);

    void on_slider_U_V_valueChanged(int value);

    void on_slider_U_S_valueChanged(int value);

    void on_slider_L_H_valueChanged(int value);

    void on_slider_L_S_valueChanged(int value);

    void on_slider_L_V_valueChanged(int value);

private:
    Ui::MainWindowDesign ui;
    QNode qnode;

};

}  // namespace HR

#endif // HR_MAIN_WINDOW_H
