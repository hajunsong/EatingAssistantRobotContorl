#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <iostream>
#include <QtDebug>
#include <QTimer>
#include <QCheckBox>
#include <stdio.h>
#include <stdlib.h>
#include <QDoubleSpinBox>

#include "dxlcontrol.h"
#include "algorithmthread.h"

using namespace std;
typedef unsigned int uint;

#define R2D 57.2957795130823
#define D2R 0.0174532925199433

class Body {
public:
    int32_t home_pos;
private:
};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void btnInitClicked();
    void btnStartClicked();
    void btnDeinitClicked();
    void btnSaveClicked();
    void btnLoggingClicked();
    void btnRunClicked();
    void btnReadyClicked();

    void cbStateChanged(bool state);

    void sbValueChanged(double arg);

    void mainTimerTimeout();

    void algorithmFinish();

private:
    Ui::MainWindow *ui;
    DxlControl dxlControl;
    Body body[6];
    bool cbState[6];

    QTimer *mainTimer;
    QVector<QCheckBox*> cbArray;
    QVector<int32_t> vecPosition, vecVelocity;
    QVector<QDoubleSpinBox*> sbArray;

    AlgorithmThread *algorithm;
    const int32_t q_offset[6] = {
        static_cast<int32_t>(633.4200000 / dxlControl.POSITION_UNIT),
        static_cast<int32_t>(177.0600000 / dxlControl.POSITION_UNIT),
        static_cast<int32_t>(294.0100000 / dxlControl.POSITION_UNIT),
        static_cast<int32_t>(655.9500000 / dxlControl.POSITION_UNIT),
        static_cast<int32_t>(428.1200000 / dxlControl.POSITION_UNIT),
        static_cast<int32_t>(357.9000000 / dxlControl.POSITION_UNIT)
    };
    int32_t goal_position[6];

    double *data;
    uint indx;
    uint col_size;
    uint data_size;
};

#endif // MAINWINDOW_H
