#include "mainwindow.h"
#include "ui_mainwindow.h"

void load_data(char* file_name, unsigned int row, unsigned int col, double *data) {
    FILE *fp_in;
    const int buffer = 1000000;
    char *ptr, basic[buffer];
    fp_in = fopen(file_name, "r");
    uint i = 0, j = 0;
    while (fgets(basic, buffer, fp_in) != nullptr)
    {
        j = 0;
        ptr = strtok(basic, "\t");
        while (ptr != nullptr) {
            data[i * col + j] = atof(ptr);
            ptr = strtok(nullptr, "\t");
            j++;
        }
        i++;
        if (i >= row) break;
    }
    fclose(fp_in);
}

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(ui->btnInit_m1, SIGNAL(clicked()), this, SLOT(btnInitClicked()));
    connect(ui->btnInit_m2, SIGNAL(clicked()), this, SLOT(btnInitClicked()));
    connect(ui->btnInit_m3, SIGNAL(clicked()), this, SLOT(btnInitClicked()));
    connect(ui->btnInit_m4, SIGNAL(clicked()), this, SLOT(btnInitClicked()));
    connect(ui->btnInit_m5, SIGNAL(clicked()), this, SLOT(btnInitClicked()));
    connect(ui->btnInit_m6, SIGNAL(clicked()), this, SLOT(btnInitClicked()));

    connect(ui->btnDeinit_m1, SIGNAL(clicked()), this, SLOT(btnDeinitClicked()));
    connect(ui->btnDeinit_m2, SIGNAL(clicked()), this, SLOT(btnDeinitClicked()));
    connect(ui->btnDeinit_m3, SIGNAL(clicked()), this, SLOT(btnDeinitClicked()));
    connect(ui->btnDeinit_m4, SIGNAL(clicked()), this, SLOT(btnDeinitClicked()));
    connect(ui->btnDeinit_m5, SIGNAL(clicked()), this, SLOT(btnDeinitClicked()));
    connect(ui->btnDeinit_m6, SIGNAL(clicked()), this, SLOT(btnDeinitClicked()));

    connect(ui->cbTorqueEnable_m1, SIGNAL(toggled(bool)), this, SLOT(cbStateChanged(bool)));
    connect(ui->cbTorqueEnable_m2, SIGNAL(toggled(bool)), this, SLOT(cbStateChanged(bool)));
    connect(ui->cbTorqueEnable_m3, SIGNAL(toggled(bool)), this, SLOT(cbStateChanged(bool)));
    connect(ui->cbTorqueEnable_m4, SIGNAL(toggled(bool)), this, SLOT(cbStateChanged(bool)));
    connect(ui->cbTorqueEnable_m5, SIGNAL(toggled(bool)), this, SLOT(cbStateChanged(bool)));
    connect(ui->cbTorqueEnable_m6, SIGNAL(toggled(bool)), this, SLOT(cbStateChanged(bool)));

    connect(ui->sbPos_m1, SIGNAL(valueChanged(double)), this, SLOT(sbValueChanged(double)));
    connect(ui->sbPos_m2, SIGNAL(valueChanged(double)), this, SLOT(sbValueChanged(double)));
    connect(ui->sbPos_m3, SIGNAL(valueChanged(double)), this, SLOT(sbValueChanged(double)));
    connect(ui->sbPos_m4, SIGNAL(valueChanged(double)), this, SLOT(sbValueChanged(double)));
    connect(ui->sbPos_m5, SIGNAL(valueChanged(double)), this, SLOT(sbValueChanged(double)));
    connect(ui->sbPos_m6, SIGNAL(valueChanged(double)), this, SLOT(sbValueChanged(double)));

    dxlControl.init();

    connect(ui->btnStart, SIGNAL(clicked()), this, SLOT(btnStartClicked()));

    mainTimer = new QTimer(this);
    mainTimer->setInterval(100);
    connect(mainTimer, SIGNAL(timeout()), this, SLOT(mainTimerTimeout()));

    ui->sbPos_m1->setRange(-999999, 999999);
    ui->sbPos_m2->setRange(-999999, 999999);
    ui->sbPos_m3->setRange(-999999, 999999);
    ui->sbPos_m4->setRange(-999999, 999999);
    ui->sbPos_m5->setRange(-999999, 999999);
    ui->sbPos_m6->setRange(-999999, 999999);

    ui->sbVel_m1->setRange(-999999, 999999);
    ui->sbVel_m2->setRange(-999999, 999999);
    ui->sbVel_m3->setRange(-999999, 999999);
    ui->sbVel_m4->setRange(-999999, 999999);
    ui->sbVel_m5->setRange(-999999, 999999);
    ui->sbVel_m6->setRange(-999999, 999999);

    cbArray.push_back(ui->cbTorqueEnable_m1);
    cbArray.push_back(ui->cbTorqueEnable_m2);
    cbArray.push_back(ui->cbTorqueEnable_m3);
    cbArray.push_back(ui->cbTorqueEnable_m4);
    cbArray.push_back(ui->cbTorqueEnable_m5);
    cbArray.push_back(ui->cbTorqueEnable_m6);

    sbArray.push_back(ui->sbPos_m1);
    sbArray.push_back(ui->sbPos_m2);
    sbArray.push_back(ui->sbPos_m3);
    sbArray.push_back(ui->sbPos_m4);
    sbArray.push_back(ui->sbPos_m5);
    sbArray.push_back(ui->sbPos_m6);

    connect(ui->btnSave, SIGNAL(clicked()), this, SLOT(btnSaveClicked()));
    connect(ui->btnLogging, SIGNAL(clicked()), this, SLOT(btnLoggingClicked()));
    connect(ui->btnRun, SIGNAL(clicked()), this, SLOT(btnRunClicked()));
    connect(ui->btnReady, SIGNAL(clicked()), this, SLOT(btnReadyClicked()));

    algorithm = new AlgorithmThread(this);
    connect(algorithm, SIGNAL(finish()), this, SLOT(algorithmFinish()));

    data_size = 8001;
    col_size = 13;

    data = new double[data_size * col_size];
    char file_name[256];
    sprintf_s(file_name, "../data/hj_inverse_kinematics_result.txt");
    load_data(file_name, data_size, col_size, data);
}

MainWindow::~MainWindow()
{
    for (uint8_t i = 0; i < 6; i++) {
        dxlControl.dxl_deinit(i, body[i].home_pos);
    }
    cbArray.clear();
    vecPosition.clear();
    vecVelocity.clear();

    delete[] data;

    delete ui;
}

void MainWindow::btnInitClicked() {
    uint8_t id = static_cast<uint8_t>(sender()->objectName().toStdString().back()) - 49;
    int dxlState = dxlControl.dxl_init(id);
    cbArray[id]->setChecked(dxlState);
    body[id].home_pos = dxlControl.getPresentPosition(id);
    sbArray[id]->setValue(body[id].home_pos*dxlControl.POSITION_UNIT);
}

void MainWindow::btnDeinitClicked() {
    uint8_t id = static_cast<uint8_t>(sender()->objectName().toStdString().back()) - 49;
    dxlControl.dxl_deinit(id, body[id].home_pos);
    cbArray[id]->setChecked(0);
}

void MainWindow::cbStateChanged(bool state) {
    uint8_t id = static_cast<uint8_t>(sender()->objectName().toStdString().back()) - 49;
    dxlControl.setTorqueEnable(state, id);
}

void MainWindow::sbValueChanged(double arg) {
    uint8_t id = static_cast<uint8_t>(sender()->objectName().toStdString().back()) - 49;
    dxlControl.setPosition(static_cast<int32_t>(arg/dxlControl.POSITION_UNIT), id);
}

void MainWindow::btnStartClicked() {
    for (uint i = 0; i < 6; i++) {
        goal_position[i] = body[i].home_pos;
    }
    mainTimer->start();
}

void MainWindow::btnRunClicked() {
    algorithm->start();
}

void MainWindow::btnReadyClicked() {
    for (uint i = 0; i < 6; i++) {
        //goal_position[i] = q_offset[i];
        goal_position[i] = q_offset[i] + static_cast<int32_t>(data[(i + 1)] * R2D / dxlControl.POSITION_UNIT);
    }

    dxlControl.setGroupSyncWriteGoalPosition(goal_position);
    indx = 0;
    mainTimer->stop();
}

void MainWindow::mainTimerTimeout() {
    int32_t presentPosition[6], presentVelocity[6];
    dxlControl.getGroupSyncReadPresentPosition(presentPosition);
    dxlControl.getGroupSyncReadPresentVelocity(presentVelocity);
    for (uint i = 0; i < 6; i++) {
        ui->tbMessage->append("ID : " + QString::number(i) + ", Position : " + QString::number(presentPosition[i] * dxlControl.POSITION_UNIT) + "[deg]");
    }

    ui->sbPos_m1->setValue(presentPosition[0] * dxlControl.POSITION_UNIT);
    ui->sbPos_m2->setValue(presentPosition[1] * dxlControl.POSITION_UNIT);
    ui->sbPos_m3->setValue(presentPosition[2] * dxlControl.POSITION_UNIT);
    ui->sbPos_m4->setValue(presentPosition[3] * dxlControl.POSITION_UNIT);
    ui->sbPos_m5->setValue(presentPosition[4] * dxlControl.POSITION_UNIT);
    ui->sbPos_m6->setValue(presentPosition[5] * dxlControl.POSITION_UNIT);

    ui->sbVel_m1->setValue(presentVelocity[0] * dxlControl.VELOCITY_UNIT);
    ui->sbVel_m2->setValue(presentVelocity[1] * dxlControl.VELOCITY_UNIT);
    ui->sbVel_m3->setValue(presentVelocity[2] * dxlControl.VELOCITY_UNIT);
    ui->sbVel_m4->setValue(presentVelocity[3] * dxlControl.VELOCITY_UNIT);
    ui->sbVel_m5->setValue(presentVelocity[4] * dxlControl.VELOCITY_UNIT);
    ui->sbVel_m6->setValue(presentVelocity[5] * dxlControl.VELOCITY_UNIT);

    for (uint i = 0; i < 6; i++) {
        vecPosition.push_back(presentPosition[i]);
        vecVelocity.push_back(presentVelocity[i]);
    }

    if (indx < data_size) {
        for (uint i = 0; i < 6; i++) {
            //goal_position[i] = q_offset[i];
            goal_position[i] = q_offset[i] + static_cast<int32_t>(data[indx*col_size + (i + 1)] * R2D / dxlControl.POSITION_UNIT);
        }
        indx += 15;
    }

    dxlControl.setGroupSyncWriteGoalPosition(goal_position);
}

void MainWindow::btnSaveClicked() {
    FILE *fp;
    fp = fopen("data/save_file.txt", "w+");
    fprintf(fp, "%.7f\t%.7f\t%.7f\t%.7f\t%.7f\t%.7f\n", ui->sbPos_m1->value(), ui->sbPos_m2->value(), ui->sbPos_m3->value(), ui->sbPos_m4->value(), ui->sbPos_m5->value(), ui->sbPos_m6->value());
    fclose(fp);
}

void MainWindow::btnLoggingClicked() {
    FILE *fp;
    fp = fopen("data/logging_file.txt", "w+");
    int size = vecPosition.length() / 6;
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < 6; j++) {
            fprintf(fp, "%.7f\t", vecPosition[i * 6 + j] * dxlControl.POSITION_UNIT);
        }
        for (int j = 0; j < 6; j++) {
            fprintf(fp, "%.7f\t", vecVelocity[i * 6 + j] * dxlControl.VELOCITY_UNIT);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
}

void MainWindow::algorithmFinish() {

}
