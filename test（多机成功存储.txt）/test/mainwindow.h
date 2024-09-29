#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTcpServer>
#include <QTcpSocket>
#include <QList>
#include <QFile>
#include <QTextStream>
#include "../libs/qcustomplot.h"

enum MSG {NONE, TAKE_OFF, HOLD, LAND, FOLLOW};

//定义地面站控制消息结构体
typedef struct Message_Ground_Control_Basic
{
    uint8_t message_ID;
    uint8_t control_cmd;
    uint8_t sendto_ground;
}Ground_Control_Message;

//定义地面站接收消息结构体
typedef struct Message_Ground_Receive_Basic
{
    double uav_x;
    double uav_y;
    double uav_z;
    double uav_vx;
    double uav_vy;
    double uav_vz;
    double roll;
    double pitch;
    double yaw;
    double eso_x;
    double eso_y;
    double eso_z;
    double eso_vx;
    double eso_vy;
    double eso_vz;
    double eso_dx;
    double eso_dy;
    double eso_dz;
} Ground_Receive_Message;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_connectButton_clicked();
    void on_takeoffButton_clicked();
    void on_followButton_clicked();
    void on_landButton_clicked();
    void on_recvdataButton_clicked();
    void newConnection();
    void readData();

private:
    double start_time;
    Ui::MainWindow *ui;
    QList<QTcpServer*> tcpServers;
    QList<QTcpSocket*> clientSockets;
    QList<QFile*> dataFiles;
    QCustomPlot *customPlotX;
    QCustomPlot *customPlotY;
    QCustomPlot *customPlotZ;
};

#endif // MAINWINDOW_H
