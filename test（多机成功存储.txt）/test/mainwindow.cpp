#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QVBoxLayout>

Ground_Control_Message control_message;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      customPlotX(new QCustomPlot(this)),
      customPlotY(new QCustomPlot(this)),
      customPlotZ(new QCustomPlot(this))
{
    ui->setupUi(this);

    QVBoxLayout *layoutX = new QVBoxLayout;
    layoutX->addWidget(customPlotX);
    ui->plotWidgetX->setLayout(layoutX);

    QVBoxLayout *layoutY = new QVBoxLayout;
    layoutY->addWidget(customPlotY);
    ui->plotWidgetY->setLayout(layoutY);

    QVBoxLayout *layoutZ = new QVBoxLayout;
    layoutZ->addWidget(customPlotZ);
    ui->plotWidgetZ->setLayout(layoutZ);

    // 初始化QCustomPlot对象
    customPlotX->addGraph();
    customPlotX->addGraph();
    customPlotX->addGraph();

    customPlotY->addGraph();
    customPlotY->addGraph();
    customPlotY->addGraph();

    customPlotZ->addGraph();
    customPlotZ->addGraph();
    customPlotZ->addGraph();

    // 设置X轴标签
    customPlotX->xAxis->setLabel("Time");
    customPlotY->xAxis->setLabel("Time");
    customPlotZ->xAxis->setLabel("Time");

    // 设置Y轴标签（根据需要可以分别设置每个图形对象的Y轴标签）
    customPlotX->yAxis->setLabel("Position X");
    customPlotY->yAxis->setLabel("Position Y");
    customPlotZ->yAxis->setLabel("Position Z");

    // 设置不同的颜色
    customPlotX->graph(0)->setPen(QPen(Qt::red));
    customPlotX->graph(1)->setPen(QPen(Qt::green));
    customPlotX->graph(2)->setPen(QPen(Qt::blue));

    customPlotY->graph(0)->setPen(QPen(Qt::red));
    customPlotY->graph(1)->setPen(QPen(Qt::green));
    customPlotY->graph(2)->setPen(QPen(Qt::blue));

    customPlotZ->graph(0)->setPen(QPen(Qt::red));
    customPlotZ->graph(1)->setPen(QPen(Qt::green));
    customPlotZ->graph(2)->setPen(QPen(Qt::blue));

    // Initialize data files for each UAV
    dataFiles.append(new QFile("uav1_data.txt"));
    dataFiles.append(new QFile("uav2_data.txt"));
    dataFiles.append(new QFile("uav3_data.txt"));

    for (QFile *file : dataFiles) {
        if (!file->open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
            ui->logTextEdit->append("无法打开数据文件！");
        }
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_connectButton_clicked()
{
    for (int port : {8888, 8887, 8886}) {
        QTcpServer *tcpServer = new QTcpServer(this);
        if (tcpServer->listen(QHostAddress::Any, port)) {
            connect(tcpServer, &QTcpServer::newConnection, this, &MainWindow::newConnection);
            tcpServers.append(tcpServer);
            ui->logTextEdit->append("Listening on port " + QString::number(port));
        }
        else {
            ui->logTextEdit->append("Unable to start server on port " + QString::number(port));
            delete tcpServer;
        }
    }
}

void MainWindow::on_recvdataButton_clicked()
{
     for (QTcpSocket *socket : qAsConst(clientSockets)) {
         connect(socket, &QTcpSocket::readyRead, this, &MainWindow::readData);
     }
     ui->logTextEdit->append("开始接收所有无人机数据...");
}

void MainWindow::newConnection()
{
    for (QTcpServer *server : tcpServers) {
        while (server->hasPendingConnections()) {
            QTcpSocket *clientSocket = server->nextPendingConnection();
            if (clientSocket) {
                clientSockets.append(clientSocket);
                ui->logTextEdit->append("无人机连接成功！");
            }
        }
    }
}


void MainWindow::readData()
{
    const int maxDataChunksPerSocket = 10;
    for (QTcpSocket *socket : qAsConst(clientSockets))
    {
        // 检查连接状态
        if (socket->state() != QAbstractSocket::ConnectedState)
        {
            ui->logTextEdit->append("客户端断开连接！");
            clientSockets.removeAll(socket);
            socket->deleteLater();
            continue;
        }
        int dataChunksProcessed = 0;
        qint64 messageSize = static_cast<qint64>(sizeof(Ground_Receive_Message));
        while (socket->bytesAvailable() >= messageSize && dataChunksProcessed < maxDataChunksPerSocket)
        {
            Ground_Receive_Message recv_uav_message;
            qint64 bytesRead = socket->read(reinterpret_cast<char*>(&recv_uav_message), sizeof(recv_uav_message));
            if (bytesRead < messageSize){
                ui->logTextEdit->append("数据读取失败！");
                continue;
            }

            double time = QDateTime::currentDateTime().toMSecsSinceEpoch() / 1000.0 - start_time;  // 当前时间戳（秒）
            double posX = recv_uav_message.uav_x;
            double posY = recv_uav_message.uav_y;
            double posZ = recv_uav_message.uav_z;

            int socketIndex = clientSockets.indexOf(socket);
            if (socketIndex >= 0 && socketIndex < 3) {
                customPlotX->graph(socketIndex)->addData(time, posX);
                customPlotY->graph(socketIndex)->addData(time, posY);
                customPlotZ->graph(socketIndex)->addData(time, posZ);

                customPlotX->graph(socketIndex)->rescaleAxes(true);
                customPlotY->graph(socketIndex)->rescaleAxes(true);
                customPlotZ->graph(socketIndex)->rescaleAxes(true);

                customPlotX->replot();
                customPlotY->replot();
                customPlotZ->replot();

                QTextStream out(dataFiles[socketIndex]);
                out << " x_pos: " << recv_uav_message.uav_x << " y_pos: "  << recv_uav_message.uav_y << " z_pos: "  << recv_uav_message.uav_z
                    << " x_vel: " << recv_uav_message.uav_vx << " y_vel: " << recv_uav_message.uav_vy << " z_vel: " << recv_uav_message.uav_vz
                    << " roll: " << recv_uav_message.roll << " pitch: " << recv_uav_message.pitch << " yaw: " << recv_uav_message.yaw
                    << " eso_x: "<< recv_uav_message.eso_x << " eso_y: " << recv_uav_message.eso_y << " eso_z: "<< recv_uav_message.eso_z
                    << " eso_vx: " << recv_uav_message.eso_vx << " eso_vy: " << recv_uav_message.eso_vy << " eso_vz: " << recv_uav_message.eso_vz
                    << " eso_dx: " << recv_uav_message.eso_dx << " eso_dy: " << recv_uav_message.eso_dy << " eso_dz: "<< recv_uav_message.eso_dz << "\n";
            }
            dataChunksProcessed++;
        }
    }
}


void MainWindow::on_takeoffButton_clicked()
{
    start_time = QDateTime::currentDateTime().toMSecsSinceEpoch() / 1000.0;
    control_message.message_ID = 1;
    control_message.control_cmd = TAKE_OFF;
    control_message.sendto_ground = 1;
    for (QTcpSocket *socket : qAsConst(clientSockets)){
        socket->write(reinterpret_cast<const char*>(&control_message), sizeof(control_message));
    }
}

void MainWindow::on_followButton_clicked()
{
    control_message.message_ID = 4;
    control_message.control_cmd = FOLLOW;
    control_message.sendto_ground = 1;
    for (QTcpSocket *socket : qAsConst(clientSockets)){
        socket->write(reinterpret_cast<const char*>(&control_message), sizeof(control_message));
    }
}

void MainWindow::on_landButton_clicked()
{
    control_message.message_ID = 3;
    control_message.control_cmd = LAND;
    for (QTcpSocket *socket : qAsConst(clientSockets)){
        socket->write(reinterpret_cast<const char*>(&control_message), sizeof(control_message));
    }
}

