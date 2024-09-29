/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QPushButton *connectButton;
    QPushButton *takeoffButton;
    QPushButton *followButton;
    QPushButton *landButton;
    QTextEdit *logTextEdit;
    QWidget *plotWidgetY;
    QWidget *plotWidgetZ;
    QWidget *plotWidgetX;
    QPushButton *recvdataButton;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(806, 695);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        connectButton = new QPushButton(centralwidget);
        connectButton->setObjectName(QString::fromUtf8("connectButton"));
        connectButton->setGeometry(QRect(20, 20, 111, 41));
        takeoffButton = new QPushButton(centralwidget);
        takeoffButton->setObjectName(QString::fromUtf8("takeoffButton"));
        takeoffButton->setGeometry(QRect(280, 20, 111, 41));
        followButton = new QPushButton(centralwidget);
        followButton->setObjectName(QString::fromUtf8("followButton"));
        followButton->setGeometry(QRect(410, 20, 111, 41));
        landButton = new QPushButton(centralwidget);
        landButton->setObjectName(QString::fromUtf8("landButton"));
        landButton->setGeometry(QRect(540, 20, 111, 41));
        logTextEdit = new QTextEdit(centralwidget);
        logTextEdit->setObjectName(QString::fromUtf8("logTextEdit"));
        logTextEdit->setGeometry(QRect(20, 80, 471, 81));
        plotWidgetY = new QWidget(centralwidget);
        plotWidgetY->setObjectName(QString::fromUtf8("plotWidgetY"));
        plotWidgetY->setGeometry(QRect(20, 340, 750, 150));
        plotWidgetY->setStyleSheet(QString::fromUtf8("background-color: rgb(238, 238, 236);"));
        plotWidgetZ = new QWidget(centralwidget);
        plotWidgetZ->setObjectName(QString::fromUtf8("plotWidgetZ"));
        plotWidgetZ->setGeometry(QRect(20, 500, 750, 150));
        plotWidgetZ->setStyleSheet(QString::fromUtf8("background-color: rgb(238, 238, 236);"));
        plotWidgetX = new QWidget(centralwidget);
        plotWidgetX->setObjectName(QString::fromUtf8("plotWidgetX"));
        plotWidgetX->setGeometry(QRect(20, 180, 750, 150));
        plotWidgetX->setStyleSheet(QString::fromUtf8("background-color: rgb(238, 238, 236);"));
        recvdataButton = new QPushButton(centralwidget);
        recvdataButton->setObjectName(QString::fromUtf8("recvdataButton"));
        recvdataButton->setGeometry(QRect(150, 20, 111, 41));
        MainWindow->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        connectButton->setText(QApplication::translate("MainWindow", "\351\200\232\344\277\241", nullptr));
        takeoffButton->setText(QApplication::translate("MainWindow", "\350\265\267\351\243\236", nullptr));
        followButton->setText(QApplication::translate("MainWindow", "\350\267\237\351\232\217", nullptr));
        landButton->setText(QApplication::translate("MainWindow", "\351\231\215\350\220\275", nullptr));
        recvdataButton->setText(QApplication::translate("MainWindow", "\346\216\245\346\224\266\346\225\260\346\215\256", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
