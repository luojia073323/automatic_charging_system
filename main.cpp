#include "mainwindow.h"
#include <QApplication>
#include "login2.h"
#include <QDebug>
#include <iostream>

void LogMessage(QtMsgType type, const QMessageLogContext &context, const QString &msg);
int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication a(argc, argv);
    MainWindow w;
    login2 l;
    l.show();

    qInstallMessageHandler(LogMessage);
    qDebug("This is a Debug message");
//    w.show();


    return a.exec();
}
