#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <QTableWidget>
#include <QDebug>
#include <QFileDialog>
#include <QFileInfo>
#include <QStringList>
#include <QThread>
#include <QPushButton>
#include <QTimer>
#include <QMessageBox>
#include <QTime>
#include <string>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <math.h>
#include "login2.h"
#include "signup.h"
#include "car_tcp.h"
#include <QMutex>
#include <QTextCodec>
#include <QMessageBox>
#include <QFile>
#include <QDateTime>
#include <QElapsedTimer>
#include <QString>

using namespace std;
int index = 0;
bool MS = true;     //t 主机
bool flag_Sw=false;
bool isFirstTime = true;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    tcpServer = new QTcpServer(this);
    tcpSocket = new QTcpSocket(this);

    //默认使用主机模式，等待新的连接
    connect(tcpServer,SIGNAL(newConnection()),this,SLOT(newConnection_Slot()));
    ui->portEdit->setText("8081");
    ui->ipED->setText("172.20.10.10");
//    tcp_test();

    timer1 = new QTimer(this);
    timer1->start(5000);
    connect(timer1,&QTimer::timeout,[=](){
        if(ui->ipED_4->text()=="100")
        {
            QFont MesFont;
            MesFont.setBold(true);
            MesFont.setPointSize(12);

            QMessageBox *Charmsg = new QMessageBox();
            Charmsg->setText("     电量已充满！      ");
            Charmsg->setFont(MesFont);
            Charmsg->setWindowTitle("提示");

            QPushButton *okButton = Charmsg->addButton("确认",QMessageBox::YesRole);
            Charmsg->setStyleSheet("background-color: rgb(21,6,97); color: rgb(255,255,255);");

            while (true)
            {
                Charmsg->exec();
                if (Charmsg->clickedButton() == okButton)
                {
                    break;
                }
            }
                qApp->quit();
        }
    });

    //未登录不能修改 日志 & 用户管理 & AMR参数
//    ui->workData->setEnabled(temp::optEnable);
//    ui->userManage->setEnabled(temp::optEnable);

    QString path = QFileDialog::getOpenFileName(
                                                this,
                                                tr("打开报警日志"),
                                                "G:\\DHBLAB research\\GithubCode\\build_dhb_agv_git_main",
                                                tr("md(*.md);;All files(*.*)"));
    QFile file0(path);
    //            QFile file0("G:\\DHBLAB research\\QTcode\\build-11_log-Desktop_Qt_5_9_5_MinGW_32bit-Debug\\2022.12.04.md");
    file0.open(QIODevice::ReadOnly);
    QString string1;
    string1 = file0.readAll();
    QString str(string1);
    ui->textEdit->setText(str);

    QFont front1 = QFont("宋体",12);
    ui->textEdit->setFont(front1);
    ui->textEdit_2->setFont(front1);

    ui->pushButton_2->setFont(front1);
    ui->pushButton_4->setFont(front1);
    ui->pushButton_17->setFont(front1);
    ui->pushButton_3->setFont(front1);
    ui->pushButton_5->setFont(front1);
    ui->pushButton_16->setFont(front1);

    ui->textEdit->setStyleSheet(QLatin1String("color: rgb(255,255,255)"));
    ui->textEdit_2->setStyleSheet(QLatin1String("color: rgb(255,255,255)"));

    ui->pushButton_2->setStyleSheet("background-color: rgb(0,85,127); color: rgb(255,255,255);");
    ui->pushButton_4->setStyleSheet("background-color: rgb(0,85,127); color: rgb(255,255,255);");
    ui->pushButton_17->setStyleSheet("background-color: rgb(0,85,127); color: rgb(255,255,255);");
    ui->pushButton_3->setStyleSheet("background-color: rgb(0,85,127); color: rgb(255,255,255);");
    ui->pushButton_5->setStyleSheet("background-color: rgb(0,85,127); color: rgb(255,255,255);");
    ui->pushButton_16->setStyleSheet("background-color: rgb(0,85,127); color: rgb(255,255,255);");

    ui->dateTimeEdit_start_1->setStyleSheet("background-color: rgb(0,85,127); color: rgb(255,255,255);");
    ui->dateTimeEdit_end_1->setStyleSheet("background-color: rgb(0,85,127); color: rgb(255,255,255);");

    ui->label_2->setStyleSheet("background-color: rgb(21,6,97); color: rgb(255,255,255);");
    ui->label_4->setStyleSheet("background-color: rgb(21,6,97); color: rgb(255,255,255);");
    ui->label_5->setStyleSheet("background-color: rgb(21,6,97); color: rgb(255,255,255);");

    ui->RecvEdit->setStyleSheet("color: rgb(255,255,255);");
    ui->portEdit->setStyleSheet("color: rgb(255,255,255);");
    ui->sendEdit->setStyleSheet(QLatin1String("color: rgb(255,255,255)"));
    ui->ipED->setStyleSheet(QLatin1String("color: rgb(255,255,255)"));
    ui->ipED_2->setStyleSheet(QLatin1String("color: rgb(255,255,255)"));
    ui->ipED_3->setStyleSheet(QLatin1String("color: rgb(255,255,255)"));
    ui->ipED_4->setStyleSheet(QLatin1String("color: rgb(255,255,255)"));
    ui->ipED_5->setStyleSheet(QLatin1String("color: rgb(255,255,255)"));
    ui->ipED_6->setStyleSheet(QLatin1String("color: rgb(255,255,255)"));

    ui->stackedWidget->setCurrentIndex(index);

    QFile file_user_id("G:\\DHBLAB research\\user_id.txt");
    file_user_id.open(QIODevice::ReadOnly);
    QString string_user_id;
    string_user_id = file_user_id.readAll();
    ui->label_2->setText(string_user_id);
    file_user_id.close();

}

//检测是否有新连接进来
void MainWindow::newConnection_Slot(){

    tcpSocket=tcpServer->nextPendingConnection();//得到通信的套接字对象
    connect(tcpSocket,SIGNAL(readyRead()),this,SLOT(readyRead_Slot()));

    connect(tcpSocket,SIGNAL(disconnected()),this,SLOT(disconnected_Slot()));
   // ui->label_3->setText("已连接");
    ui->label_3->setStyleSheet("border-image: url(:/res/connect.png)");
}

//服务器或客户机连接状态
void MainWindow::disconnected_Slot(){

    tcpSocket->close();
    ui->label_3->setStyleSheet("border-image: url(:/res/discon.png)");
}


//客户机连接
void MainWindow::connected_Slot(){

    connect(tcpSocket,SIGNAL(readyRead()),this,SLOT(readyRead_Slot()));
   // ui->label_3->setText("已连接");
    ui->label_3->setStyleSheet("border-image: url(:/res/connect.png)");

    connect(tcpSocket,SIGNAL(disconnected()),this,SLOT(disconnected_Slot()));
}

//有内容进来调用

void MainWindow::readyRead_Slot(){

    /*   QString buf;
    buf = tcpSocket->readAll();
    ui->RecvEdit->appendPlainText(buf);*/

    timer1 = new QTimer(this);
    timer1->start(2000);
    connect(timer1,&QTimer::timeout,[=](){

    QByteArray receiveDate;
    QTextCodec *tc = QTextCodec::codecForName("GBK");  //编码转换,必须转换编码，否则乱码

    while(!tcpSocket->atEnd()){
        receiveDate = tcpSocket->readAll();
                               }
    if (!receiveDate.isEmpty())
    {
        QElapsedTimer timer;
        QString strBuf=tc->toUnicode(receiveDate);
        QTextStream ts(&strBuf);
        ts.seek(strBuf.lastIndexOf("\n")+2);
        QString receiveLine = ts.readLine();

        QStringList mes = receiveLine.split(",",QString::SkipEmptyParts);
        vol = mes[0].toDouble();
        cur = mes[1].toDouble();
        mar = mes[2].toDouble();

        if(cur != 655.36 && isFirstTime)
        {
            QDateTime startTim;
            startTim = QDateTime::currentDateTime();

            QFile file1("G:\\GithubCode\\wifi_receive.txt");
            file1.open(QIODevice::WriteOnly);
            file1.write(startTim.toString().toUtf8());
            file1.close();

            timer.start();
            isFirstTime = false;
        }

        if(cur == 655.36 && (!isFirstTime))
        {
            qint64 elapsed = timer.elapsed();

            QDateTime endTim;
            endTim = QDateTime::currentDateTime();

            QFile file1("G:\\GithubCode\\wifi_receive.txt");
            file1.open(QIODevice::WriteOnly);
            file1.write(","+endTim.toString().toUtf8());
            file1.close();

            int seconds = elapsed / 1000;
            int minutes = seconds / 60;
            int hours = minutes / 60;
            int mins = (seconds / 60) % 60;

            QString hour=QString("%1").arg(hours);
            QString min=QString("%1").arg(mins);

            ui->ipED_5->setText(hour);
            ui->ipED_6->setText(min);
        }

        ui->RecvEdit->appendPlainText(strBuf);
        ui->ipED_2->setText(mes[1]);
        ui->ipED_3->setText(mes[2]);
        ui->ipED_4->setText(mes[3]);

        QFile file1("G:\\GithubCode\\wifi_receive.txt");
        file1.open(QIODevice::ReadOnly);
        QString tcpString;
        tcpString = file1.readLine();
        QStringList SETim = tcpString.split(",",QString::SkipEmptyParts);
        ST = SETim[0].toDouble();
        ET = SETim[1].toDouble();

        sql_Init();
        QSqlQuery query;
        QString sql1=QString("insert into wifi (voltage,current,margin,startTime,endTime)"
                                  "values ('%1','%2','%3','%4','%5')")
                                  .arg(vol)
                                  .arg(cur)
                                  .arg(mar)
                                  .arg(ST)
                                  .arg(ET);

         if(!query.exec(sql1))
         {
             qDebug()<<"insert error";
         }
         else
         {
             qDebug()<<"insert success";
         }

    receiveDate.clear();
    }
                                            });
                                }


MainWindow::~MainWindow()
{
    delete ui;
}

void LogMessage(QtMsgType type, const QMessageLogContext &context, const QString &msg){
    static QMutex mutex;
    mutex.lock();
    QString text;
    switch(type){
    case QtDebugMsg:    text = QString("Debug:");   break;
    case QtWarningMsg:  text = QString("Warning:"); break;
    case QtCriticalMsg: text = QString("Critical:");break;
    case QtFatalMsg:    text = QString("Fatal:");   break;
    default:                                        break;
    }

    QString context_info = QString("File:(%1) Line:(%2)").arg(QString(context.file)).arg(context.line);
    QString current_date_time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString message = QString("%1 %2").arg(current_date_time).arg(msg);
    QString timestr=QDateTime::currentDateTime().toString("yyyy.MM.dd");
    QString fileName = timestr + ".md";
    QFile file0(fileName);
    file0.open(QIODevice::WriteOnly | QIODevice::Append);
    QTextStream text_stream(&file0);
    text_stream << message << "\r\n\r\n";
    file0.flush();
    file0.close();
    mutex.unlock();
}

void MainWindow::on_pushButton_4_clicked()
{
    ui->textEdit->clear();
}

void MainWindow::on_pushButton_2_clicked()
{
    ui->textEdit->update();
}


void MainWindow::on_Exit_4_clicked()
{
    QFile file_user_id("G:\\DHBLAB research\\user_id.txt");
    file_user_id.open(QIODevice::ReadOnly);
    QString string_user_id;
    string_user_id = file_user_id.readAll();
    ui->label_2->setText(string_user_id);
    file_user_id.close();
}


void MainWindow::on_Exit_clicked()
{
    QFont font4;
    font4.setBold(true);
    font4.setPointSize(10);

    QMessageBox *msg4 = new QMessageBox();
    msg4->setText("     是否确认退出登录 ？      ");
    msg4->setFont(font4);
    msg4->setWindowTitle("提示");

    QPushButton *okButton = msg4->addButton("是",QMessageBox::YesRole);
    QPushButton *cancelButton = msg4->addButton("否",QMessageBox::NoRole);
    msg4->setStyleSheet("background-color: rgb(21,6,97); color: rgb(255,255,255);");

    msg4->exec();

    if(msg4->clickedButton() == okButton)
    {
        qApp->closeAllWindows();
        qApp->setQuitOnLastWindowClosed(false);
        login2 *l = new login2;
        l->show();
    }
    else if(msg4->clickedButton() == cancelButton)
    {
          msg4->close();
    }
}

void MainWindow::on_Exit_2_clicked()
{
    qApp->closeAllWindows();
    qApp->setQuitOnLastWindowClosed(false);
    login2 *l = new login2;
    l->show();
}

void MainWindow::on_pushButton_3_clicked()
{
    ui->textEdit_2->update();
}

void MainWindow::on_pushButton_5_clicked()
{
    ui->textEdit_2->clear();
}

void MainWindow::on_Exit_3_clicked()
{
    index = 1-index;
    ui->stackedWidget->setCurrentIndex(index);
}


void MainWindow::on_openBt_clicked()
{
    flag_Sw=!flag_Sw;
    if(flag_Sw)
    {
        if(MS)          //主机
        {
            //connect(tcpServer,SIGNAL(newConnection()),this,SLOT(newConnection_Slot()));
            tcpServer->listen(QHostAddress::Any,ui->portEdit->text().toUInt());
        }
        else            //客户机
        {
            tcpSocket->connectToHost(ui->ipED->text(),ui->portEdit->text().toUInt());
            connect(tcpSocket,SIGNAL(connected()),this,SLOT(connected_Slot()));
        }
        ui->openBt->setStyleSheet("border-image: url(:/res/open.png)");
        ui->wifi->setStyleSheet("border-image: url(:/res/wifi_on.png)");
    }
    else
    {
        tcpServer->close();
        tcpSocket->close();
        ui->openBt->setStyleSheet("border-image: url(:/res/close.png)");
        ui->wifi->setStyleSheet("border-image: url(:/res/wifi_off.png)");
    }

    QString text = ui->textEdit_2->toPlainText();
    ui->textEdit_2->insertPlainText(text +='\n');
    QString strtime = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
    ui->textEdit_2->insertPlainText(strtime + "  " + "通讯连接" + '\n');

    QTextCursor cursor=ui->textEdit_2->textCursor();
    cursor.movePosition(QTextCursor::End);
    ui->textEdit_2->setTextCursor(cursor);
}

void MainWindow::on_sendBt_clicked()
{
    QString SendCon =  ui->sendEdit->text().toLocal8Bit().data();
    if(flag_Sw)
    {
        if(SendCon!="")
        {
            //封装编码
            QByteArray receiveDate;
            QTextCodec *tc = QTextCodec::codecForName("GBK");  //编码转换,必须转换编码，否则乱码

            //对发送框编码
            receiveDate =  ui->sendEdit->text().toLocal8Bit().data();
            QString strBuf=tc->toUnicode(receiveDate);

            //整合符号 ->
            QString  str = "->";
            QString str2 = str.append(strBuf);

             //向输出框打印发送的数据
             ui->RecvEdit->appendPlainText(str2);
             tcpSocket->write(ui->sendEdit->text().toLocal8Bit().data());
        }
        else
        QMessageBox::critical(this,"警告","不能发送空白信息");
    }
    else
    QMessageBox::critical(this,"提示","发送失败，网络尚未连接");

    QString text = ui->textEdit_2->toPlainText();
    ui->textEdit_2->insertPlainText(text +='\n');
    QString strtime = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
    ui->textEdit_2->insertPlainText(strtime + "  " + "发送数据" + '\n');

    QTextCursor cursor=ui->textEdit_2->textCursor();
    cursor.movePosition(QTextCursor::End);
    ui->textEdit_2->setTextCursor(cursor);
}

void MainWindow::on_clearBt_clicked()
{
    ui->RecvEdit->clear();

    QString text = ui->textEdit_2->toPlainText();
    ui->textEdit_2->insertPlainText(text +='\n');
    QString strtime = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
    ui->textEdit_2->insertPlainText(strtime + "  " + "清除数据" + '\n');

    QTextCursor cursor=ui->textEdit_2->textCursor();
    cursor.movePosition(QTextCursor::End);
    ui->textEdit_2->setTextCursor(cursor);
}

void MainWindow::on_sendBt_pressed()
{
     ui->sendBt->setStyleSheet("border-image: url(:/res/send_a.png)");
}

void MainWindow::on_sendBt_released()
{
    ui->sendBt->setStyleSheet("border-image: url(:/res/send.png)");
}

void MainWindow::on_clearBt_pressed()
{
    ui->clearBt->setStyleSheet("border-image: url(:/res/clear_a.png)");
}

void MainWindow::on_clearBt_released()
{
    ui->clearBt->setStyleSheet("border-image: url(:/res/clear_b.png)");
}

void MainWindow::on_clearS_pressed()
{
    ui->sendEdit->clear();
    ui->clearS->setStyleSheet("border-image: url(:/res/clearS_a.png)");

    QString text = ui->textEdit_2->toPlainText();
    ui->textEdit_2->insertPlainText(text +='\n');
    QString strtime = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
    ui->textEdit_2->insertPlainText(strtime + "  " + "清除发送" + '\n');

    QTextCursor cursor=ui->textEdit_2->textCursor();
    cursor.movePosition(QTextCursor::End);
    ui->textEdit_2->setTextCursor(cursor);
}

void MainWindow::on_clearS_released()
{
    ui->clearS->setStyleSheet("border-image: url(:/res/clearS.png)");
}

void MainWindow::on_ms_clicked()
{
    if(!flag_Sw)
    {
       tcpServer->close();
       tcpSocket->close();
       if(MS)          //客户机
       {
           MS=false;
           ui->label->setText("客户机模式");
           ui->ms->setStyleSheet("border-image: url(:/res/client.png);");
       }
       else            //主机
       {
           MS=true;
           //必须注销，否则切换为主机时候，主机不能用
           //connect(tcpServer,SIGNAL(newConnection()),this,SLOT(newConnection_Slot()));
           ui->label->setText("主机模式");
           ui->ms->setStyleSheet("border-image: url(:/res/server.png);");
       }
    }
    else
       QMessageBox::critical(this,"提示","请先关闭网络，再切换模式类型");

    QString text = ui->textEdit_2->toPlainText();
    ui->textEdit_2->insertPlainText(text +='\n');
    QString strtime = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
    ui->textEdit_2->insertPlainText(strtime + "  " + "主客对调" + '\n');

    QTextCursor cursor=ui->textEdit_2->textCursor();
    cursor.movePosition(QTextCursor::End);
    ui->textEdit_2->setTextCursor(cursor);
}

void sql_Init()
{
    QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE");
    db.setDatabaseName("G:\\GithubCode\\tcp.db");
    if(!db.open())
    {
    qDebug()<<"*失败原因*"<<db.lastError().text();
    }
    else
    {
        qDebug()<<"*连接成功*";
    }

    QString create_sql1=QString("create table if not exists wifi(id integer primary key autoincrement,"
                                                       "voltage text unique not NULL,"
                                                       "current text not NULL,"
                                                       "margin text not NULL,"
                                                       "startTime text not NULL,"
                                                       "endTime text not NULL)");
    QSqlQuery query;
    if(!query.exec(create_sql1))
    {
        qDebug() << "*table create error*";
    }
    else
    {
        qDebug() << "*table create success*";
    }
}
