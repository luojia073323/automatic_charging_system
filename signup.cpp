#include "signup.h"
#include "login2.h"
#include "ui_signup.h"
#include "mainwindow.h"
#include <QFont>
#include <QMessageBox>
#include <QTimer>
#include <QSqlQuery>
#include <QDebug>
#include <QSqlError>

signup::signup(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::signup)
{
    ui->setupUi(this);

    this->setWindowTitle("注册界面");
    ui->comboBox->addItem("操作员");
    ui->comboBox->addItem("管理员");

    sqlite_Init();

    QSqlQuery query;
    QString select_max_sql="select max(id) from user1";
    query.prepare(select_max_sql);
    if(!query.exec())
    {
        qDebug() << query.lastError();
    }
    else
    {
        while(query.next())
        {
            int max_id = query.value(0).toInt();
            qDebug() << QString("%1").arg(max_id);
            QString strNew = QString("%1").arg(max_id,4,10,QLatin1Char('0'));
            ui->lineEdit_susername->setText("2023" + strNew);
        }
    }

    QFont font;
    font.setBold(true);
    font.setPointSize(10);

    QFont font1;
    font1.setBold(true);
    font1.setPointSize(16);

    ui->label->setFont(font);
    ui->label_2->setFont(font);
    ui->label_3->setFont(font);
    ui->label_5->setFont(font);

    ui->lineEdit_sensure->setFont(font);
    ui->lineEdit_spassword->setFont(font);
    ui->lineEdit_susername->setFont(font);

    ui->label_6->setFont(font1);
    ui->comboBox->setFont(font);
    ui->pushButton_sure->setFont(font);
    ui->pushButton_returnlogin->setFont(font);

    ui->label_6->setStyleSheet(QLatin1String("color: rgb(255,255,255)"));
    ui->label_5->setStyleSheet(QLatin1String("color: rgb(255,255,255)"));
    ui->label_2->setStyleSheet(QLatin1String("color: rgb(255,255,255)"));
    ui->label_3->setStyleSheet(QLatin1String("color: rgb(255,255,255)"));
    ui->label->setStyleSheet(QLatin1String("color: rgb(255,255,255)"));
    ui->pushButton_returnlogin->setStyleSheet("background-color: rgb(0,85,127); color: rgb(255,255,255);");
    ui->pushButton_sure->setStyleSheet("background-color: rgb(0,85,127); color: rgb(255,255,255);");

}

signup::~signup()
{
    delete ui;
} 

void signup::on_pushButton_sure_clicked()
{
    //  *********************************** register **************************************
    sqlite_Init();

    QFont font0;
    font0.setBold(true);
    font0.setPointSize(10);

    QMessageBox *msg = new QMessageBox();
    msg->setText("     两次密码输入不一致 !      ");
    msg->setFont(font0);
    msg->setWindowTitle("注册认证");
    msg->setStandardButtons(0);       

    QTimer::singleShot(5000,msg,SLOT(accept()));
    msg->setStyleSheet("background-color: rgb(21,6,97); color: rgb(255,255,255);");


    QMessageBox *msg3 = new QMessageBox();
    msg3->setText("     密码设置不能少于六位 !      ");
    msg3->setFont(font0);
    msg3->setWindowTitle("注册认证");
    msg3->setStandardButtons(0);

    QTimer::singleShot(5000,msg3,SLOT(accept()));
    msg3->setStyleSheet("background-color: rgb(21,6,97); color: rgb(255,255,255);");


    QString username = ui->lineEdit_susername->text();
    QString password = ui->lineEdit_spassword->text();
    QString surepassword = ui->lineEdit_sensure->text();
    emit sendID(ui->lineEdit_susername->text());
//    QString power = ui->comboBox->currentText();

    QFile file_user_id("G:\\DHBLAB research\\user_id.txt");
    file_user_id.open(QIODevice::WriteOnly);
    file_user_id.write(username.toStdString().data());
    file_user_id.close();

    QSqlQuery query;
    int passSize = password.size();
    if(passSize >= 6)
    {
        if(password == surepassword)
        {
            QString sql_user0=QString("insert into user1 (username,password,power,signupTime,modificationTime)"
                                      "values ('%1','%2','%3','%4','%5')")
                                      .arg(username)
                                      .arg(password)
                                      .arg(ui->comboBox->currentIndex())
                                      .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"))
                                      .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));



             if(!query.exec(sql_user0))
             {
                 qDebug()<<"insert into error";
             }
             else
             {
                 qDebug()<<"insert into success";

                 QMessageBox *msg1 = new QMessageBox();
                 msg1->setText("   您的账号为 ：" + QString("%1").arg(username)      + "   ...  "+'\n'
                              + "   您的密码为 ：" + QString("%2").arg(password)      + "   ...  "+'\n');
                 msg1->setFont(font0);
                 msg1->setWindowTitle("注册认证");
                 msg1->setStandardButtons(0);
                 QTimer::singleShot(8000,msg1,SLOT(accept()));
                 msg1->setStyleSheet("background-color: rgb(21,6,97); color: rgb(255,255,255);");
                 msg1->exec();

                 MainWindow *l2 = new MainWindow;
                 l2->show();
                 this->close();
             }

         }
         else
         {
             msg->exec();
         }
    }
    else
    {
         msg3->exec();
    }
    //  *********************************** register **************************************

}


void signup::on_pushButton_returnlogin_clicked()
{
    this->close();
    login2 *l = new login2;
    l->show();
}

