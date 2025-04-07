#include "login2.h"
#include "ui_login2.h"
#include "mainwindow.h"
//#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QSqlDatabase>
#include <QSqlError>
#include <QSqlQuery>
#include <QDebug>
#include "signup.h"
#include <QFont>
#include <QTimer>
#include <QFile>

bool temp::optEnable = false;

 login2::login2(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::login2)
{
    ui->setupUi(this);

    this->setWindowTitle("登录界面");
    ui->comboBox->addItem("操作员");
    ui->comboBox->addItem("管理员");

    QFont font;
    font.setBold(true);
    font.setPointSize(16);

    ui->label_5->setFont(font);
    ui->label_5->setStyleSheet(QLatin1String("color: rgb(255,255,255)"));
    ui->pushButton_2->setStyleSheet("background-color: rgb(0,85,127); color: rgb(255,255,255);");
    ui->pushButton->setStyleSheet("background-color: rgb(0,85,127); color: rgb(255,255,255);");

//    if(ui->comboBox->currentIndex()==0)
//    {
//        emit sendData();
//        MainWindow *app = new MainWindow;
//        connect(this,SIGNAL(sendData()),app,SLOT(receiveData()));
//    }
}

login2::~login2()
{
    delete ui;
}


void login2::on_pushButton_clicked()
{

// **************************  standard example  ***************************
//      if (ui->lineEdit_user->text().trimmed() == tr("20000001")
//              && ui->lineEdit_passward->text() == tr("1111"))
//      {
//          temp::optEnable = true;
//        accept();

//       m = new MainWindow;
//       m->show();
//       this->hide();
//      }
//      else
//      {
//          QMessageBox::warning(this,"提示","用户名或密码输入错误");
//      }
//    **************************  standard example  ***************************

       sqlite_Init();

       QFont font0;
       font0.setBold(true);
       font0.setPointSize(10);

       QMessageBox *msg = new QMessageBox();
       msg->setText("   登录失败, 账户密码或权限选择错误 ！  ");
       msg->setFont(font0);
       msg->setWindowTitle("登录认证");
       msg->setStandardButtons(0);

       QTimer::singleShot(5000,msg,SLOT(accept()));
       msg->setStyleSheet("background-color: rgb(21,6,97); color: rgb(255,255,255);");

       QString username = ui->lineEdit_user->text();
       QFile file_user_id("G:\\DHBLAB research\\user_id.txt");
       file_user_id.open(QIODevice::WriteOnly);
       file_user_id.write(username.toStdString().data());
       file_user_id.close();
//       emit sendID0(username);

       QString password = ui->lineEdit_passward->text();
//       QString power = ui->comboBox->currentText();

       if (ui->comboBox->currentIndex()==1)
       {
           temp::optEnable = true;
       }
       else
           temp::optEnable = false;

       QString sql_user1 = QString("select * from user1 where username= '%1' "
                                   "and password='%2' and power='%3' ")
                .arg(username).arg(password).arg(ui->comboBox->currentIndex());
       QSqlQuery query(sql_user1);

       if(!query.next())
           {
               qDebug()<<"Login error";
               msg->exec();
           }
       else
           {
               qDebug()<<"Login success";
               l1 = new MainWindow;
               l1->show();
               this->hide();
           }
}


void login2::on_pushButton_2_clicked()
{
// **************** reset *****************
//    ui->lineEdit_user->clear();
//    ui->lineEdit_passward->clear();
//    ui->comboBox->clear();
// **************** reset *****************

// *************** jump to signup *****************
    this->close();
    signup *s = new signup;
    s->show();
// *************** jump to signup *****************

}


void sqlite_Init()
{
    QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE");
    db.setDatabaseName("G://DHBLAB research//user_test.db");
    if(!db.open())
    {
    qDebug()<<"****reason of failure****"<<db.lastError().text();
    }
    else
    {
        qDebug()<<"****connected successfully****!!!";
    }

    QString create_sql=QString("create table if not exists user1(id integer primary key autoincrement,"
                                                       "username text unique not NULL,"
                                                       "password text not NULL,"
                                                       "power text not NULL,"
                                                       "signupTime text not NULL,"
                                                       "modificationTime text not NULL)");
    QSqlQuery query;
    if(!query.exec(create_sql))
    {
        qDebug() << "table create error";
    }
    else
    {
        qDebug() << "table create success";
    }

}
