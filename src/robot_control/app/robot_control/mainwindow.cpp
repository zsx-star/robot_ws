#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setMaximumSize(337,253);
    this->setMinimumSize(337,253);
    socket = new QTcpSocket();
    ui->lineEdit_ip->setText("192.168.0.194");
    ui->lineEdit_port->setText("12345");
    cmd_header = "<robot>";

    keys_state = std::vector<bool>(4,false);
    send_cmd_thread = std::shared_ptr<std::thread>(new std::thread(std::bind(&MainWindow::sendCmdThread,this)));

    ui->pushButton_up->setShortcut(Qt::Key_Up);
    ui->pushButton_down->setShortcut(Qt::Key_Down);
    ui->pushButton_left->setShortcut(Qt::Key_Left);
    ui->pushButton_right->setShortcut(Qt::Key_Right);

}

MainWindow::~MainWindow()
{
    send_cmd_thread_flag = false;
    delete ui;
    delete socket;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    auto result = QMessageBox::question(this,"Close","Confirm Exit",QMessageBox::Yes,QMessageBox::No);
    if(result == QMessageBox::Yes)
    {
        send_cmd_thread_flag = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        event->accept();
    }
    else
        event->ignore();
}

void MainWindow::sendCmdThread()
{
    send_cmd_thread_flag = true;
    while(send_cmd_thread_flag)
    {
        for(int i=0; i<keys_state.size();++i)
        {
            if(keys_state[i])
            {
                std::string cmd = cmd_header+std::to_string(i+1);
                qint64 writeResult = socket->write(QString::fromStdString(cmd).toLatin1());
                bool BoolFlush = socket->flush();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void MainWindow::on_pushButton_connect_clicked(bool checked)
{
    if(ui->pushButton_connect->isChecked())
    {
        socket->connectToHost(ui->lineEdit_ip->text(),ui->lineEdit_port->text().toUShort());

        if(!socket->waitForConnected(3000))
        {
            QMessageBox::warning(this,"warning","connect failed!!!",QMessageBox::Yes);
            ui->pushButton_connect->setChecked(false);
        }
        else
        {
            ui->statusBar->showMessage("connect completed ",1000);
            ui->pushButton_connect->setText("disconnect");
        }
    }
    else
    {
        socket->close();
        ui->pushButton_connect->setText("connect");
    }
}

void MainWindow::on_pushButton_up_pressed()
{
    keys_state[0] = true;
}

void MainWindow::on_pushButton_up_released()
{
   keys_state[0] = false;
}

void MainWindow::on_pushButton_down_pressed()
{
    keys_state[1] = true;
}

void MainWindow::on_pushButton_down_released()
{
    keys_state[1] = false;
}

void MainWindow::on_pushButton_left_pressed()
{
    keys_state[2] = true;
}

void MainWindow::on_pushButton_left_released()
{
    keys_state[2] = false;
}

void MainWindow::on_pushButton_right_pressed()
{
    keys_state[3] = true;
}

void MainWindow::on_pushButton_right_released()
{
    keys_state[3] = false;
}
