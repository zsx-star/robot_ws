#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle(tr("Robot Control"));
    this->setMaximumSize(390,250);
    this->setMinimumSize(390,250);
    performance_ = new PerformanceRecorder("config.ini");
    socket = new QTcpSocket();
    cmd_header = "<robot>";

    keys_state = std::vector<bool>(4,false);
    mutexes = std::vector<std::mutex>(4);
    setFocusPolicy(Qt::StrongFocus);

    loadUserPerformance();

}

MainWindow::~MainWindow()
{
    send_cmd_thread_flag = false;
    delete ui;
    delete socket;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    auto result = QMessageBox::question(this,"Close","Confirm Exit",QMessageBox::No,QMessageBox::Yes);
    if(result == QMessageBox::Yes)
    {
        send_cmd_thread_flag = false;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        dumpUserPerformance();
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
            //std::unique_lock<std::mutex> lock(mutexes[i]);
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
            send_cmd_thread = std::shared_ptr<std::thread>(new std::thread(std::bind(&MainWindow::sendCmdThread,this)));
            ui->statusBar->showMessage("connect completed ",1000);
            ui->pushButton_connect->setText("disconnect");
        }
    }
    else
    {
        send_cmd_thread_flag = false;
        socket->close();
        ui->pushButton_connect->setText("connect");
    }
}

void MainWindow::on_pushButton_up_pressed()
{
    //std::unique_lock<std::mutex> lock(mutexes[0]);
    keys_state[0] = true;
}

void MainWindow::on_pushButton_up_released()
{
   //std::unique_lock<std::mutex> lock(mutexes[0]);
   keys_state[0] = false;
}

void MainWindow::on_pushButton_down_pressed()
{
    //std::unique_lock<std::mutex> lock(mutexes[1]);
    keys_state[1] = true;
}

void MainWindow::on_pushButton_down_released()
{
    //std::unique_lock<std::mutex> lock(mutexes[1]);
    keys_state[1] = false;
}

void MainWindow::on_pushButton_left_pressed()
{
    //std::unique_lock<std::mutex> lock(mutexes[2]);
    keys_state[2] = true;
}

void MainWindow::on_pushButton_left_released()
{
    //std::unique_lock<std::mutex> lock(mutexes[2]);
    keys_state[2] = false;
}

void MainWindow::on_pushButton_right_pressed()
{
    //std::unique_lock<std::mutex> lock(mutexes[3]);
    keys_state[3] = true;
}

void MainWindow::on_pushButton_right_released()
{
    //std::unique_lock<std::mutex> lock(mutexes[3]);
    keys_state[3] = false;
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{

    if(event->key() == Qt::Key_Up)
        on_pushButton_up_pressed();
    else if(event->key() == Qt::Key_Down)
        on_pushButton_down_pressed();
    else if(event->key() == Qt::Key_Left)
        on_pushButton_left_pressed();
    else if(event->key() == Qt::Key_Right)
        on_pushButton_right_pressed();

}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Up)
        on_pushButton_up_released();
    else if(event->key() == Qt::Key_Down)
        on_pushButton_down_released();
    else if(event->key() == Qt::Key_Left)
        on_pushButton_left_released();
    else if(event->key() == Qt::Key_Right)
        on_pushButton_right_released();
}


void MainWindow::loadUserPerformance()
{
    QString ip;
    performance_->readPerformance(QString("ip"),ip,QString("socket"));
    ui->lineEdit_ip->setText(ip);

    QString port;
    performance_->readPerformance(QString("port"),port,QString("socket"));
    ui->lineEdit_port->setText(port);
}

void MainWindow::dumpUserPerformance()
{
    performance_->writePerformance(QString("ip"), ui->lineEdit_ip->text(),QString("socket"));
    performance_->writePerformance(QString("port"),ui->lineEdit_port->text(),QString("socket"));
}

