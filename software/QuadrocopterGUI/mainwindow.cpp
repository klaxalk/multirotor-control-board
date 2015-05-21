#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>
#include <QString>
#include <QRunnable>
#include <QThreadPool>
#include <QThread>
#include "serial.h"
#include "mpc/elevator/elevatorMpc.h"
#include "CMatrixLib.h"

int comPort=0;
int i = 0;
bool threadRun=false;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->pushButton_2->setEnabled(false);
    ui->pushButton_3->setEnabled(false);
    ui->pushButton_4->setEnabled(false);
    ui->pushButton_5->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    comPort=ui->textEdit->toPlainText().toInt();
    QMessageBox::information(this,"ConnectXbee","Com port sent: " + QString::number(comPort));
    ui->pushButton_2->setEnabled(true);
    ui->pushButton_3->setEnabled(true);
    ui->pushButton_4->setEnabled(true);
    ui->pushButton_5->setEnabled(true);
    startSerial(comPort);
    thread = new checkThread;
    thread->start();
    threadRun=true;

}

void MainWindow::on_pushButton_2_clicked()
{
    newQuadroVer = new quadrover(this);
    newQuadroVer->show();
}

void MainWindow::realtimeCheck()
{
    checkSerial();
}

void MainWindow::closeEvent (QCloseEvent *event)
{
    QMessageBox::StandardButton resBtn = QMessageBox::question( this,"Multicopter GUI",
                                                                tr("Are you sure?\n"),
                                                                QMessageBox::Cancel | QMessageBox::No | QMessageBox::Yes,
                                                                QMessageBox::Yes);
    if (resBtn != QMessageBox::Yes) {
        event->ignore();
    } else {
        if(threadRun){
            thread->m_abort=true;
            thread->wait();
        }
        event->accept();
    }
}

void MainWindow::on_pushButton_3_clicked()
{
    newQuadroHor = new quadrohor(this);
    newQuadroHor->show();
}

void MainWindow::on_pushButton_4_clicked()
{
    newQuadroOne = new quadroone(this);
    newQuadroOne->show();
}

void MainWindow::on_pushButton_5_clicked()
{
    newQuadro = new quadro(this);
    newQuadro->show();
}
