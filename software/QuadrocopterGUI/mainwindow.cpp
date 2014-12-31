#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>
#include <QString>

int comPort=0;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->pushButton_2->setEnabled(false);
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
     //method for open communication with comPort

}

void MainWindow::on_pushButton_2_clicked()
{
    newQuadro = new quadro(this);
    newQuadro->show();
}
