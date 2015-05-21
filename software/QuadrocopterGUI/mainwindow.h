#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "quadro.h"
#include "quadrover.h"
#include "quadrohor.h"
#include "quadroone.h"
#include "qcustomplot.h"
#include "checkthread.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void realtimeCheck();
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void closeEvent (QCloseEvent *event);

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

private:
    Ui::MainWindow *ui;
    quadro *newQuadro;
    quadroone *newQuadroOne;
    quadrover *newQuadroVer;
    quadrohor *newQuadroHor;
    QTimer mainDataTimer;
    checkThread *thread;
};

#endif // MAINWINDOW_H
