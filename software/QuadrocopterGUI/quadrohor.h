#ifndef QUADROHOR_H
#define QUADROHOR_H

#include <QMainWindow>
#include <QCheckBox>
#include "qcustomplot.h"
#include "sendtrajectory.h"
#include "failuredetection.h"
#include "errordialog.h"


namespace Ui {
class quadrohor;
}

class quadrohor : public QMainWindow
{
    Q_OBJECT

public:
    explicit quadrohor(QWidget *parent = 0);
    ~quadrohor();
    void startGraph();
    void createScrollCheckBox();
    void setAllCheckFalse();
    void detectStatus();
    void createLegend();
    void currentDateTime();
    void setSignalsOn();
    void startFailureDetectionThread();
    void graph_1(QCustomPlot *graph1);
    void graph_2(QCustomPlot *graph2);


private slots:
    void realtimeDataSlotGraph1();
    void realtimeDataSlotGraph2();
    void realtimeDataStatus();

    void on_comboBox_activated(int index);
    void Plot1SignalAll_clicked();
    void Plot2SignalAll_clicked();

    void Plot1Signals_clicked();
    void Plot2Signals_clicked();

    void on_actionK1_triggered();
    void on_actionK2_triggered();
    void on_actionK3_triggered();
    void on_actionKC1_triggered();

    void on_actionOn_triggered();
    void on_actionOff_triggered();
    void on_actionVelocity_triggered();
    void on_actionPosition_triggered();
    void on_actionFollow_trajectory_triggered();
    void on_actionLoad_trajectory_triggered();
    void on_actionBoth_triggered();
    void on_actionOff_3_triggered();
    void on_actionOn_2_triggered();
    void on_actionOff_2_triggered();
    void on_actionFollow_trajectory_off_triggered();

    void on_actionRelative_triggered();
    void on_actionAbsolute_triggered();
    void on_actionRelative_2_triggered();
    void on_actionAbsolute_2_triggered();
    void on_actionRelative_3_triggered();
    void on_actionAbsolute_3_triggered();
    void on_actionRelative_4_triggered();
    void on_actionAbsolute_4_triggered();
    void on_actionRelative_5_triggered();
    void on_actionAbsolute_5_triggered();
    void closeEvent (QCloseEvent *event);

    void on_pushButton_clicked();

private:
    Ui::quadrohor *ui;
    QTimer dataTimer;

    QCheckBox *Plot1Signals[38];
    QCheckBox *Plot2Signals[38];
    bool failureDetectionBool;
    unsigned char kopter;
    sendTrajectory *newTrajectory;
    errorDialog *errorDia;
    float dataValues[37]={ };
    char myTime[80];
   // failuredetection *failureDetectionThread;
};

#endif // QUADROHOR_H
