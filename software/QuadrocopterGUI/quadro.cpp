#include <QDebug>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>
#include <QString>
#include <QTime>
#include <iostream>
#include <fstream>
#include <string>
#include "quadro.h"
#include "ui_quadro.h"
#include "receive.h"
#include "send.h"
#include "time.h"

using namespace std;

quadro::quadro(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::quadro)
{
    ui->setupUi(this);
    failureDetectionBool=false;
    //failureDetectionThread = new failuredetection;
    currentDateTime();
    createScrollCheckBox();
    createLegend();
    setAllCheckFalse();
    startGraph();
    detectStatus();
    on_actionK1_triggered();
    on_comboBox_activated(0);
}

quadro::~quadro()
{
    delete ui;
}

void quadro::startGraph()
{
    graph_1(ui->graph1);
    graph_2(ui->graph2);
    graph_3(ui->graph3);
    graph_4(ui->graph4);

    ui->graph1->replot();
    ui->graph2->replot();
    ui->graph3->replot();
    ui->graph4->replot();
}

void quadro::graph_1(QCustomPlot *graph1)
{
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
    QMessageBox::critical(this, "", "You're using Qt < 4.7, the realtime data needs functions that are available with Qt 4.7 to work properly");
#endif

    // set title of plot:
    //graph1->plotLayout()->insertRow(0);
    //graph1->plotLayout()->addElement(0, 0, new QCPPlotTitle(graph1, "Plot 1"));


    graph1->addGraph(); // blue line
    graph1->graph(0)->setPen(QPen(Qt::blue));
    graph1->graph(0)->setName("Signal 1");
    //graph1->graph(0)->setBrush(QBrush(QColor(240, 255, 200)));      // vybarveni od modre cary
    // graph1->graph(0)->setAntialiasedFill(false);
    graph1->addGraph(); // red line
    graph1->graph(1)->setPen(QPen(Qt::red));
    graph1->graph(1)->setName("Signal 2");
    // graph1->graph(0)->setChannelFillGraph(graph1->graph(1));    // upresneni ze od modre k cervene
    graph1->addGraph();
    graph1->graph(2)->setPen(QPen(Qt::green));
    graph1->graph(2)->setName("Signal 3");
    graph1->addGraph();
    graph1->graph(3)->setPen(QPen(Qt::black));
    graph1->graph(3)->setName("Signal 4");
    graph1->addGraph();
    graph1->graph(4)->setPen(QPen(Qt::cyan));
    graph1->graph(4)->setName("Signal 5");
    graph1->addGraph();
    graph1->graph(5)->setPen(QPen(Qt::gray));
    graph1->graph(5)->setName("Signal 6");
    graph1->addGraph();
    graph1->graph(6)->setPen(QPen(Qt::yellow));
    graph1->graph(6)->setName("Signal 7");
    graph1->addGraph();
    graph1->graph(7)->setPen(QPen(Qt::magenta));
    graph1->graph(7)->setName("Signal 8");
    graph1->addGraph();
    graph1->graph(8)->setPen(QPen(Qt::darkBlue));
    graph1->graph(8)->setName("Signal 9");
    graph1->addGraph();
    graph1->graph(9)->setPen(QPen(Qt::darkRed));
    graph1->graph(9)->setName("Signal 10");
    graph1->addGraph();
    graph1->graph(10)->setPen(QPen(Qt::darkGreen));
    graph1->graph(10)->setName("Signal 11");
    graph1->addGraph();
    graph1->graph(11)->setPen(QPen(Qt::darkCyan));
    graph1->graph(11)->setName("Signal 12");
    graph1->addGraph();
    graph1->graph(12)->setPen(QPen(Qt::darkGray));
    graph1->graph(12)->setName("Signal 13");
    graph1->addGraph();
    graph1->graph(13)->setPen(QPen(Qt::darkYellow));
    graph1->graph(13)->setName("Signal 14");
    graph1->addGraph();
    graph1->graph(14)->setPen(QPen(Qt::darkMagenta));
    graph1->graph(14)->setName("Signal 15");
    graph1->addGraph();
    //
    graph1->graph(15)->setPen(QPen(QColor(85,0,130)));//fialová
    graph1->graph(15)->setName("Signal 16");
    graph1->addGraph();
    graph1->graph(16)->setPen(QPen(QColor(255,170,255)));//růžová
    graph1->graph(16)->setName("Signal 17");
    graph1->addGraph();
    graph1->graph(17)->setPen(QPen(QColor(0,255,130)));//svetle zelená
    graph1->graph(17)->setName("Signal 18");
    graph1->addGraph();
    graph1->graph(18)->setPen(QPen(QColor(170,170,255)));//svetle fialová
    graph1->graph(18)->setName("Signal 19");
    graph1->addGraph();
    graph1->graph(19)->setPen(QPen(QColor(255,130,0)));//oranzová
    graph1->graph(19)->setName("Signal 20");
    graph1->addGraph();
    graph1->graph(20)->setPen(QPen(QColor(80,25,0)));//hnědá
    graph1->graph(20)->setName("Signal 21");
    graph1->addGraph();
    graph1->graph(21)->setPen(QPen(QColor(100,0,80)));//vínová
    graph1->graph(21)->setName("Signal 22");
    graph1->addGraph();
    graph1->graph(22)->setPen(QPen(QColor(170,255,0)));// zarive zelená
    graph1->graph(22)->setName("Signal 23");
    graph1->addGraph();
    graph1->graph(23)->setPen(QPen(QColor(255,170,110)));//vybledlá oranžová
    graph1->graph(23)->setName("Signal 24");
    graph1->addGraph();
    graph1->graph(24)->setPen(QPen(QColor(175,90,0)));//tmavě oranžová
    graph1->graph(24)->setName("Signal 25");
    graph1->addGraph();
    graph1->graph(25)->setPen(QPen(QColor(80,100,120)));//vybledlá modrá
    graph1->graph(25)->setName("Signal 26");
    graph1->addGraph();
    graph1->graph(26)->setPen(QPen(QColor(170,60,110)));//tmavě růžová
    graph1->graph(26)->setName("Signal 27");
    graph1->addGraph();
    graph1->graph(27)->setPen(QPen(QColor(85,85,0)));//tmavá žlutá
    graph1->graph(27)->setName("Signal 28");
    graph1->addGraph();
    graph1->graph(28)->setPen(QPen(QColor(80,70,70)));// ? ?
    graph1->graph(28)->setName("Signal 29");
    graph1->addGraph();
    graph1->graph(29)->setPen(QPen(QColor(10,100,80)));//zelená
    graph1->graph(29)->setName("Signal 30");
    graph1->addGraph();
    graph1->graph(30)->setPen(QPen(QColor(30,110,130)));//tmavě tyrkysová
    graph1->graph(30)->setName("Signal 31");
    graph1->addGraph();
    graph1->graph(31)->setPen(QPen(QColor(180,180,180)));//stříbrná
    graph1->graph(31)->setName("Signal 32");
    graph1->addGraph();
    graph1->graph(32)->setPen(QPen(Qt::red));//
    graph1->graph(32)->setName("Signal 33");
    graph1->addGraph();
    graph1->graph(33)->setPen(QPen(Qt::blue));//
    graph1->graph(33)->setName("Signal 34");
    graph1->addGraph();
    graph1->graph(34)->setPen(QPen(Qt::green));//
    graph1->graph(34)->setName("Signal 35");
    graph1->addGraph();
    graph1->graph(35)->setPen(QPen(Qt::magenta));//
    graph1->graph(35)->setName("Signal 36");
    graph1->addGraph();
    graph1->graph(36)->setPen(QPen(Qt::gray));//
    graph1->graph(36)->setName("Signal 37");

    graph1->xAxis->setTickLabelType(QCPAxis::ltDateTime);       // pro casovou osu
    graph1->xAxis->setDateTimeFormat("hh:mm:ss");
    graph1->xAxis->setAutoTickStep(false);
    graph1->xAxis->setTickStep(2);


    graph1->axisRect()->setupFullAxesBox();

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(graph1->xAxis, SIGNAL(rangeChanged(QCPRange)), graph1->xAxis2, SLOT(setRange(QCPRange)));
    connect(graph1->yAxis, SIGNAL(rangeChanged(QCPRange)), graph1->yAxis2, SLOT(setRange(QCPRange)));

    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlotGraph1()));
    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void quadro::realtimeDataSlotGraph1()
{
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
    double key1 = 0;
#else
    double key1 = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
#endif
    static double lastPointKey1 = 0;
    if (key1-lastPointKey1 > 0.01) // at most add point every 10 ms
    {

        dataValues[0] = getTelemetry(kopter,TELEMETRIES.GROUND_DISTANCE_ESTIMATED);
        dataValues[1] = getTelemetry(kopter,TELEMETRIES.GROUND_DISTANCE);
        dataValues[2] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_SPEED);
        dataValues[3] = getTelemetry(kopter,TELEMETRIES.AILERON_SPEED);
        dataValues[4] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_SPEED_ESTIMATED);
        dataValues[5] = getTelemetry(kopter,TELEMETRIES.AILERON_SPEED_ESTIMATED);
        dataValues[6] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_POS_ESTIMATED);
        dataValues[7] = getTelemetry(kopter,TELEMETRIES.AILERON_POS_ESTIMATED);
        dataValues[8] = getTelemetry(kopter,TELEMETRIES.THROTTLE_CONTROLLER_OUTPUT);
        dataValues[9] = getTelemetry(kopter,TELEMETRIES.THROTTLE_SPEED);
        dataValues[10] = getTelemetry(kopter,TELEMETRIES.AILERON_VEL_CONTROLLER_OUTPUT);
        dataValues[11] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_VEL_CONTROLLER_OUTPUT);
        dataValues[12] = getTelemetry(kopter,TELEMETRIES.AILERON_POS_CONTROLLER_OUTPUT);
        dataValues[13] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_POS_CONTROLLER_OUTPUT);
        dataValues[14] = getTelemetry(kopter,TELEMETRIES.THROTTLE_SETPOINT);
        dataValues[15] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_POS_SETPOINT);
        dataValues[16] = getTelemetry(kopter,TELEMETRIES.AILERON_POS_SETPOINT);
        dataValues[17] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_VEL_SETPOINT);
        dataValues[18] = getTelemetry(kopter,TELEMETRIES.AILERON_VEL_SETPOINT);
        dataValues[19] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_SPEED_ESTIMATED2);
        dataValues[20] = getTelemetry(kopter,TELEMETRIES.AILERON_SPEED_ESTIMATED2);
        dataValues[21] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_ACC);
        dataValues[22] = getTelemetry(kopter,TELEMETRIES.AILERON_ACC);
        dataValues[23] = getTelemetry(kopter,TELEMETRIES.VALID_GUMSTIX);
        dataValues[24] = getTelemetry(kopter,TELEMETRIES.ELEVATOR_DESIRED_SPEED_POS_CONT);
        dataValues[25] = getTelemetry(kopter,TELEMETRIES.AILERON_DESIRED_SPEED_POS_CONT);
        dataValues[26] = getTelemetry(kopter,TELEMETRIES.ELE_DES_SPEED_POS_CONT_LEADER);
        dataValues[27] = getTelemetry(kopter,TELEMETRIES.AIL_DES_SPEED_POS_CONT_LEADER);
        dataValues[28] = getTelemetry(kopter,TELEMETRIES.OUTPUT_THROTTLE);
        dataValues[29] = getTelemetry(kopter,TELEMETRIES.OUTPUT_ELEVATOR);
        dataValues[30] = getTelemetry(kopter,TELEMETRIES.OUTPUT_AILERON);
        dataValues[31] = getTelemetry(kopter,TELEMETRIES.OUTPUT_RUDDER);
        dataValues[32] = getTelemetry(kopter,TELEMETRIES.BLOB_DISTANCE);
        dataValues[33] = getTelemetry(kopter,TELEMETRIES.BLOB_HORIZONTAL);
        dataValues[34] = getTelemetry(kopter,TELEMETRIES.BLOB_VERTICAL);
        dataValues[35] = getTelemetry(kopter,TELEMETRIES.PITCH_ANGLE);
        dataValues[36] = getTelemetry(kopter,TELEMETRIES.ROLL_ANGLE);

        time_t     now = time(0);
        struct tm  tstruct;
        char TMP[80];
        tstruct = *localtime(&now);
        strftime(TMP, sizeof(TMP), "%X", &tstruct);
        ofstream outputFile(myTime,ios::out | ios::app);
        outputFile << TMP;
        outputFile << ",";
        for(int i=0;i<37;i++){
            outputFile << dataValues[i];
            outputFile << ",";
        }
        outputFile << "\n";
        outputFile.close();

        if(Plot1Signals[0]->isChecked() || Plot1Signals[1]->isChecked() || Plot1Signals[2]->isChecked() || Plot1Signals[3]->isChecked() || Plot1Signals[4]->isChecked()
                || Plot1Signals[5]->isChecked() || Plot1Signals[6]->isChecked() || Plot1Signals[7]->isChecked() || Plot1Signals[8]->isChecked() || Plot1Signals[9]->isChecked()
                || Plot1Signals[10]->isChecked() || Plot1Signals[11]->isChecked() || Plot1Signals[12]->isChecked() || Plot1Signals[13]->isChecked() || Plot1Signals[14]->isChecked()
                || Plot1Signals[15]->isChecked() || Plot1Signals[16]->isChecked() || Plot1Signals[17]->isChecked() || Plot1Signals[18]->isChecked() || Plot1Signals[19]->isChecked()
                || Plot1Signals[20]->isChecked() || Plot1Signals[21]->isChecked() || Plot1Signals[22]->isChecked() || Plot1Signals[23]->isChecked() || Plot1Signals[24]->isChecked()
                || Plot1Signals[25]->isChecked() || Plot1Signals[26]->isChecked() || Plot1Signals[27]->isChecked() || Plot1Signals[28]->isChecked() || Plot1Signals[29]->isChecked()
                || Plot1Signals[30]->isChecked() || Plot1Signals[31]->isChecked() || Plot1Signals[32]->isChecked() || Plot1Signals[33]->isChecked() || Plot1Signals[34]->isChecked()
                || Plot1Signals[35]->isChecked() || Plot1Signals[36]->isChecked()){

            for(int i=0;i<37;i++){
                if(Plot1Signals[i]->isChecked()){
                    ui->graph1->graph(i)->addData(key1, dataValues[i]);
                    ui->graph1->graph(i)->rescaleValueAxis();
                }
            }
            // remove data of lines that's outside visible range:
            for(int i=0;i<37;i++){
                ui->graph1->graph(i)->removeDataBefore(key1-10);
            }
        }
        lastPointKey1 = key1;
    }
    ui->graph1->xAxis->setRange(key1+0.25, 10, Qt::AlignRight);
    ui->graph1->replot();

    // calculate frames per second:
    // static double lastFpskey;
    // static int frameCount;
    //  ++frameCount;
    /*  if (key-lastFpskey > 2) // average fps over 2 seconds
  {
    ui->statusBar->showMessage(
          QString("%1 FPS, Total Data points: %2")
          .arg(frameCount/(key-lastFpskey), 0, 'f', 0)
          .arg(ui->graph1->graph(0)->data()->count()+ui->graph1->graph(1)->data()->count())
          , 0);
    lastFpskey = key;
    frameCount = 0;
  }*/
    // delay
    //   if(ui->Quadro1Signal1->isChecked()||ui->Quadro1Signal2->isChecked()||ui->Quadro1Signal3->isChecked()||ui->Quadro1Signal4->isChecked()
    //       ||ui->Quadro1Signal5->isChecked()||ui->Quadro1Signal6->isChecked()||ui->Quadro1Signal7->isChecked()||ui->Quadro1Signal8->isChecked()){
    //   }

}

void quadro::graph_2(QCustomPlot *graph2)
{
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
    QMessageBox::critical(this, "", "You're using Qt < 4.7, the realtime data needs functions that are available with Qt 4.7 to work properly");
#endif



    graph2->addGraph(); // blue line
    graph2->graph(0)->setPen(QPen(Qt::blue));
    graph2->graph(0)->setName("Signal 1");
    //graph2->graph(0)->setBrush(QBrush(QColor(240, 255, 200)));      // vybarveni od modre cary
    // graph2->graph(0)->setAntialiasedFill(false);
    graph2->addGraph(); // red line
    graph2->graph(1)->setPen(QPen(Qt::red));
    graph2->graph(1)->setName("Signal 2");
    // graph2->graph(0)->setChannelFillGraph(graph2->graph(1));    // upresneni ze od modre k cervene
    graph2->addGraph();
    graph2->graph(2)->setPen(QPen(Qt::green));
    graph2->graph(2)->setName("Signal 3");
    graph2->addGraph();
    graph2->graph(3)->setPen(QPen(Qt::black));
    graph2->graph(3)->setName("Signal 4");
    graph2->addGraph();
    graph2->graph(4)->setPen(QPen(Qt::cyan));
    graph2->graph(4)->setName("Signal 5");
    graph2->addGraph();
    graph2->graph(5)->setPen(QPen(Qt::gray));
    graph2->graph(5)->setName("Signal 6");
    graph2->addGraph();
    graph2->graph(6)->setPen(QPen(Qt::yellow));
    graph2->graph(6)->setName("Signal 7");
    graph2->addGraph();
    graph2->graph(7)->setPen(QPen(Qt::magenta));
    graph2->graph(7)->setName("Signal 8");
    graph2->addGraph();
    graph2->graph(8)->setPen(QPen(Qt::darkBlue));
    graph2->graph(8)->setName("Signal 9");
    graph2->addGraph();
    graph2->graph(9)->setPen(QPen(Qt::darkRed));
    graph2->graph(9)->setName("Signal 10");
    graph2->addGraph();
    graph2->graph(10)->setPen(QPen(Qt::darkGreen));
    graph2->graph(10)->setName("Signal 11");
    graph2->addGraph();
    graph2->graph(11)->setPen(QPen(Qt::darkCyan));
    graph2->graph(11)->setName("Signal 12");
    graph2->addGraph();
    graph2->graph(12)->setPen(QPen(Qt::darkGray));
    graph2->graph(12)->setName("Signal 13");
    graph2->addGraph();
    graph2->graph(13)->setPen(QPen(Qt::darkYellow));
    graph2->graph(13)->setName("Signal 14");
    graph2->addGraph();
    graph2->graph(14)->setPen(QPen(Qt::darkMagenta));
    graph2->graph(14)->setName("Signal 15");
    graph2->addGraph();
    //
    graph2->graph(15)->setPen(QPen(QColor(85,0,130)));//fialová
    graph2->graph(15)->setName("Signal 16");
    graph2->addGraph();
    graph2->graph(16)->setPen(QPen(QColor(255,170,255)));//růžová
    graph2->graph(16)->setName("Signal 17");
    graph2->addGraph();
    graph2->graph(17)->setPen(QPen(QColor(0,255,130)));//svetle zelená
    graph2->graph(17)->setName("Signal 18");
    graph2->addGraph();
    graph2->graph(18)->setPen(QPen(QColor(170,170,255)));//svetle fialová
    graph2->graph(18)->setName("Signal 19");
    graph2->addGraph();
    graph2->graph(19)->setPen(QPen(QColor(255,130,0)));//oranzová
    graph2->graph(19)->setName("Signal 20");
    graph2->addGraph();
    graph2->graph(20)->setPen(QPen(QColor(80,25,0)));//hnědá
    graph2->graph(20)->setName("Signal 21");
    graph2->addGraph();
    graph2->graph(21)->setPen(QPen(QColor(100,0,80)));//vínová
    graph2->graph(21)->setName("Signal 22");
    graph2->addGraph();
    graph2->graph(22)->setPen(QPen(QColor(170,255,0)));// zarive zelená
    graph2->graph(22)->setName("Signal 23");
    graph2->addGraph();
    graph2->graph(23)->setPen(QPen(QColor(255,170,110)));//vybledlá oranžová
    graph2->graph(23)->setName("Signal 24");
    graph2->addGraph();
    graph2->graph(24)->setPen(QPen(QColor(175,90,0)));//tmavě oranžová
    graph2->graph(24)->setName("Signal 25");
    graph2->addGraph();
    graph2->graph(25)->setPen(QPen(QColor(80,100,120)));//vybledlá modrá
    graph2->graph(25)->setName("Signal 26");
    graph2->addGraph();
    graph2->graph(26)->setPen(QPen(QColor(170,60,110)));//tmavě růžová
    graph2->graph(26)->setName("Signal 27");
    graph2->addGraph();
    graph2->graph(27)->setPen(QPen(QColor(85,85,0)));//tmavá žlutá
    graph2->graph(27)->setName("Signal 28");
    graph2->addGraph();
    graph2->graph(28)->setPen(QPen(QColor(80,70,70)));// ? ?
    graph2->graph(28)->setName("Signal 29");
    graph2->addGraph();
    graph2->graph(29)->setPen(QPen(QColor(10,100,80)));//zelená
    graph2->graph(29)->setName("Signal 30");
    graph2->addGraph();
    graph2->graph(30)->setPen(QPen(QColor(30,110,130)));//tmavě tyrkysová
    graph2->graph(30)->setName("Signal 31");
    graph2->addGraph();
    graph2->graph(31)->setPen(QPen(QColor(180,180,180)));//stříbrná
    graph2->graph(31)->setName("Signal 32");

    graph2->addGraph();
    graph2->graph(32)->setPen(QPen(Qt::red));//
    graph2->graph(32)->setName("Signal 33");
    graph2->addGraph();
    graph2->graph(33)->setPen(QPen(Qt::blue));//
    graph2->graph(33)->setName("Signal 34");
    graph2->addGraph();
    graph2->graph(34)->setPen(QPen(Qt::green));//
    graph2->graph(34)->setName("Signal 35");
    graph2->addGraph();
    graph2->graph(35)->setPen(QPen(Qt::magenta));//
    graph2->graph(35)->setName("Signal 36");
    graph2->addGraph();
    graph2->graph(36)->setPen(QPen(Qt::gray));//
    graph2->graph(36)->setName("Signal 37");
    // pro osu se vzorkama
    graph2->xAxis->setAutoTickStep(false);
    graph2->xAxis->setTickLabelType(QCPAxis::ltDateTime);
    graph2->xAxis->setDateTimeFormat("hh:mm:ss");
    graph2->xAxis->setAutoTickStep(false);
    graph2->xAxis->setTickStep(2);
    graph2->axisRect()->setupFullAxesBox();

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(graph2->xAxis, SIGNAL(rangeChanged(QCPRange)), graph2->xAxis2, SLOT(setRange(QCPRange)));
    connect(graph2->yAxis, SIGNAL(rangeChanged(QCPRange)), graph2->yAxis2, SLOT(setRange(QCPRange)));

    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlotGraph2()));
    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void quadro::realtimeDataSlotGraph2()
{
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
    double key2 = 0;
#else
    double key2 = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
#endif
    static double lastPointKey2 = 0;
    if (key2-lastPointKey2 > 0.01) // at most add point every 10 ms
    {
        if(Plot2Signals[0]->isChecked() || Plot2Signals[1]->isChecked() || Plot2Signals[2]->isChecked() || Plot2Signals[3]->isChecked() || Plot2Signals[4]->isChecked()
                || Plot2Signals[5]->isChecked() || Plot2Signals[6]->isChecked() || Plot2Signals[7]->isChecked() || Plot2Signals[8]->isChecked() || Plot2Signals[9]->isChecked()
                || Plot2Signals[10]->isChecked() || Plot2Signals[11]->isChecked() || Plot2Signals[12]->isChecked() || Plot2Signals[13]->isChecked() || Plot2Signals[14]->isChecked()
                || Plot2Signals[15]->isChecked() || Plot2Signals[16]->isChecked() || Plot2Signals[17]->isChecked() || Plot2Signals[18]->isChecked() || Plot2Signals[19]->isChecked()
                || Plot2Signals[20]->isChecked() || Plot2Signals[21]->isChecked() || Plot2Signals[22]->isChecked() || Plot2Signals[23]->isChecked() || Plot2Signals[24]->isChecked()
                || Plot2Signals[25]->isChecked() || Plot2Signals[26]->isChecked() || Plot2Signals[27]->isChecked() || Plot2Signals[28]->isChecked() || Plot2Signals[29]->isChecked()
                || Plot2Signals[30]->isChecked() || Plot2Signals[31]->isChecked() || Plot2Signals[32]->isChecked() || Plot2Signals[33]->isChecked() || Plot2Signals[34]->isChecked()
                || Plot2Signals[35]->isChecked() || Plot2Signals[36]->isChecked()){

            for(int i=0;i<37;i++){
                if(Plot2Signals[i]->isChecked()){
                    ui->graph2->graph(i)->addData(key2, dataValues[i]);
                    ui->graph2->graph(i)->rescaleValueAxis();
                }
            }

            for(int i=0;i<37;i++){
                ui->graph2->graph(i)->removeDataBefore(key2-10);
            }


            // make key axis range scroll with the data (at a constant range size of 8):
        }
        lastPointKey2 = key2;

    }
    ui->graph2->xAxis->setRange(key2+0.25, 10, Qt::AlignRight);
    ui->graph2->replot();
}

void quadro::graph_3(QCustomPlot *graph3)
{
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
    QMessageBox::critical(this, "", "You're using Qt < 4.7, the realtime data needs functions that are available with Qt 4.7 to work properly");
#endif

    graph3->addGraph(); // blue line
    graph3->graph(0)->setPen(QPen(Qt::blue));
    graph3->graph(0)->setName("Signal 1");
    //graph3->graph(0)->setBrush(QBrush(QColor(240, 255, 200)));      // vybarveni od modre cary
    // graph3->graph(0)->setAntialiasedFill(false);
    graph3->addGraph(); // red line
    graph3->graph(1)->setPen(QPen(Qt::red));
    graph3->graph(1)->setName("Signal 2");
    // graph3->graph(0)->setChannelFillGraph(graph3->graph(1));    // upresneni ze od modre k cervene
    graph3->addGraph();
    graph3->graph(2)->setPen(QPen(Qt::green));
    graph3->graph(2)->setName("Signal 3");
    graph3->addGraph();
    graph3->graph(3)->setPen(QPen(Qt::black));
    graph3->graph(3)->setName("Signal 4");
    graph3->addGraph();
    graph3->graph(4)->setPen(QPen(Qt::cyan));
    graph3->graph(4)->setName("Signal 5");
    graph3->addGraph();
    graph3->graph(5)->setPen(QPen(Qt::gray));
    graph3->graph(5)->setName("Signal 6");
    graph3->addGraph();
    graph3->graph(6)->setPen(QPen(Qt::yellow));
    graph3->graph(6)->setName("Signal 7");
    graph3->addGraph();
    graph3->graph(7)->setPen(QPen(Qt::magenta));
    graph3->graph(7)->setName("Signal 8");
    graph3->addGraph();
    graph3->graph(8)->setPen(QPen(Qt::darkBlue));
    graph3->graph(8)->setName("Signal 9");
    graph3->addGraph();
    graph3->graph(9)->setPen(QPen(Qt::darkRed));
    graph3->graph(9)->setName("Signal 10");
    graph3->addGraph();
    graph3->graph(10)->setPen(QPen(Qt::darkGreen));
    graph3->graph(10)->setName("Signal 11");
    graph3->addGraph();
    graph3->graph(11)->setPen(QPen(Qt::darkCyan));
    graph3->graph(11)->setName("Signal 12");
    graph3->addGraph();
    graph3->graph(12)->setPen(QPen(Qt::darkGray));
    graph3->graph(12)->setName("Signal 13");
    graph3->addGraph();
    graph3->graph(13)->setPen(QPen(Qt::darkYellow));
    graph3->graph(13)->setName("Signal 14");
    graph3->addGraph();
    graph3->graph(14)->setPen(QPen(Qt::darkMagenta));
    graph3->graph(14)->setName("Signal 15");
    graph3->addGraph();
    //
    graph3->graph(15)->setPen(QPen(QColor(85,0,130)));//fialová
    graph3->graph(15)->setName("Signal 16");
    graph3->addGraph();
    graph3->graph(16)->setPen(QPen(QColor(255,170,255)));//růžová
    graph3->graph(16)->setName("Signal 17");
    graph3->addGraph();
    graph3->graph(17)->setPen(QPen(QColor(0,255,130)));//svetle zelená
    graph3->graph(17)->setName("Signal 18");
    graph3->addGraph();
    graph3->graph(18)->setPen(QPen(QColor(170,170,255)));//svetle fialová
    graph3->graph(18)->setName("Signal 19");
    graph3->addGraph();
    graph3->graph(19)->setPen(QPen(QColor(255,130,0)));//oranzová
    graph3->graph(19)->setName("Signal 20");
    graph3->addGraph();
    graph3->graph(20)->setPen(QPen(QColor(80,25,0)));//hnědá
    graph3->graph(20)->setName("Signal 21");
    graph3->addGraph();
    graph3->graph(21)->setPen(QPen(QColor(100,0,80)));//vínová
    graph3->graph(21)->setName("Signal 22");
    graph3->addGraph();
    graph3->graph(22)->setPen(QPen(QColor(170,255,0)));// zarive zelená
    graph3->graph(22)->setName("Signal 23");
    graph3->addGraph();
    graph3->graph(23)->setPen(QPen(QColor(255,170,110)));//vybledlá oranžová
    graph3->graph(23)->setName("Signal 24");
    graph3->addGraph();
    graph3->graph(24)->setPen(QPen(QColor(175,90,0)));//tmavě oranžová
    graph3->graph(24)->setName("Signal 25");
    graph3->addGraph();
    graph3->graph(25)->setPen(QPen(QColor(80,100,120)));//vybledlá modrá
    graph3->graph(25)->setName("Signal 26");
    graph3->addGraph();
    graph3->graph(26)->setPen(QPen(QColor(170,60,110)));//tmavě růžová
    graph3->graph(26)->setName("Signal 27");
    graph3->addGraph();
    graph3->graph(27)->setPen(QPen(QColor(85,85,0)));//tmavá žlutá
    graph3->graph(27)->setName("Signal 28");
    graph3->addGraph();
    graph3->graph(28)->setPen(QPen(QColor(80,70,70)));// ? ?
    graph3->graph(28)->setName("Signal 29");
    graph3->addGraph();
    graph3->graph(29)->setPen(QPen(QColor(10,100,80)));//zelená
    graph3->graph(29)->setName("Signal 30");
    graph3->addGraph();
    graph3->graph(30)->setPen(QPen(QColor(30,110,130)));//tmavě tyrkysová
    graph3->graph(30)->setName("Signal 31");
    graph3->addGraph();
    graph3->graph(31)->setPen(QPen(QColor(180,180,180)));//stříbrná
    graph3->graph(31)->setName("Signal 32");

    graph3->addGraph();
    graph3->graph(32)->setPen(QPen(Qt::red));//
    graph3->graph(32)->setName("Signal 33");
    graph3->addGraph();
    graph3->graph(33)->setPen(QPen(Qt::blue));//
    graph3->graph(33)->setName("Signal 34");
    graph3->addGraph();
    graph3->graph(34)->setPen(QPen(Qt::green));//
    graph3->graph(34)->setName("Signal 35");
    graph3->addGraph();
    graph3->graph(35)->setPen(QPen(Qt::magenta));//
    graph3->graph(35)->setName("Signal 36");
    graph3->addGraph();
    graph3->graph(36)->setPen(QPen(Qt::gray));//
    graph3->graph(36)->setName("Signal 37");

    graph3->xAxis->setTickLabelType(QCPAxis::ltDateTime);       // pro casovou osu
    graph3->xAxis->setDateTimeFormat("hh:mm:ss");
    graph3->xAxis->setAutoTickStep(false);
    graph3->xAxis->setTickStep(2);

    graph3->axisRect()->setupFullAxesBox();

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(graph3->xAxis, SIGNAL(rangeChanged(QCPRange)), graph3->xAxis2, SLOT(setRange(QCPRange)));
    connect(graph3->yAxis, SIGNAL(rangeChanged(QCPRange)), graph3->yAxis2, SLOT(setRange(QCPRange)));

    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlotGraph3()));
    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void quadro::realtimeDataSlotGraph3()
{
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
    double key3 = 0;
#else
    double key3 = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
#endif
    static double lastPointKey3 = 0;
    if (key3-lastPointKey3 > 0.01) // at most add point every 10 ms
    {
        if(Plot3Signals[0]->isChecked() || Plot3Signals[1]->isChecked() || Plot3Signals[2]->isChecked() || Plot3Signals[3]->isChecked() || Plot3Signals[4]->isChecked()
                || Plot3Signals[5]->isChecked() || Plot3Signals[6]->isChecked() || Plot3Signals[7]->isChecked() || Plot3Signals[8]->isChecked() || Plot3Signals[9]->isChecked()
                || Plot3Signals[10]->isChecked() || Plot3Signals[11]->isChecked() || Plot3Signals[12]->isChecked() || Plot3Signals[13]->isChecked() || Plot3Signals[14]->isChecked()
                || Plot3Signals[15]->isChecked() || Plot3Signals[16]->isChecked() || Plot3Signals[17]->isChecked() || Plot3Signals[18]->isChecked() || Plot3Signals[19]->isChecked()
                || Plot3Signals[20]->isChecked() || Plot3Signals[21]->isChecked() || Plot3Signals[22]->isChecked() || Plot3Signals[23]->isChecked() || Plot3Signals[24]->isChecked()
                || Plot3Signals[25]->isChecked() || Plot3Signals[26]->isChecked() || Plot3Signals[27]->isChecked() || Plot3Signals[28]->isChecked() || Plot3Signals[29]->isChecked()
                || Plot3Signals[30]->isChecked() || Plot3Signals[31]->isChecked() || Plot3Signals[32]->isChecked() || Plot3Signals[33]->isChecked() || Plot3Signals[34]->isChecked()
                || Plot3Signals[35]->isChecked() || Plot3Signals[36]->isChecked()){

            for(int i=0;i<37;i++){
                if(Plot3Signals[i]->isChecked()){
                    ui->graph3->graph(i)->addData(key3, dataValues[i]);
                    ui->graph3->graph(i)->rescaleValueAxis();
                }
            }
            // remove data of lines that's outside visible range:
            for(int i=0;i<37;i++){
                ui->graph3->graph(i)->removeDataBefore(key3-10);
            }
            // make key axis range scroll with the data (at a constant range size of 8):
        }
        lastPointKey3 = key3;
    }
    ui->graph3->xAxis->setRange(key3+0.25, 10, Qt::AlignRight);
    ui->graph3->replot();
}

void quadro::graph_4(QCustomPlot *graph4)
{
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
    QMessageBox::critical(this, "", "You're using Qt < 4.7, the realtime data needs functions that are available with Qt 4.7 to work properly");
#endif


    graph4->addGraph(); // blue line
    graph4->graph(0)->setPen(QPen(Qt::blue));
    graph4->graph(0)->setName("Signal 1");
    //graph4->graph(0)->setBrush(QBrush(QColor(240, 255, 200)));      // vybarveni od modre cary
    // graph4->graph(0)->setAntialiasedFill(false);
    graph4->addGraph(); // red line
    graph4->graph(1)->setPen(QPen(Qt::red));
    graph4->graph(1)->setName("Signal 2");
    // graph4->graph(0)->setChannelFillGraph(graph4->graph(1));    // upresneni ze od modre k cervene
    graph4->addGraph();
    graph4->graph(2)->setPen(QPen(Qt::green));
    graph4->graph(2)->setName("Signal 3");
    graph4->addGraph();
    graph4->graph(3)->setPen(QPen(Qt::black));
    graph4->graph(3)->setName("Signal 4");
    graph4->addGraph();
    graph4->graph(4)->setPen(QPen(Qt::cyan));
    graph4->graph(4)->setName("Signal 5");
    graph4->addGraph();
    graph4->graph(5)->setPen(QPen(Qt::gray));
    graph4->graph(5)->setName("Signal 6");
    graph4->addGraph();
    graph4->graph(6)->setPen(QPen(Qt::yellow));
    graph4->graph(6)->setName("Signal 7");
    graph4->addGraph();
    graph4->graph(7)->setPen(QPen(Qt::magenta));
    graph4->graph(7)->setName("Signal 8");
    graph4->addGraph();
    graph4->graph(8)->setPen(QPen(Qt::darkBlue));
    graph4->graph(8)->setName("Signal 9");
    graph4->addGraph();
    graph4->graph(9)->setPen(QPen(Qt::darkRed));
    graph4->graph(9)->setName("Signal 10");
    graph4->addGraph();
    graph4->graph(10)->setPen(QPen(Qt::darkGreen));
    graph4->graph(10)->setName("Signal 11");
    graph4->addGraph();
    graph4->graph(11)->setPen(QPen(Qt::darkCyan));
    graph4->graph(11)->setName("Signal 12");
    graph4->addGraph();
    graph4->graph(12)->setPen(QPen(Qt::darkGray));
    graph4->graph(12)->setName("Signal 13");
    graph4->addGraph();
    graph4->graph(13)->setPen(QPen(Qt::darkYellow));
    graph4->graph(13)->setName("Signal 14");
    graph4->addGraph();
    graph4->graph(14)->setPen(QPen(Qt::darkMagenta));
    graph4->graph(14)->setName("Signal 15");
    graph4->addGraph();
    //
    graph4->graph(15)->setPen(QPen(QColor(85,0,130)));//fialová
    graph4->graph(15)->setName("Signal 16");
    graph4->addGraph();
    graph4->graph(16)->setPen(QPen(QColor(255,170,255)));//růžová
    graph4->graph(16)->setName("Signal 17");
    graph4->addGraph();
    graph4->graph(17)->setPen(QPen(QColor(0,255,130)));//svetle zelená
    graph4->graph(17)->setName("Signal 18");
    graph4->addGraph();
    graph4->graph(18)->setPen(QPen(QColor(170,170,255)));//svetle fialová
    graph4->graph(18)->setName("Signal 19");
    graph4->addGraph();
    graph4->graph(19)->setPen(QPen(QColor(255,130,0)));//oranzová
    graph4->graph(19)->setName("Signal 20");
    graph4->addGraph();
    graph4->graph(20)->setPen(QPen(QColor(80,25,0)));//hnědá
    graph4->graph(20)->setName("Signal 21");
    graph4->addGraph();
    graph4->graph(21)->setPen(QPen(QColor(100,0,80)));//vínová
    graph4->graph(21)->setName("Signal 22");
    graph4->addGraph();
    graph4->graph(22)->setPen(QPen(QColor(170,255,0)));// zarive zelená
    graph4->graph(22)->setName("Signal 23");
    graph4->addGraph();
    graph4->graph(23)->setPen(QPen(QColor(255,170,110)));//vybledlá oranžová
    graph4->graph(23)->setName("Signal 24");
    graph4->addGraph();
    graph4->graph(24)->setPen(QPen(QColor(175,90,0)));//tmavě oranžová
    graph4->graph(24)->setName("Signal 25");
    graph4->addGraph();
    graph4->graph(25)->setPen(QPen(QColor(80,100,120)));//vybledlá modrá
    graph4->graph(25)->setName("Signal 26");
    graph4->addGraph();
    graph4->graph(26)->setPen(QPen(QColor(170,60,110)));//tmavě růžová
    graph4->graph(26)->setName("Signal 27");
    graph4->addGraph();
    graph4->graph(27)->setPen(QPen(QColor(85,85,0)));//tmavá žlutá
    graph4->graph(27)->setName("Signal 28");
    graph4->addGraph();
    graph4->graph(28)->setPen(QPen(QColor(80,70,70)));// ? ?
    graph4->graph(28)->setName("Signal 29");
    graph4->addGraph();
    graph4->graph(29)->setPen(QPen(QColor(10,100,80)));//zelená
    graph4->graph(29)->setName("Signal 30");
    graph4->addGraph();
    graph4->graph(30)->setPen(QPen(QColor(30,110,130)));//tmavě tyrkysová
    graph4->graph(30)->setName("Signal 31");
    graph4->addGraph();
    graph4->graph(31)->setPen(QPen(QColor(180,180,180)));//stříbrná
    graph4->graph(31)->setName("Signal 32");

    graph4->addGraph();
    graph4->graph(32)->setPen(QPen(Qt::red));//
    graph4->graph(32)->setName("Signal 33");
    graph4->addGraph();
    graph4->graph(33)->setPen(QPen(Qt::blue));//
    graph4->graph(33)->setName("Signal 34");
    graph4->addGraph();
    graph4->graph(34)->setPen(QPen(Qt::green));//
    graph4->graph(34)->setName("Signal 35");
    graph4->addGraph();
    graph4->graph(35)->setPen(QPen(Qt::magenta));//
    graph4->graph(35)->setName("Signal 36");
    graph4->addGraph();
    graph4->graph(36)->setPen(QPen(Qt::gray));//
    graph4->graph(36)->setName("Signal 37");

    graph4->xAxis->setTickLabelType(QCPAxis::ltDateTime);       // pro casovou osu
    graph4->xAxis->setDateTimeFormat("hh:mm:ss");
    graph4->xAxis->setAutoTickStep(false);
    graph4->xAxis->setTickStep(2);

    graph4->axisRect()->setupFullAxesBox();

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(graph4->xAxis, SIGNAL(rangeChanged(QCPRange)), graph4->xAxis2, SLOT(setRange(QCPRange)));
    connect(graph4->yAxis, SIGNAL(rangeChanged(QCPRange)), graph4->yAxis2, SLOT(setRange(QCPRange)));

    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlotGraph4()));
    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void quadro::realtimeDataSlotGraph4()
{
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
    double key4 = 0;
#else
    double key4 = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
#endif
    static double lastPointKey4 = 0;
    if (key4-lastPointKey4 > 0.01) // at most add point every 10 ms
    {
        if(Plot4Signals[0]->isChecked() || Plot4Signals[1]->isChecked() || Plot4Signals[2]->isChecked() || Plot4Signals[3]->isChecked() || Plot4Signals[4]->isChecked()
                || Plot4Signals[5]->isChecked() || Plot4Signals[6]->isChecked() || Plot4Signals[7]->isChecked() || Plot4Signals[8]->isChecked() || Plot4Signals[9]->isChecked()
                || Plot4Signals[10]->isChecked() || Plot4Signals[11]->isChecked() || Plot4Signals[12]->isChecked() || Plot4Signals[13]->isChecked() || Plot4Signals[14]->isChecked()
                || Plot4Signals[15]->isChecked() || Plot4Signals[16]->isChecked() || Plot4Signals[17]->isChecked() || Plot4Signals[18]->isChecked() || Plot4Signals[19]->isChecked()
                || Plot4Signals[20]->isChecked() || Plot4Signals[21]->isChecked() || Plot4Signals[22]->isChecked() || Plot4Signals[23]->isChecked() || Plot4Signals[24]->isChecked()
                || Plot4Signals[25]->isChecked() || Plot4Signals[26]->isChecked() || Plot4Signals[27]->isChecked() || Plot4Signals[28]->isChecked() || Plot4Signals[29]->isChecked()
                || Plot4Signals[30]->isChecked() || Plot4Signals[31]->isChecked() || Plot4Signals[32]->isChecked() || Plot4Signals[33]->isChecked() || Plot4Signals[34]->isChecked()
                || Plot4Signals[35]->isChecked() || Plot4Signals[36]->isChecked()){

            for(int i=0;i<37;i++){
                if(Plot4Signals[i]->isChecked()){
                    ui->graph4->graph(i)->addData(key4, dataValues[i]);
                    ui->graph4->graph(i)->rescaleValueAxis();
                }
            }
            // remove data of lines that's outside visible range:
            for(int i=0;i<37;i++){
                ui->graph4->graph(i)->removeDataBefore(key4-10);
            }
            // make key axis range scroll with the data (at a constant range size of 8):
        }
        lastPointKey4 = key4;
    }
    ui->graph4->xAxis->setRange(key4+0.25, 10, Qt::AlignRight);
    ui->graph4->replot();
}

void quadro::createScrollCheckBox()
{
    QVBoxLayout *checkLayout = new QVBoxLayout();
    Plot1Signals[0] = new QCheckBox("Ground distance estimated");
    connect(Plot1Signals[0],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[1] = new QCheckBox("Ground distance");
    connect(Plot1Signals[1],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[2] = new QCheckBox("Elevator speed");
    connect(Plot1Signals[2],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[3] = new QCheckBox("Aileron_speed");
    connect(Plot1Signals[3],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[4] = new QCheckBox("Elevator speed estimated");
    connect(Plot1Signals[4],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[5] = new QCheckBox("Aileron speed estimated");
    connect(Plot1Signals[5],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[6] = new QCheckBox("Elevator position estimated");
    connect(Plot1Signals[6],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[7] = new QCheckBox("Aileron position estimated");
    connect(Plot1Signals[7],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[8] = new QCheckBox("Throttle controller output");
    connect(Plot1Signals[8],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[9] = new QCheckBox("Throttle speed");
    connect(Plot1Signals[9],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[10] = new QCheckBox("Aileron velocity controller output");
    connect(Plot1Signals[10],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[11] = new QCheckBox("Elevator velocity controller output");
    connect(Plot1Signals[11],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[12] = new QCheckBox("Aileron position controller output");
    connect(Plot1Signals[12],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[13] = new QCheckBox("Elevator position controller output");
    connect(Plot1Signals[13],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[14] = new QCheckBox("Throttle setpoint");
    connect(Plot1Signals[14],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[15] = new QCheckBox("Elevator position setpoint");
    connect(Plot1Signals[15],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[16] = new QCheckBox("Aileron position setpoint");
    connect(Plot1Signals[16],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[17] = new QCheckBox("Elevator velocity setpoint");
    connect(Plot1Signals[17],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[18] = new QCheckBox("Aileron velocity setpoint");
    connect(Plot1Signals[18],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[19] = new QCheckBox("Elevator speed estimated 2");
    connect(Plot1Signals[19],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[20] = new QCheckBox("Aileron speed estimated 2");
    connect(Plot1Signals[20],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[21] = new QCheckBox("Elevator ACC");
    connect(Plot1Signals[21],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[22] = new QCheckBox("Aileron ACC");
    connect(Plot1Signals[22],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[23] = new QCheckBox("Valid gumstix");
    connect(Plot1Signals[23],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[24] = new QCheckBox("Elevator desired speed position cont");
    connect(Plot1Signals[24],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[25] = new QCheckBox("Aileron desired speed position cont");
    connect(Plot1Signals[25],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[26] = new QCheckBox("Elevator desired speed pos cont leader");
    connect(Plot1Signals[26],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[27] = new QCheckBox("Aileron desired speed pos cont leader");
    connect(Plot1Signals[27],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[28] = new QCheckBox("Output throttle");
    connect(Plot1Signals[28],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[29] = new QCheckBox("Output elevator");
    connect(Plot1Signals[29],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[30] = new QCheckBox("Output aileron");
    connect(Plot1Signals[30],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[31] = new QCheckBox("Output rudder");
    connect(Plot1Signals[31],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[32] = new QCheckBox("Blob Distance");
    connect(Plot1Signals[32],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[33] = new QCheckBox("Blob Horizontal");
    connect(Plot1Signals[33],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[34] = new QCheckBox("Blob Vertical");
    connect(Plot1Signals[34],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[35] = new QCheckBox("Pitch Angle");
    connect(Plot1Signals[35],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[36] = new QCheckBox("Roll Angle");
    connect(Plot1Signals[36],SIGNAL(clicked()),this,SLOT(Plot1Signals_clicked()));
    Plot1Signals[37] = new QCheckBox("All signals");
    connect(Plot1Signals[37],SIGNAL(clicked()),this,SLOT(Plot1SignalAll_clicked()));

    Plot2Signals[0] = new QCheckBox("Ground distance estimated");
    connect(Plot2Signals[0],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[1] = new QCheckBox("Ground distance");
    connect(Plot2Signals[1],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[2] = new QCheckBox("Elevator speed");
    connect(Plot2Signals[2],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[3] = new QCheckBox("Aileron_speed");
    connect(Plot2Signals[3],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[4] = new QCheckBox("Elevator speed estimated");
    connect(Plot2Signals[4],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[5] = new QCheckBox("Aileron speed estimated");
    connect(Plot2Signals[5],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[6] = new QCheckBox("Elevator position estimated");
    connect(Plot2Signals[6],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[7] = new QCheckBox("Aileron position estimated");
    connect(Plot2Signals[7],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[8] = new QCheckBox("Throttle controller output");
    connect(Plot2Signals[8],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[9] = new QCheckBox("Throttle speed");
    connect(Plot2Signals[9],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[10] = new QCheckBox("Aileron velocity controller output");
    connect(Plot2Signals[10],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[11] = new QCheckBox("Elevator velocity controller output");
    connect(Plot2Signals[11],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[12] = new QCheckBox("Aileron position controller output");
    connect(Plot2Signals[12],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[13] = new QCheckBox("Elevator position controller output");
    connect(Plot2Signals[13],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[14] = new QCheckBox("Throttle setpoint");
    connect(Plot2Signals[14],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[15] = new QCheckBox("Elevator position setpoint");
    connect(Plot2Signals[15],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[16] = new QCheckBox("Aileron position setpoint");
    connect(Plot2Signals[16],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[17] = new QCheckBox("Elevator velocity setpoint");
    connect(Plot2Signals[17],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[18] = new QCheckBox("Aileron velocity setpoint");
    connect(Plot2Signals[18],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[19] = new QCheckBox("Elevator speed estimated 2");
    connect(Plot2Signals[19],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[20] = new QCheckBox("Aileron speed estimated 2");
    connect(Plot2Signals[20],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[21] = new QCheckBox("Elevator ACC");
    connect(Plot2Signals[21],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[22] = new QCheckBox("Aileron ACC");
    connect(Plot2Signals[22],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[23] = new QCheckBox("Valid gumstix");
    connect(Plot2Signals[23],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[24] = new QCheckBox("Elevator desired speed position cont");
    connect(Plot2Signals[24],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[25] = new QCheckBox("Aileron desired speed position cont");
    connect(Plot2Signals[25],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[26] = new QCheckBox("Elevator desired speed pos cont leader");
    connect(Plot2Signals[26],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[27] = new QCheckBox("Aileron desired speed pos cont leader");
    connect(Plot2Signals[27],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[28] = new QCheckBox("Output throttle");
    connect(Plot2Signals[28],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[29] = new QCheckBox("Output elevator");
    connect(Plot2Signals[29],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[30] = new QCheckBox("Output aileron");
    connect(Plot2Signals[30],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[31] = new QCheckBox("Output rudder");
    connect(Plot2Signals[31],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[32] = new QCheckBox("Blob Distance");
    connect(Plot2Signals[32],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[33] = new QCheckBox("Blob Horizontal");
    connect(Plot2Signals[33],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[34] = new QCheckBox("Blob Vertical");
    connect(Plot2Signals[34],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[35] = new QCheckBox("Pitch Angle");
    connect(Plot2Signals[35],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[36] = new QCheckBox("Roll Angle");
    connect(Plot2Signals[36],SIGNAL(clicked()),this,SLOT(Plot2Signals_clicked()));
    Plot2Signals[37] = new QCheckBox("All signals");
    connect(Plot2Signals[37],SIGNAL(clicked()),this,SLOT(Plot2SignalAll_clicked()));

    Plot3Signals[0] = new QCheckBox("Ground distance estimated");
    connect(Plot3Signals[0],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[1] = new QCheckBox("Ground distance");
    connect(Plot3Signals[1],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[2] = new QCheckBox("Elevator speed");
    connect(Plot3Signals[2],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[3] = new QCheckBox("Aileron_speed");
    connect(Plot3Signals[3],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[4] = new QCheckBox("Elevator speed estimated");
    connect(Plot3Signals[4],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[5] = new QCheckBox("Aileron speed estimated");
    connect(Plot3Signals[5],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[6] = new QCheckBox("Elevator position estimated");
    connect(Plot3Signals[6],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[7] = new QCheckBox("Aileron position estimated");
    connect(Plot3Signals[7],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[8] = new QCheckBox("Throttle controller output");
    connect(Plot3Signals[8],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[9] = new QCheckBox("Throttle speed");
    connect(Plot3Signals[9],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[10] = new QCheckBox("Aileron velocity controller output");
    connect(Plot3Signals[10],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[11] = new QCheckBox("Elevator velocity controller output");
    connect(Plot3Signals[11],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[12] = new QCheckBox("Aileron position controller output");
    connect(Plot3Signals[12],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[13] = new QCheckBox("Elevator position controller output");
    connect(Plot3Signals[13],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[14] = new QCheckBox("Throttle setpoint");
    connect(Plot3Signals[14],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[15] = new QCheckBox("Elevator position setpoint");
    connect(Plot3Signals[15],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[16] = new QCheckBox("Aileron position setpoint");
    connect(Plot3Signals[16],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[17] = new QCheckBox("Elevator velocity setpoint");
    connect(Plot3Signals[17],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[18] = new QCheckBox("Aileron velocity setpoint");
    connect(Plot3Signals[18],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[19] = new QCheckBox("Elevator speed estimated 2");
    connect(Plot3Signals[19],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[20] = new QCheckBox("Aileron speed estimated 2");
    connect(Plot3Signals[20],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[21] = new QCheckBox("Elevator ACC");
    connect(Plot3Signals[21],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[22] = new QCheckBox("Aileron ACC");
    connect(Plot3Signals[22],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[23] = new QCheckBox("Valid gumstix");
    connect(Plot3Signals[23],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[24] = new QCheckBox("Elevator desired speed position cont");
    connect(Plot3Signals[24],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[25] = new QCheckBox("Aileron desired speed position cont");
    connect(Plot3Signals[25],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[26] = new QCheckBox("Elevator desired speed pos cont leader");
    connect(Plot3Signals[26],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[27] = new QCheckBox("Aileron desired speed pos cont leader");
    connect(Plot3Signals[27],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[28] = new QCheckBox("Output throttle");
    connect(Plot3Signals[28],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[29] = new QCheckBox("Output elevator");
    connect(Plot3Signals[29],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[30] = new QCheckBox("Output aileron");
    connect(Plot3Signals[30],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[31] = new QCheckBox("Output rudder");
    connect(Plot3Signals[31],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[32] = new QCheckBox("Blob Distance");
    connect(Plot3Signals[32],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[33] = new QCheckBox("Blob Horizontal");
    connect(Plot3Signals[33],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[34] = new QCheckBox("Blob Vertical");
    connect(Plot3Signals[34],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[35] = new QCheckBox("Pitch Angle");
    connect(Plot3Signals[35],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[36] = new QCheckBox("Roll Angle");
    connect(Plot3Signals[36],SIGNAL(clicked()),this,SLOT(Plot3Signals_clicked()));
    Plot3Signals[37] = new QCheckBox("All signals");
    connect(Plot3Signals[37],SIGNAL(clicked()),this,SLOT(Plot3SignalAll_clicked()));

    Plot4Signals[0] = new QCheckBox("Ground distance estimated");
    connect(Plot4Signals[0],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[1] = new QCheckBox("Ground distance");
    connect(Plot4Signals[1],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[2] = new QCheckBox("Elevator speed");
    connect(Plot4Signals[2],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[3] = new QCheckBox("Aileron_speed");
    connect(Plot4Signals[3],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[4] = new QCheckBox("Elevator speed estimated");
    connect(Plot4Signals[4],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[5] = new QCheckBox("Aileron speed estimated");
    connect(Plot4Signals[5],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[6] = new QCheckBox("Elevator position estimated");
    connect(Plot4Signals[6],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[7] = new QCheckBox("Aileron position estimated");
    connect(Plot4Signals[7],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[8] = new QCheckBox("Throttle controller output");
    connect(Plot4Signals[8],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[9] = new QCheckBox("Throttle speed");
    connect(Plot4Signals[9],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[10] = new QCheckBox("Aileron velocity controller output");
    connect(Plot4Signals[10],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[11] = new QCheckBox("Elevator velocity controller output");
    connect(Plot4Signals[11],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[12] = new QCheckBox("Aileron position controller output");
    connect(Plot4Signals[12],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[13] = new QCheckBox("Elevator position controller output");
    connect(Plot4Signals[13],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[14] = new QCheckBox("Throttle setpoint");
    connect(Plot4Signals[14],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[15] = new QCheckBox("Elevator position setpoint");
    connect(Plot4Signals[15],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[16] = new QCheckBox("Aileron position setpoint");
    connect(Plot4Signals[16],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[17] = new QCheckBox("Elevator velocity setpoint");
    connect(Plot4Signals[17],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[18] = new QCheckBox("Aileron velocity setpoint");
    connect(Plot4Signals[18],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[19] = new QCheckBox("Elevator speed estimated 2");
    connect(Plot4Signals[19],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[20] = new QCheckBox("Aileron speed estimated 2");
    connect(Plot4Signals[20],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[21] = new QCheckBox("Elevator ACC");
    connect(Plot4Signals[21],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[22] = new QCheckBox("Aileron ACC");
    connect(Plot4Signals[22],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[23] = new QCheckBox("Valid gumstix");
    connect(Plot4Signals[23],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[24] = new QCheckBox("Elevator desired speed position cont");
    connect(Plot4Signals[24],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[25] = new QCheckBox("Aileron desired speed position cont");
    connect(Plot4Signals[25],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[26] = new QCheckBox("Elevator desired speed pos cont leader");
    connect(Plot4Signals[26],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[27] = new QCheckBox("Aileron desired speed pos cont leader");
    connect(Plot4Signals[27],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[28] = new QCheckBox("Output throttle");
    connect(Plot4Signals[28],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[29] = new QCheckBox("Output elevator");
    connect(Plot4Signals[29],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[30] = new QCheckBox("Output aileron");
    connect(Plot4Signals[30],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[31] = new QCheckBox("Output rudder");
    connect(Plot4Signals[31],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[32] = new QCheckBox("Blob Distance");
    connect(Plot4Signals[32],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[33] = new QCheckBox("Blob Horizontal");
    connect(Plot4Signals[33],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[34] = new QCheckBox("Blob Vertical");
    connect(Plot4Signals[34],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[35] = new QCheckBox("Pitch Angle");
    connect(Plot4Signals[35],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[36] = new QCheckBox("Roll Angle");
    connect(Plot4Signals[36],SIGNAL(clicked()),this,SLOT(Plot4Signals_clicked()));
    Plot4Signals[37] = new QCheckBox("All signals");
    connect(Plot4Signals[37],SIGNAL(clicked()),this,SLOT(Plot4SignalAll_clicked()));

    for(int i=0;i<38;i++){
        checkLayout->addWidget(Plot1Signals[i]);
    }
    for(int i=0;i<38;i++){
        checkLayout->addWidget(Plot2Signals[i]);
    }
    for(int i=0;i<38;i++){
        checkLayout->addWidget(Plot3Signals[i]);
    }
    for(int i=0;i<38;i++){
        checkLayout->addWidget(Plot4Signals[i]);
    }

    ui->scrollContents->setLayout(checkLayout);

}

void quadro::setAllCheckFalse()
{
    for(int i=0;i<38;i++){
        Plot1Signals[i]->setVisible(false);
    }
    for(int i=0;i<38;i++){
        Plot2Signals[i]->setVisible(false);
    }
    for(int i=0;i<38;i++){
        Plot3Signals[i]->setVisible(false);
    }
    for(int i=0;i<38;i++){
        Plot4Signals[i]->setVisible(false);
    }

}
void quadro::on_comboBox_activated(int index)
{
    if(index==0){
        setAllCheckFalse();
        for(int i=0;i<38;i++){
            Plot1Signals[i]->setVisible(true);
        }
    }
    else if(index==1){
        setAllCheckFalse();
        for(int i=0;i<38;i++){
            Plot2Signals[i]->setVisible(true);
        }
    }
    else if(index==2){
        setAllCheckFalse();
        for(int i=0;i<38;i++){
            Plot3Signals[i]->setVisible(true);
        }
    }
    else if(index==3){
        setAllCheckFalse();
        for(int i=0;i<38;i++){
            Plot4Signals[i]->setVisible(true);
        }
    }
}

void quadro::Plot1SignalAll_clicked()
{
    if(Plot1Signals[37]->isChecked()){
        for(int i=0;i<37;i++){
            Plot1Signals[i]->setChecked(true);
        }

    }else{
        for(int i=0;i<37;i++){
            Plot1Signals[i]->setChecked(false);
        }

    }
}

void quadro::Plot1Signals_clicked(){
    for(int i=0;i<37;i++){
        if(!Plot1Signals[i]->isChecked()){
            Plot1Signals[37]->setChecked(false);
        }
    }
}


void quadro::Plot2SignalAll_clicked()
{    if(Plot2Signals[37]->isChecked()){
        for(int i=0;i<37;i++){
            Plot2Signals[i]->setChecked(true);
        }

    }else{
        for(int i=0;i<37;i++){
            Plot2Signals[i]->setChecked(false);
        }

    }
}
void quadro::Plot2Signals_clicked(){
    for(int i=0;i<37;i++){
        if(!Plot2Signals[i]->isChecked()){
            Plot2Signals[37]->setChecked(false);
        }
    }
}

void quadro::Plot3SignalAll_clicked()
{
    if(Plot3Signals[37]->isChecked()){
        for(int i=0;i<37;i++){
            Plot3Signals[i]->setChecked(true);
        }

    }else{
        for(int i=0;i<37;i++){
            Plot3Signals[i]->setChecked(false);
        }

    }
}
void quadro::Plot3Signals_clicked(){
    for(int i=0;i<37;i++){
        if(!Plot3Signals[i]->isChecked()){
            Plot3Signals[37]->setChecked(false);
        }
    }
}

void quadro::Plot4SignalAll_clicked()
{
    if(Plot4Signals[37]->isChecked()){
        for(int i=0;i<37;i++){
            Plot4Signals[i]->setChecked(true);
        }

    }else{
        for(int i=0;i<37;i++){
            Plot4Signals[i]->setChecked(false);
        }

    }
}
void quadro::Plot4Signals_clicked(){
    for(int i=0;i<37;i++){
        if(!Plot4Signals[i]->isChecked()){
            Plot4Signals[37]->setChecked(false);
        }
    }
}

void quadro::detectStatus()
{
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataStatus()));
    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void quadro::realtimeDataStatus()
{
    if (getStatus(kopter, COMMANDS.CONTROLLERS)==CONTROLLERS.OFF){
        ui->controllersStatus->setText("Off");
    }else if (getStatus(kopter, COMMANDS.CONTROLLERS)==CONTROLLERS.VELOCITY){
        ui->controllersStatus->setText("Velocity");
    }else if (getStatus(kopter, COMMANDS.CONTROLLERS)==CONTROLLERS.POSITION){
        ui->controllersStatus->setText("Position");
    }else if (getStatus(kopter, COMMANDS.CONTROLLERS)==CONTROLLERS.BOTH){
        ui->controllersStatus->setText("Both");
    }

    if (getStatus(kopter, COMMANDS.LANDING)==ONOFF.ON){
        ui->landingStatus->setText("On");
    }else if (getStatus(kopter, COMMANDS.LANDING)==ONOFF.OFF){
        ui->landingStatus->setText("Off");
    }

    if (getStatus(kopter, COMMANDS.GUMSTIX)==ONOFF.ON){
        ui->gumstixStatus->setText("On");
    }else if (getStatus(kopter, COMMANDS.GUMSTIX)==ONOFF.OFF){
        ui->gumstixStatus->setText("Off");
    }
    if (getStatus(kopter, COMMANDS.TRAJECTORY_FOLLOW)==ONOFF.ON){
        ui->trajectoryStatus->setText("On");
    }else if (getStatus(kopter, COMMANDS.TRAJECTORY_FOLLOW)==ONOFF.OFF){
        ui->trajectoryStatus->setText("Off");
    }

}

void quadro::createLegend()
{
    QFormLayout *legendLay = new QFormLayout;
    // 1 line
    QLabel *legend1 = new QLabel("Ground distance estimated");
    QFrame *line1 = new QFrame();
    line1->setObjectName(QString::fromUtf8("line"));
    line1->setFrameShape(QFrame::HLine);
    line1->setFixedSize(35,15);
    line1->setLineWidth(2);
    QPalette palette1 = line1->palette();
    palette1.setColor(QPalette::WindowText, Qt::blue);
    line1->setPalette(palette1);

    QLabel *legend2 = new QLabel("Ground distance");
    QFrame *line2 = new QFrame();
    line2->setObjectName(QString::fromUtf8("line"));
    line2->setFrameShape(QFrame::HLine);
    line2->setFixedSize(35,15);
    line2->setLineWidth(2);
    QPalette palette2 = line2->palette();
    palette2.setColor(QPalette::WindowText, Qt::red);
    line2->setPalette(palette2);

    QLabel *legend3 = new QLabel("Elevator speed");
    QFrame *line3 = new QFrame();
    line3->setObjectName(QString::fromUtf8("line"));
    line3->setFrameShape(QFrame::HLine);
    line3->setFixedSize(35,15);
    line3->setLineWidth(2);
    QPalette palette3 = line3->palette();
    palette3.setColor(QPalette::WindowText, Qt::green);
    line3->setPalette(palette3);

    QLabel *legend4 = new QLabel("Aileron_speed");
    QFrame *line4 = new QFrame();
    line4->setObjectName(QString::fromUtf8("line"));
    line4->setFrameShape(QFrame::HLine);
    line4->setFixedSize(35,15);
    line4->setLineWidth(2);
    QPalette palette4 = line4->palette();
    palette4.setColor(QPalette::WindowText, Qt::black);
    line4->setPalette(palette4);

    QLabel *legend5 = new QLabel("Elevator speed estimated");
    QFrame *line5 = new QFrame();
    line5->setObjectName(QString::fromUtf8("line"));
    line5->setFrameShape(QFrame::HLine);
    line5->setFixedSize(35,15);
    line5->setLineWidth(2);
    QPalette palette5 = line5->palette();
    palette5.setColor(QPalette::WindowText, Qt::cyan);
    line5->setPalette(palette5);

    QLabel *legend6 = new QLabel("Aileron speed estimated");
    QFrame *line6 = new QFrame();
    line6->setObjectName(QString::fromUtf8("line"));
    line6->setFrameShape(QFrame::HLine);
    line6->setFixedSize(35,15);
    line6->setLineWidth(2);
    QPalette palette6 = line6->palette();
    palette6.setColor(QPalette::WindowText, Qt::gray);
    line6->setPalette(palette6);

    QLabel *legend7 = new QLabel("Elevator position estimated");
    QFrame *line7 = new QFrame();
    line7->setObjectName(QString::fromUtf8("line"));
    line7->setFrameShape(QFrame::HLine);
    line7->setFixedSize(35,15);
    line7->setLineWidth(2);
    QPalette palette7 = line7->palette();
    palette7.setColor(QPalette::WindowText, Qt::yellow);
    line7->setPalette(palette7);

    QLabel *legend8 = new QLabel("Aileron position estimated");
    QFrame *line8 = new QFrame();
    line8->setObjectName(QString::fromUtf8("line"));
    line8->setFrameShape(QFrame::HLine);
    line8->setFixedSize(35,15);
    line8->setLineWidth(2);
    QPalette palette8 = line8->palette();
    palette8.setColor(QPalette::WindowText, Qt::magenta);
    line8->setPalette(palette8);

    QLabel *legend9 = new QLabel("Throttle controller output");
    QFrame *line9 = new QFrame();
    line9->setObjectName(QString::fromUtf8("line"));
    line9->setFrameShape(QFrame::HLine);
    line9->setFixedSize(35,15);
    line9->setLineWidth(2);
    QPalette palette9 = line9->palette();
    palette9.setColor(QPalette::WindowText, Qt::darkBlue);
    line9->setPalette(palette9);

    QLabel *legend10 = new QLabel("Throttle speed");
    QFrame *line10 = new QFrame();
    line10->setObjectName(QString::fromUtf8("line"));
    line10->setFrameShape(QFrame::HLine);
    line10->setFixedSize(35,15);
    line10->setLineWidth(2);
    QPalette palette10 = line10->palette();
    palette10.setColor(QPalette::WindowText, Qt::darkRed);
    line10->setPalette(palette10);

    QLabel *legend11 = new QLabel("Aileron velocity controller output");
    QFrame *line11 = new QFrame();
    line11->setObjectName(QString::fromUtf8("line"));
    line11->setFrameShape(QFrame::HLine);
    line11->setFixedSize(35,15);
    line11->setLineWidth(2);
    QPalette palette11 = line11->palette();
    palette11.setColor(QPalette::WindowText, Qt::darkGreen);
    line11->setPalette(palette11);

    QLabel *legend12 = new QLabel("Elevator velocity controller output");
    QFrame *line12 = new QFrame();
    line12->setObjectName(QString::fromUtf8("line"));
    line12->setFrameShape(QFrame::HLine);
    line12->setFixedSize(35,15);
    line12->setLineWidth(2);
    QPalette palette12 = line12->palette();
    palette12.setColor(QPalette::WindowText, Qt::darkCyan);
    line12->setPalette(palette12);

    QLabel *legend13 = new QLabel("Aileron position controller output");
    QFrame *line13 = new QFrame();
    line13->setObjectName(QString::fromUtf8("line"));
    line13->setFrameShape(QFrame::HLine);
    line13->setFixedSize(35,15);
    line13->setLineWidth(2);
    QPalette palette13 = line13->palette();
    palette13.setColor(QPalette::WindowText, Qt::darkGray);
    line13->setPalette(palette13);

    QLabel *legend14 = new QLabel("Elevator position controller output");
    QFrame *line14 = new QFrame();
    line14->setObjectName(QString::fromUtf8("line"));
    line14->setFrameShape(QFrame::HLine);
    line14->setFixedSize(35,15);
    line14->setLineWidth(2);
    QPalette palette14 = line14->palette();
    palette14.setColor(QPalette::WindowText, Qt::darkYellow);
    line14->setPalette(palette14);

    QLabel *legend15 = new QLabel("Throttle setpoint");
    QFrame *line15 = new QFrame();
    line15->setObjectName(QString::fromUtf8("line"));
    line15->setFrameShape(QFrame::HLine);
    line15->setFixedSize(35,15);
    line15->setLineWidth(2);
    QPalette palette15 = line15->palette();
    palette15.setColor(QPalette::WindowText, Qt::darkMagenta);
    line15->setPalette(palette15);

    QLabel *legend16 = new QLabel("Elevator position setpoint");
    QFrame *line16 = new QFrame();
    line16->setObjectName(QString::fromUtf8("line"));
    line16->setFrameShape(QFrame::HLine);
    line16->setFixedSize(35,15);
    line16->setLineWidth(2);
    QPalette palette16 = line16->palette();
    palette16.setColor(QPalette::WindowText, QColor(85,0,130));
    line16->setPalette(palette16);

    QLabel *legend17 = new QLabel("Aileron position setpoint");
    QFrame *line17 = new QFrame();
    line17->setObjectName(QString::fromUtf8("line"));
    line17->setFrameShape(QFrame::HLine);
    line17->setFixedSize(35,15);
    line17->setLineWidth(2);
    QPalette palette17 = line17->palette();
    palette17.setColor(QPalette::WindowText, QColor(255,170,255));
    line17->setPalette(palette17);

    QLabel *legend18 = new QLabel("Elevator velocity setpoint");
    QFrame *line18 = new QFrame();
    line18->setObjectName(QString::fromUtf8("line"));
    line18->setFrameShape(QFrame::HLine);
    line18->setFixedSize(35,15);
    line18->setLineWidth(2);
    QPalette palette18 = line18->palette();
    palette18.setColor(QPalette::WindowText, QColor(0,255,130));
    line18->setPalette(palette18);

    QLabel *legend19 = new QLabel("Aileron velocity setpoint");
    QFrame *line19 = new QFrame();
    line19->setObjectName(QString::fromUtf8("line"));
    line19->setFrameShape(QFrame::HLine);
    line19->setFixedSize(35,15);
    line19->setLineWidth(2);
    QPalette palette19 = line19->palette();
    palette19.setColor(QPalette::WindowText, QColor(170,170,255));
    line19->setPalette(palette19);

    QLabel *legend20 = new QLabel("Elevator speed estimated 2");
    QFrame *line20 = new QFrame();
    line20->setObjectName(QString::fromUtf8("line"));
    line20->setFrameShape(QFrame::HLine);
    line20->setFixedSize(35,15);
    line20->setLineWidth(2);
    QPalette palette20 = line20->palette();
    palette20.setColor(QPalette::WindowText, QColor(255,130,0));
    line20->setPalette(palette20);

    QLabel *legend21 = new QLabel("Aileron speed estimated 2");
    QFrame *line21 = new QFrame();
    line21->setObjectName(QString::fromUtf8("line"));
    line21->setFrameShape(QFrame::HLine);
    line21->setFixedSize(35,15);
    line21->setLineWidth(2);
    QPalette palette21 = line21->palette();
    palette21.setColor(QPalette::WindowText, QColor(80,25,0));
    line21->setPalette(palette21);

    QLabel *legend22 = new QLabel("Elevator ACC");
    QFrame *line22 = new QFrame();
    line22->setObjectName(QString::fromUtf8("line"));
    line22->setFrameShape(QFrame::HLine);
    line22->setFixedSize(35,15);
    line22->setLineWidth(2);
    QPalette palette22 = line22->palette();
    palette22.setColor(QPalette::WindowText, QColor(100,0,80));
    line22->setPalette(palette22);

    QLabel *legend23 = new QLabel("Aileron ACC");
    QFrame *line23 = new QFrame();
    line23->setObjectName(QString::fromUtf8("line"));
    line23->setFrameShape(QFrame::HLine);
    line23->setFixedSize(35,15);
    line23->setLineWidth(2);
    QPalette palette23 = line23->palette();
    palette23.setColor(QPalette::WindowText, QColor(170,255,0));
    line23->setPalette(palette23);

    QLabel *legend24 = new QLabel("Valid gumstix");
    QFrame *line24 = new QFrame();
    line24->setObjectName(QString::fromUtf8("line"));
    line24->setFrameShape(QFrame::HLine);
    line24->setFixedSize(35,15);
    line24->setLineWidth(2);
    QPalette palette24 = line24->palette();
    palette24.setColor(QPalette::WindowText, QColor(255,170,110));
    line24->setPalette(palette24);

    QLabel *legend25 = new QLabel("Elevator desired speed position cont");
    QFrame *line25 = new QFrame();
    line25->setObjectName(QString::fromUtf8("line"));
    line25->setFrameShape(QFrame::HLine);
    line25->setFixedSize(35,15);
    line25->setLineWidth(2);
    QPalette palette25 = line25->palette();
    palette25.setColor(QPalette::WindowText, QColor(175,90,0));
    line25->setPalette(palette25);

    QLabel *legend26 = new QLabel("Aileron desired speed position cont");
    QFrame *line26 = new QFrame();
    line26->setObjectName(QString::fromUtf8("line"));
    line26->setFrameShape(QFrame::HLine);
    line26->setFixedSize(35,15);
    line26->setLineWidth(2);
    QPalette palette26 = line26->palette();
    palette26.setColor(QPalette::WindowText, QColor(80,100,120));
    line26->setPalette(palette26);

    QLabel *legend27 = new QLabel("Elevator desired speed pos cont leader");
    QFrame *line27 = new QFrame();
    line27->setObjectName(QString::fromUtf8("line"));
    line27->setFrameShape(QFrame::HLine);
    line27->setFixedSize(35,15);
    line27->setLineWidth(2);
    QPalette palette27 = line27->palette();
    palette27.setColor(QPalette::WindowText, QColor(170,60,110));
    line27->setPalette(palette27);

    QLabel *legend28 = new QLabel("Aileron desired speed pos cont leader");
    QFrame *line28 = new QFrame();
    line28->setObjectName(QString::fromUtf8("line"));
    line28->setFrameShape(QFrame::HLine);
    line28->setFixedSize(35,15);
    line28->setLineWidth(2);
    QPalette palette28 = line28->palette();
    palette28.setColor(QPalette::WindowText, QColor(85,85,0));
    line28->setPalette(palette28);

    QLabel *legend29 = new QLabel("Output throttle");
    QFrame *line29 = new QFrame();
    line29->setObjectName(QString::fromUtf8("line"));
    line29->setFrameShape(QFrame::HLine);
    line29->setFixedSize(35,15);
    line29->setLineWidth(2);
    QPalette palette29 = line29->palette();
    palette29.setColor(QPalette::WindowText, QColor(80,70,70));
    line29->setPalette(palette29);

    QLabel *legend30 = new QLabel("Output elevator");
    QFrame *line30 = new QFrame();
    line30->setObjectName(QString::fromUtf8("line"));
    line30->setFrameShape(QFrame::HLine);
    line30->setFixedSize(35,15);
    line30->setLineWidth(2);
    QPalette palette30 = line30->palette();
    palette30.setColor(QPalette::WindowText, QColor(10,100,80));
    line30->setPalette(palette30);

    QLabel *legend31 = new QLabel("Output aileron");
    QFrame *line31 = new QFrame();
    line31->setObjectName(QString::fromUtf8("line"));
    line31->setFrameShape(QFrame::HLine);
    line31->setFixedSize(35,15);
    line31->setLineWidth(2);
    QPalette palette31 = line31->palette();
    palette31.setColor(QPalette::WindowText, QColor(30,110,130));
    line31->setPalette(palette31);

    QLabel *legend32 = new QLabel("Output rudder");
    QFrame *line32 = new QFrame();
    line32->setObjectName(QString::fromUtf8("line"));
    line32->setFrameShape(QFrame::HLine);
    line32->setFixedSize(35,15);
    line32->setLineWidth(2);
    QPalette palette32 = line32->palette();
    palette32.setColor(QPalette::WindowText, QColor(180,180,180));
    line32->setPalette(palette32);

    QLabel *legend33 = new QLabel("Blob Distance");
    QFrame *line33 = new QFrame();
    line33->setObjectName(QString::fromUtf8("line"));
    line33->setFrameShape(QFrame::HLine);
    line33->setFixedSize(35,15);
    line33->setLineWidth(2);
    QPalette palette33 = line33->palette();
    palette33.setColor(QPalette::WindowText, Qt::red);
    line33->setPalette(palette33);

    QLabel *legend34 = new QLabel("Blob Horizontal");
    QFrame *line34 = new QFrame();
    line34->setObjectName(QString::fromUtf8("line"));
    line34->setFrameShape(QFrame::HLine);
    line34->setFixedSize(35,15);
    line34->setLineWidth(2);
    QPalette palette34 = line34->palette();
    palette34.setColor(QPalette::WindowText, Qt::blue);
    line34->setPalette(palette34);

    QLabel *legend35 = new QLabel("Blob Vertical");
    QFrame *line35 = new QFrame();
    line35->setObjectName(QString::fromUtf8("line"));
    line35->setFrameShape(QFrame::HLine);
    line35->setFixedSize(35,15);
    line35->setLineWidth(2);
    QPalette palette35 = line35->palette();
    palette35.setColor(QPalette::WindowText, Qt::green);
    line35->setPalette(palette35);

    QLabel *legend36 = new QLabel("Pitch Angle");
    QFrame *line36 = new QFrame();
    line36->setObjectName(QString::fromUtf8("line"));
    line36->setFrameShape(QFrame::HLine);
    line36->setFixedSize(35,15);
    line36->setLineWidth(2);
    QPalette palette36 = line36->palette();
    palette36.setColor(QPalette::WindowText, Qt::magenta);
    line36->setPalette(palette36);

    QLabel *legend37 = new QLabel("Roll Angle");
    QFrame *line37 = new QFrame();
    line37->setObjectName(QString::fromUtf8("line"));
    line37->setFrameShape(QFrame::HLine);
    line37->setFixedSize(35,15);
    line37->setLineWidth(2);
    QPalette palette37 = line37->palette();
    palette37.setColor(QPalette::WindowText, Qt::gray);
    line37->setPalette(palette37);

    legendLay->addRow(line1,legend1);
    legendLay->addRow(line2,legend2);
    legendLay->addRow(line3,legend3);
    legendLay->addRow(line4,legend4);
    legendLay->addRow(line5,legend5);
    legendLay->addRow(line6,legend6);
    legendLay->addRow(line7,legend7);
    legendLay->addRow(line8,legend8);
    legendLay->addRow(line9,legend9);
    legendLay->addRow(line10,legend10);
    legendLay->addRow(line11,legend11);
    legendLay->addRow(line12,legend12);
    legendLay->addRow(line13,legend13);
    legendLay->addRow(line14,legend14);
    legendLay->addRow(line15,legend15);
    legendLay->addRow(line16,legend16);
    legendLay->addRow(line17,legend17);
    legendLay->addRow(line18,legend18);
    legendLay->addRow(line19,legend19);
    legendLay->addRow(line20,legend20);
    legendLay->addRow(line21,legend21);
    legendLay->addRow(line22,legend22);
    legendLay->addRow(line23,legend23);
    legendLay->addRow(line24,legend24);
    legendLay->addRow(line25,legend25);
    legendLay->addRow(line26,legend26);
    legendLay->addRow(line27,legend27);
    legendLay->addRow(line28,legend28);
    legendLay->addRow(line29,legend29);
    legendLay->addRow(line30,legend30);
    legendLay->addRow(line31,legend31);
    legendLay->addRow(line32,legend32);
    legendLay->addRow(line28,legend33);
    legendLay->addRow(line29,legend34);
    legendLay->addRow(line30,legend35);
    legendLay->addRow(line31,legend36);
    legendLay->addRow(line32,legend37);
    ui->scrollLegend->setLayout(legendLay);
}

void quadro::on_actionK1_triggered()
{
    kopter=KOPTERS.K1;
    setSignalsOn();
    startFailureDetectionThread();
}

void quadro::on_actionK2_triggered()
{
    kopter=KOPTERS.K2;
    setSignalsOn();
    startFailureDetectionThread();
}

void quadro::on_actionK3_triggered()
{
    kopter=KOPTERS.K3;
    setSignalsOn();
    startFailureDetectionThread();
}

void quadro::on_actionKC1_triggered()
{
    kopter=KOPTERS.KC1;
    setSignalsOn();
    startFailureDetectionThread();
}

// Landing
void quadro::on_actionOn_triggered()
{
    land(kopter,ONOFF.ON);
}

void quadro::on_actionOff_triggered()
{
    land(kopter,ONOFF.OFF);
}
// controllers
void quadro::on_actionVelocity_triggered()
{
    setController(kopter,CONTROLLERS.VELOCITY);
}

void quadro::on_actionPosition_triggered()
{
    setController(kopter,CONTROLLERS.POSITION);
}
void quadro::on_actionBoth_triggered()
{
    setController(kopter,CONTROLLERS.BOTH);
}

void quadro::on_actionOff_3_triggered()
{
    setController(kopter,CONTROLLERS.OFF);
}

// setpoints
void quadro::on_actionRelative_triggered()
{    bool ok;
     double d = QInputDialog::getDouble(this, tr("Enter value"),
                                        tr("Amount:"), 0, -100, 100, 2, &ok);

      setSetpoint(kopter,SETPOINTS.THROTTLE_SP,POSITIONS.RELATIV, (float)d);
}

void quadro::on_actionAbsolute_triggered()
{    bool ok;
     double d = QInputDialog::getDouble(this, tr("Enter value"),
                                        tr("Amount:"), 0, -100, 100, 2, &ok);
      setSetpoint(kopter,SETPOINTS.THROTTLE_SP,POSITIONS.ABSOLUT, (float)d);
}

void quadro::on_actionRelative_2_triggered()
{    bool ok;
     double d = QInputDialog::getDouble(this, tr("Enter value"),
                                        tr("Amount:"), 0, -100, 100, 2, &ok);
      setSetpoint(kopter,SETPOINTS.ELEVATOR_POSITION,POSITIONS.RELATIV, (float)d);
}

void quadro::on_actionAbsolute_2_triggered()
{    bool ok;
     double d = QInputDialog::getDouble(this, tr("Enter value"),
                                        tr("Amount:"), 0, -100, 100, 2, &ok);
      setSetpoint(kopter,SETPOINTS.ELEVATOR_POSITION,POSITIONS.ABSOLUT, (float)d);
}

void quadro::on_actionRelative_3_triggered()
{    bool ok;
     double d = QInputDialog::getDouble(this, tr("Enter value"),
                                        tr("Amount:"), 0, -100, 100, 2, &ok);
      setSetpoint(kopter,SETPOINTS.AILERON_POSITION,POSITIONS.RELATIV, (float)d);
}

void quadro::on_actionAbsolute_3_triggered()
{    bool ok;
     double d = QInputDialog::getDouble(this, tr("Enter value"),
                                        tr("Amount:"), 0, -100, 100, 2, &ok);
      setSetpoint(kopter,SETPOINTS.AILERON_POSITION,POSITIONS.ABSOLUT, (float)d);
}

void quadro::on_actionRelative_4_triggered()
{    bool ok;
     double d = QInputDialog::getDouble(this, tr("Enter value"),
                                        tr("Amount:"), 0, -100, 100, 2, &ok);
      setSetpoint(kopter,SETPOINTS.ELEVATOR_VELOCITY,POSITIONS.RELATIV, (float)d);
}

void quadro::on_actionAbsolute_4_triggered()
{    bool ok;
     double d = QInputDialog::getDouble(this, tr("Enter value"),
                                        tr("Amount:"), 0, -100, 100, 2, &ok);
      setSetpoint(kopter,SETPOINTS.ELEVATOR_VELOCITY,POSITIONS.ABSOLUT, (float)d);
}

void quadro::on_actionRelative_5_triggered()
{    bool ok;
     double d = QInputDialog::getDouble(this, tr("Enter value"),
                                        tr("Amount:"), 0, -100, 100, 2, &ok);
      setSetpoint(kopter,SETPOINTS.AILERON_VELOCITY,POSITIONS.RELATIV, (float)d);
}

void quadro::on_actionAbsolute_5_triggered()
{    bool ok;
     double d = QInputDialog::getDouble(this, tr("Enter value"),
                                        tr("Amount:"), 0, -100, 100, 2, &ok);
      setSetpoint(kopter,SETPOINTS.AILERON_VELOCITY,POSITIONS.ABSOLUT, (float)d);
}

// Trajectory
void quadro::on_actionFollow_trajectory_triggered()
{
    trajectoryFollow(kopter,ONOFF.ON);
}
void quadro::on_actionFollow_trajectory_off_triggered()
{
    trajectoryFollow(kopter,ONOFF.OFF);
}

void quadro::on_actionLoad_trajectory_triggered()
{
    newTrajectory = new sendTrajectory(this);
    newTrajectory->show();
    newTrajectory->setKopter(kopter);
}

// Gumstix
void quadro::on_actionOn_2_triggered()
{
    setGumstix(kopter,ONOFF.ON);
}

void quadro::on_actionOff_2_triggered()
{
    setGumstix(kopter,ONOFF.OFF);
}

void quadro::currentDateTime(){
    time_t     now = time(0);
    struct tm  tstruct;
    char TMP[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(TMP, sizeof(TMP), "%Y-%m-%d.%X", &tstruct);
    for (int i=0; TMP[i]!= '\0'; i++){
        if (TMP[i] == ':')TMP[i] = ';';
    }
    sprintf(myTime,"dataLog_%s.txt",TMP);
}

void quadro::setSignalsOn(){
    telemetryGet(kopter,TELEMETRIES.GROUND_DISTANCE_ESTIMATED,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.GROUND_DISTANCE,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.ELEVATOR_SPEED,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.AILERON_SPEED,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.ELEVATOR_SPEED_ESTIMATED,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.AILERON_SPEED_ESTIMATED,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.ELEVATOR_POS_ESTIMATED,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.AILERON_POS_ESTIMATED,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.THROTTLE_CONTROLLER_OUTPUT,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.THROTTLE_SPEED,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.AILERON_VEL_CONTROLLER_OUTPUT,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.ELEVATOR_VEL_CONTROLLER_OUTPUT,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.AILERON_POS_CONTROLLER_OUTPUT,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.ELEVATOR_POS_CONTROLLER_OUTPUT,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.THROTTLE_SETPOINT,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.ELEVATOR_POS_SETPOINT,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.AILERON_POS_SETPOINT,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.ELEVATOR_VEL_SETPOINT,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.AILERON_VEL_SETPOINT,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.ELEVATOR_SPEED_ESTIMATED2,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.AILERON_SPEED_ESTIMATED2,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.ELEVATOR_ACC,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.AILERON_ACC,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.VALID_GUMSTIX,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.ELEVATOR_DESIRED_SPEED_POS_CONT,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.AILERON_DESIRED_SPEED_POS_CONT,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.ELE_DES_SPEED_POS_CONT_LEADER,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.AIL_DES_SPEED_POS_CONT_LEADER,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.OUTPUT_THROTTLE,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.OUTPUT_ELEVATOR,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.OUTPUT_AILERON,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.OUTPUT_RUDDER,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.BLOB_DISTANCE,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.BLOB_HORIZONTAL,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.BLOB_VERTICAL,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.PITCH_ANGLE,ONOFF.ON);
    telemetryGet(kopter,TELEMETRIES.ROLL_ANGLE,ONOFF.ON);
}


void quadro::closeEvent (QCloseEvent *event)
{
    QMessageBox::StandardButton resBtn = QMessageBox::question( this,"Quadcopter",
                                                                tr("Are you sure?\n"),
                                                                QMessageBox::Cancel | QMessageBox::No | QMessageBox::Yes,
                                                                QMessageBox::Yes);
    if (resBtn != QMessageBox::Yes) {
        event->ignore();
    } else {
        dataTimer.stop();
        //failureDetectionThread->m_abort=true;
        //failureDetectionThread->wait();
        event->accept();
    }
}

void quadro::startFailureDetectionThread()
{
    if(failureDetectionBool==false){
        //failureDetectionThread->setKopter(kopter);
      //  failureDetectionThread->start();
        failureDetectionBool=true;
    }else{
       // failureDetectionThread->setKopter(kopter);
    }

}

void quadro::on_pushButton_clicked()
{
    errorDia = new errorDialog(this);
    //errorDia->setFailure(failureDetectionThread);
    errorDia->show();
    errorDia->writeErrors();
}
