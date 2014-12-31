#include "quadro.h"
#include "ui_quadro.h"
#include <QDebug>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>


double key1=0,lastPointKey1=0,key2=0,lastPointKey2=0,key3=0,lastPointKey3=0,key4=0,lastPointKey4=0;
int numberOfQuadro=0;
boolean statusEnabled=false;

quadro::quadro(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::quadro)
{
    ui->setupUi(this);
    createScrollCheckBox();
    createLegend();
    setAllCheckFalse();
   startGraph();
   detectStatus();
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
    graph1->plotLayout()->insertRow(0);
    graph1->plotLayout()->addElement(0, 0, new QCPPlotTitle(graph1, "Plot 1"));


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

    /*graph1->xAxis->setTickLabelType(QCPAxis::ltDateTime);       // pro casovou osu
  graph1->xAxis->setDateTimeFormat("hh:mm:ss");
  graph1->xAxis->setAutoTickStep(false);
  graph1->xAxis->setTickStep(2);*/
    graph1->xAxis->setTickLabelRotation(30);                      // pro osu se vzorkama
    graph1->xAxis->setAutoTickStep(false);
    graph1->xAxis->setTickStep(1);

    graph1->axisRect()->setupFullAxesBox();

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(graph1->xAxis, SIGNAL(rangeChanged(QCPRange)), graph1->xAxis2, SLOT(setRange(QCPRange)));
    connect(graph1->yAxis, SIGNAL(rangeChanged(QCPRange)), graph1->yAxis2, SLOT(setRange(QCPRange)));

    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    connect(&dataTimer1, SIGNAL(timeout()), this, SLOT(realtimeDataSlotGraph1()));
    dataTimer1.start(0); // Interval 0 means to refresh as fast as possible
}

void quadro::realtimeDataSlotGraph1()
{
    // calculate two new data points:                                    // pro cas
    /*#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
  double key = 0;
#else
  double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
#endif
  static double lastPointKey = 0;*/
  //  if (key1-lastPointKey1 > 1)   {                   // at most add point every 10 ms

/*
        // add data to lines:
        if(ui->Quadro1Signal1->isChecked()){
            double value0 = sin(key1*1.6+cos(key1*1.7)*2)*10 + sin(key1*1.2+0.56)*20 + 26;
            ui->graph1->graph(0)->addData(key1, value0);
            ui->graph1->graph(8)->clearData();
            ui->graph1->graph(8)->addData(key1, value0);
        }
        if(ui->Quadro1Signal2->isChecked()){
            double value1 = sin(key1*1.3+cos(key1*1.2)*1.2)*7 + sin(key1*0.9+0.26)*24 + 26;
            ui->graph1->graph(1)->addData(key1, value1);
            ui->graph1->graph(9)->clearData();
            ui->graph1->graph(9)->addData(key1, value1);
        }
        if(ui->Quadro1Signal3->isChecked()){
            double value2 = getter();
            ui->graph1->graph(2)->addData(key1, value2);
            ui->graph1->graph(10)->clearData();
            ui->graph1->graph(10)->addData(key1, value2);
        }
        if(ui->Quadro1Signal4->isChecked()){
            double value3 = 15;
            ui->graph1->graph(3)->addData(key1, value3);
            ui->graph1->graph(11)->clearData();
            ui->graph1->graph(11)->addData(key1, value3);
        }
        if(ui->Quadro1Signal5->isChecked()){
            double value4 = 20;
            ui->graph1->graph(4)->addData(key1, value4);
            ui->graph1->graph(12)->clearData();
            ui->graph1->graph(12)->addData(key1, value4);
        }
        if(ui->Quadro1Signal6->isChecked()){
            double value5 = 25;
            ui->graph1->graph(5)->addData(key1, value5);
            ui->graph1->graph(13)->clearData();
            ui->graph1->graph(13)->addData(key1, value5);
        }
        if(ui->Quadro1Signal7->isChecked()){
            double value6 = 30;
            ui->graph1->graph(6)->addData(key1, value6);
            ui->graph1->graph(14)->clearData();
            ui->graph1->graph(14)->addData(key1, value6);
        }
        if(ui->Quadro1Signal8->isChecked()){
            double value7 = 40;
            ui->graph1->graph(7)->addData(key1, value7);
            ui->graph1->graph(15)->clearData();
            ui->graph1->graph(15)->addData(key1, value7);
        }*/
    if(Plot1Signal1->isChecked()){
        double value0 = sin(key1*1.6+cos(key1*1.7)*2)*10 + sin(key1*1.2+0.56)*20 + 26;
        ui->graph1->graph(0)->addData(key1, value0);
    }
    if(Plot1Signal2->isChecked()){
        double value1 = cos(key1*1.6+cos(key1*1.7)*2)*10 + sin(key1*1.2+0.56)*20 + 26;
        ui->graph1->graph(1)->addData(key1, value1);
    }
    if(Plot1Signal3->isChecked()){
        double value2 = sin(key1*1.6+cos(key1*1.7)*2)*10 + sin(key1*1.2+0.56)*20 ;
        ui->graph1->graph(2)->addData(key1, value2);
    }
    if(Plot1Signal4->isChecked()){
        double value3 = sin(key1*1.6+cos(key1*1.7)*2)*10 + sin(key1*1.2+0.56)*20 +16;
        ui->graph1->graph(3)->addData(key1, value3);
    }
    if(Plot1Signal5->isChecked()){
        double value4 = sin(key1*1.6+cos(key1*1.7)*2)*10 + sin(key1*1.2+0.56)*20 + 56;
        ui->graph1->graph(4)->addData(key1, value4);
    }
    if(Plot1Signal6->isChecked()){
        double value5 = sin(key1*1.6+cos(key1*1.7)*2)*10 + sin(key1*1.2+0.56)*20 + 6;
        ui->graph1->graph(5)->addData(key1, value5);
    }
    if(Plot1Signal7->isChecked()){
        double value6 = sin(key1*1.6+cos(key1*1.7)*2)*10 + sin(key1*1.2+0.56)*20 + 36;
        ui->graph1->graph(6)->addData(key1, value6);
    }
    if(Plot1Signal8->isChecked()){
        double value7 = 5;
        ui->graph1->graph(7)->addData(key1, value7);
    }
    if(Plot1Signal9->isChecked()){
        double value8 = sin(cos(key1*1.7)*2)*10 + sin(key1*1.2+0.56)*20 + 26;
        ui->graph1->graph(8)->addData(key1, value8);
    }
    if(Plot1Signal10->isChecked()){
        double value9 = sin(key1*1.6+cos(key1*1.7))*10 + sin(key1*1.2+0.56)*20 + 26;
        ui->graph1->graph(9)->addData(key1, value9);
    }
    if(Plot1Signal11->isChecked()){
        double value10 = sin(key1*1.6+cos(key1*1.7)*2)*10 + 26;
        ui->graph1->graph(10)->addData(key1, value10);
    }
    if(Plot1Signal12->isChecked()){
        double value11 = sin(key1*1.6) + sin(key1*1.2+0.56)*20 + 26;
        ui->graph1->graph(11)->addData(key1, value11);
    }
    if(Plot1Signal13->isChecked()){
        double value12 = 10;
        ui->graph1->graph(12)->addData(key1, value12);
    }
    if(Plot1Signal14->isChecked()){
        double value13 = 2;
        ui->graph1->graph(13)->addData(key1, value13);
    }
    if(Plot1Signal15->isChecked()){
        double value14 = 20;
        ui->graph1->graph(14)->addData(key1, value14);
    }
    if(Plot1Signal16->isChecked()){
        double value15 = 15;
        ui->graph1->graph(15)->addData(key1, value15);
    }
    if(Plot1Signal17->isChecked()){
        double value16 = 16;
        ui->graph1->graph(16)->addData(key1, value16);
    }
    if(Plot1Signal18->isChecked()){
        double value17 = 17;
        ui->graph1->graph(17)->addData(key1, value17);
    }
    if(Plot1Signal19->isChecked()){
        double value18 = 18;
        ui->graph1->graph(18)->addData(key1, value18);
    }
    if(Plot1Signal20->isChecked()){
        double value19 = 19;
        ui->graph1->graph(19)->addData(key1, value19);
    }
    if(Plot1Signal21->isChecked()){
        double value20 = 20;
        ui->graph1->graph(20)->addData(key1, value20);
    }
    if(Plot1Signal22->isChecked()){
        double value21 = 21;
        ui->graph1->graph(21)->addData(key1, value21);
    }
    if(Plot1Signal23->isChecked()){
        double value22 = 22;
        ui->graph1->graph(22)->addData(key1, value22);
    }
    if(Plot1Signal24->isChecked()){
        double value23 = 23;
        ui->graph1->graph(23)->addData(key1, value23);
    }
    if(Plot1Signal25->isChecked()){
        double value24 = 24;
        ui->graph1->graph(24)->addData(key1, value24);
    }
    if(Plot1Signal26->isChecked()){
        double value25 = 25;
        ui->graph1->graph(25)->addData(key1, value25);
    }
    if(Plot1Signal27->isChecked()){
        double value26 = 26;
        ui->graph1->graph(26)->addData(key1, value26);
    }
    if(Plot1Signal28->isChecked()){
        double value27 = 27;
        ui->graph1->graph(27)->addData(key1, value27);
    }
    if(Plot1Signal29->isChecked()){
        double value28 = 28;
        ui->graph1->graph(28)->addData(key1, value28);
    }
    if(Plot1Signal30->isChecked()){
        double value29 = 29;
        ui->graph1->graph(29)->addData(key1, value29);
    }
    if(Plot1Signal31->isChecked()){
        double value30 = 30;
        ui->graph1->graph(30)->addData(key1, value30);
    }
    if(Plot1Signal32->isChecked()){
        double value31 = 31;
        ui->graph1->graph(31)->addData(key1, value31);
    }
        // remove data of lines that's outside visible range:
        ui->graph1->graph(0)->removeDataBefore(key1-100);
        ui->graph1->graph(1)->removeDataBefore(key1-100);
        ui->graph1->graph(2)->removeDataBefore(key1-100);
        ui->graph1->graph(3)->removeDataBefore(key1-100);
        ui->graph1->graph(4)->removeDataBefore(key1-100);
        ui->graph1->graph(5)->removeDataBefore(key1-100);
        ui->graph1->graph(6)->removeDataBefore(key1-100);
        ui->graph1->graph(7)->removeDataBefore(key1-100);
        ui->graph1->graph(8)->removeDataBefore(key1-100);
        ui->graph1->graph(9)->removeDataBefore(key1-100);
        ui->graph1->graph(10)->removeDataBefore(key1-100);
        ui->graph1->graph(11)->removeDataBefore(key1-100);
        ui->graph1->graph(12)->removeDataBefore(key1-100);
        ui->graph1->graph(13)->removeDataBefore(key1-100);
        ui->graph1->graph(14)->removeDataBefore(key1-100);
        ui->graph1->graph(15)->removeDataBefore(key1-100);
        ui->graph1->graph(16)->removeDataBefore(key1-100);
        ui->graph1->graph(17)->removeDataBefore(key1-100);
        ui->graph1->graph(18)->removeDataBefore(key1-100);
        ui->graph1->graph(19)->removeDataBefore(key1-100);
        ui->graph1->graph(20)->removeDataBefore(key1-100);
        ui->graph1->graph(21)->removeDataBefore(key1-100);
        ui->graph1->graph(22)->removeDataBefore(key1-100);
        ui->graph1->graph(23)->removeDataBefore(key1-100);
        ui->graph1->graph(24)->removeDataBefore(key1-100);
        ui->graph1->graph(25)->removeDataBefore(key1-100);
        ui->graph1->graph(26)->removeDataBefore(key1-100);
        ui->graph1->graph(27)->removeDataBefore(key1-100);
        ui->graph1->graph(28)->removeDataBefore(key1-100);
        ui->graph1->graph(29)->removeDataBefore(key1-100);
        ui->graph1->graph(30)->removeDataBefore(key1-100);
        ui->graph1->graph(31)->removeDataBefore(key1-100);

        // rescale value (vertical) axis to fit the current data:
        ui->graph1->graph(0)->rescaleValueAxis();
        ui->graph1->graph(1)->rescaleValueAxis(true);
        ui->graph1->graph(2)->rescaleValueAxis(true);
        ui->graph1->graph(3)->rescaleValueAxis(true);
        ui->graph1->graph(4)->rescaleValueAxis(true);
        ui->graph1->graph(5)->rescaleValueAxis(true);
        ui->graph1->graph(6)->rescaleValueAxis(true);
        ui->graph1->graph(7)->rescaleValueAxis(true);
        ui->graph1->graph(8)->rescaleValueAxis(true);
        ui->graph1->graph(9)->rescaleValueAxis(true);
        ui->graph1->graph(10)->rescaleValueAxis(true);
        ui->graph1->graph(11)->rescaleValueAxis(true);
        ui->graph1->graph(12)->rescaleValueAxis(true);
        ui->graph1->graph(13)->rescaleValueAxis(true);
        ui->graph1->graph(14)->rescaleValueAxis(true);
        ui->graph1->graph(15)->rescaleValueAxis(true);
        ui->graph1->graph(16)->rescaleValueAxis(true);
        ui->graph1->graph(17)->rescaleValueAxis(true);
        ui->graph1->graph(18)->rescaleValueAxis(true);
        ui->graph1->graph(19)->rescaleValueAxis(true);
        ui->graph1->graph(20)->rescaleValueAxis(true);
        ui->graph1->graph(21)->rescaleValueAxis(true);
        ui->graph1->graph(22)->rescaleValueAxis(true);
        ui->graph1->graph(23)->rescaleValueAxis(true);
        ui->graph1->graph(24)->rescaleValueAxis(true);
        ui->graph1->graph(25)->rescaleValueAxis(true);
        ui->graph1->graph(26)->rescaleValueAxis(true);
        ui->graph1->graph(27)->rescaleValueAxis(true);
        ui->graph1->graph(28)->rescaleValueAxis(true);
        ui->graph1->graph(29)->rescaleValueAxis(true);
        ui->graph1->graph(30)->rescaleValueAxis(true);
        ui->graph1->graph(31)->rescaleValueAxis(true);
        lastPointKey1 = key1;
   // }
    // make key1 axis range scroll with the data (at a constant range size of 8):
    ui->graph1->xAxis->setRange(key1+0.25, 10, Qt::AlignRight);
    ui->graph1->replot();

    // calculate frames per second:
    // static double lastFpskey1;
    // static int frameCount;
    //  ++frameCount;
    /*  if (key1-lastFpskey1 > 2) // average fps over 2 seconds
  {
    ui->statusBar->showMessage(
          QString("%1 FPS, Total Data points: %2")
          .arg(frameCount/(key1-lastFpskey1), 0, 'f', 0)
          .arg(ui->graph1->graph(0)->data()->count()+ui->graph1->graph(1)->data()->count())
          , 0);
    lastFpskey1 = key1;
    frameCount = 0;
  }*/
    // delay
 //   if(ui->Quadro1Signal1->isChecked()||ui->Quadro1Signal2->isChecked()||ui->Quadro1Signal3->isChecked()||ui->Quadro1Signal4->isChecked()
     //       ||ui->Quadro1Signal5->isChecked()||ui->Quadro1Signal6->isChecked()||ui->Quadro1Signal7->isChecked()||ui->Quadro1Signal8->isChecked()){
        key1=key1+0.03;
 //   }
}

void quadro::graph_2(QCustomPlot *graph2)
{
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
QMessageBox::critical(this, "", "You're using Qt < 4.7, the realtime data needs functions that are available with Qt 4.7 to work properly");
#endif

// set title of plot:
graph2->plotLayout()->insertRow(0);
graph2->plotLayout()->addElement(0, 0, new QCPPlotTitle(graph2, "Plot 2"));


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

graph2->xAxis->setTickLabelRotation(30);                      // pro osu se vzorkama
graph2->xAxis->setAutoTickStep(false);
graph2->xAxis->setTickStep(1);

graph2->axisRect()->setupFullAxesBox();

// make left and bottom axes transfer their ranges to right and top axes:
connect(graph2->xAxis, SIGNAL(rangeChanged(QCPRange)), graph2->xAxis2, SLOT(setRange(QCPRange)));
connect(graph2->yAxis, SIGNAL(rangeChanged(QCPRange)), graph2->yAxis2, SLOT(setRange(QCPRange)));

// setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
connect(&dataTimer2, SIGNAL(timeout()), this, SLOT(realtimeDataSlotGraph2()));
dataTimer2.start(0); // Interval 0 means to refresh as fast as possible
}

void quadro::realtimeDataSlotGraph2()
{
    if (key2-lastPointKey2 > 0.01)                      // at most add point every 10 ms
    {
        if(Plot2Signal1->isChecked()){
            double value0 = sin(key2*1.6+cos(key2*1.7)*2)*10 + sin(key2*1.2+0.56)*20 + 26;
            ui->graph2->graph(0)->addData(key2, value0);
        }
        if(Plot2Signal2->isChecked()){
            double value1 = cos(key2*1.6+cos(key2*1.7)*2)*10 + sin(key2*1.2+0.56)*20 + 26;
            ui->graph2->graph(1)->addData(key2, value1);
        }
        if(Plot2Signal3->isChecked()){
            double value2 = sin(key2*1.6+cos(key2*1.7)*2)*10 + sin(key2*1.2+0.56)*20 ;
            ui->graph2->graph(2)->addData(key2, value2);
        }
        if(Plot2Signal4->isChecked()){
            double value3 = sin(key2*1.6+cos(key2*1.7)*2)*10 + sin(key2*1.2+0.56)*20 +16;
            ui->graph2->graph(3)->addData(key2, value3);
        }
        if(Plot2Signal5->isChecked()){
            double value4 = sin(key2*1.6+cos(key2*1.7)*2)*10 + sin(key2*1.2+0.56)*20 + 56;
            ui->graph2->graph(4)->addData(key2, value4);
        }
        if(Plot2Signal6->isChecked()){
            double value5 = sin(key2*1.6+cos(key2*1.7)*2)*10 + sin(key2*1.2+0.56)*20 + 6;
            ui->graph2->graph(5)->addData(key2, value5);
        }
        if(Plot2Signal7->isChecked()){
            double value6 = sin(key2*1.6+cos(key2*1.7)*2)*10 + sin(key2*1.2+0.56)*20 + 36;
            ui->graph2->graph(6)->addData(key2, value6);
        }
        if(Plot2Signal8->isChecked()){
            double value7 = 5;
            ui->graph2->graph(7)->addData(key2, value7);
        }
        if(Plot2Signal9->isChecked()){
            double value8 = sin(cos(key2*1.7)*2)*10 + sin(key2*1.2+0.56)*20 + 26;
            ui->graph2->graph(8)->addData(key2, value8);
        }
        if(Plot2Signal10->isChecked()){
            double value9 = sin(key2*1.6+cos(key2*1.7))*10 + sin(key2*1.2+0.56)*20 + 26;
            ui->graph2->graph(9)->addData(key2, value9);
        }
        if(Plot2Signal11->isChecked()){
            double value10 = sin(key2*1.6+cos(key2*1.7)*2)*10 + 26;
            ui->graph2->graph(10)->addData(key2, value10);
        }
        if(Plot2Signal12->isChecked()){
            double value11 = sin(key2*1.6) + sin(key2*1.2+0.56)*20 + 26;
            ui->graph2->graph(11)->addData(key2, value11);
        }
        if(Plot2Signal13->isChecked()){
            double value12 = 10;
            ui->graph2->graph(12)->addData(key2, value12);
        }
        if(Plot2Signal14->isChecked()){
            double value13 = 2;
            ui->graph2->graph(13)->addData(key2, value13);
        }
        if(Plot2Signal15->isChecked()){
            double value14 = 20;
            ui->graph2->graph(14)->addData(key2, value14);
        }
        if(Plot2Signal16->isChecked()){
            double value15 = 15;
            ui->graph2->graph(15)->addData(key2, value15);
        }
        if(Plot2Signal17->isChecked()){
            double value16 = 16;
            ui->graph2->graph(16)->addData(key2, value16);
        }
        if(Plot2Signal18->isChecked()){
            double value17 = 17;
            ui->graph2->graph(17)->addData(key2, value17);
        }
        if(Plot2Signal19->isChecked()){
            double value18 = 18;
            ui->graph2->graph(18)->addData(key2, value18);
        }
        if(Plot2Signal20->isChecked()){
            double value19 = 19;
            ui->graph2->graph(19)->addData(key2, value19);
        }
        if(Plot2Signal21->isChecked()){
            double value20 = 20;
            ui->graph2->graph(20)->addData(key2, value20);
        }
        if(Plot2Signal22->isChecked()){
            double value21 = 21;
            ui->graph2->graph(21)->addData(key2, value21);
        }
        if(Plot2Signal23->isChecked()){
            double value22 = 22;
            ui->graph2->graph(22)->addData(key2, value22);
        }
        if(Plot2Signal24->isChecked()){
            double value23 = 23;
            ui->graph2->graph(23)->addData(key2, value23);
        }
        if(Plot2Signal25->isChecked()){
            double value24 = 24;
            ui->graph2->graph(24)->addData(key2, value24);
        }
        if(Plot2Signal26->isChecked()){
            double value25 = 25;
            ui->graph2->graph(25)->addData(key2, value25);
        }
        if(Plot2Signal27->isChecked()){
            double value26 = 26;
            ui->graph2->graph(26)->addData(key2, value26);
        }
        if(Plot2Signal28->isChecked()){
            double value27 = 27;
            ui->graph2->graph(27)->addData(key2, value27);
        }
        if(Plot2Signal29->isChecked()){
            double value28 = 28;
            ui->graph2->graph(28)->addData(key2, value28);
        }
        if(Plot2Signal30->isChecked()){
            double value29 = 29;
            ui->graph2->graph(29)->addData(key2, value29);
        }
        if(Plot2Signal31->isChecked()){
            double value30 = 30;
            ui->graph2->graph(30)->addData(key2, value30);
        }
        if(Plot2Signal32->isChecked()){
            double value31 = 31;
            ui->graph2->graph(31)->addData(key2, value31);
        }
    // remove data of lines that's outside visible range:
    ui->graph2->graph(0)->removeDataBefore(key2-10);
    ui->graph2->graph(1)->removeDataBefore(key2-10);
    ui->graph2->graph(2)->removeDataBefore(key2-10);
    ui->graph2->graph(3)->removeDataBefore(key2-10);
    ui->graph2->graph(4)->removeDataBefore(key2-10);
    ui->graph2->graph(5)->removeDataBefore(key2-10);
    ui->graph2->graph(6)->removeDataBefore(key2-10);
    ui->graph2->graph(7)->removeDataBefore(key2-10);
    ui->graph2->graph(8)->removeDataBefore(key2-10);
    ui->graph2->graph(9)->removeDataBefore(key2-10);
    ui->graph2->graph(10)->removeDataBefore(key2-10);
    ui->graph2->graph(11)->removeDataBefore(key2-10);
    ui->graph2->graph(12)->removeDataBefore(key2-10);
    ui->graph2->graph(13)->removeDataBefore(key2-10);
    ui->graph2->graph(14)->removeDataBefore(key2-10);
    ui->graph2->graph(15)->removeDataBefore(key2-10);
    ui->graph2->graph(16)->removeDataBefore(key2-10);
    ui->graph2->graph(17)->removeDataBefore(key2-10);
    ui->graph2->graph(18)->removeDataBefore(key2-10);
    ui->graph2->graph(19)->removeDataBefore(key2-10);
    ui->graph2->graph(20)->removeDataBefore(key2-10);
    ui->graph2->graph(21)->removeDataBefore(key2-10);
    ui->graph2->graph(22)->removeDataBefore(key2-10);
    ui->graph2->graph(23)->removeDataBefore(key2-10);
    ui->graph2->graph(24)->removeDataBefore(key2-10);
    ui->graph2->graph(25)->removeDataBefore(key2-10);
    ui->graph2->graph(26)->removeDataBefore(key2-10);
    ui->graph2->graph(27)->removeDataBefore(key2-10);
    ui->graph2->graph(28)->removeDataBefore(key2-10);
    ui->graph2->graph(29)->removeDataBefore(key2-10);
    ui->graph2->graph(30)->removeDataBefore(key2-10);
    ui->graph2->graph(31)->removeDataBefore(key2-10);

    // rescale value (vertical) axis to fit the current data:
    ui->graph2->graph(0)->rescaleValueAxis();
    ui->graph2->graph(1)->rescaleValueAxis(true);
    ui->graph2->graph(2)->rescaleValueAxis(true);
    ui->graph2->graph(3)->rescaleValueAxis(true);
    ui->graph2->graph(4)->rescaleValueAxis(true);
    ui->graph2->graph(5)->rescaleValueAxis(true);
    ui->graph2->graph(6)->rescaleValueAxis(true);
    ui->graph2->graph(7)->rescaleValueAxis(true);
    ui->graph2->graph(8)->rescaleValueAxis(true);
    ui->graph2->graph(9)->rescaleValueAxis(true);
    ui->graph2->graph(10)->rescaleValueAxis(true);
    ui->graph2->graph(11)->rescaleValueAxis(true);
    ui->graph2->graph(12)->rescaleValueAxis(true);
    ui->graph2->graph(13)->rescaleValueAxis(true);
    ui->graph2->graph(14)->rescaleValueAxis(true);
    ui->graph2->graph(15)->rescaleValueAxis(true);
    ui->graph2->graph(16)->rescaleValueAxis(true);
    ui->graph2->graph(17)->rescaleValueAxis(true);
    ui->graph2->graph(18)->rescaleValueAxis(true);
    ui->graph2->graph(19)->rescaleValueAxis(true);
    ui->graph2->graph(20)->rescaleValueAxis(true);
    ui->graph2->graph(21)->rescaleValueAxis(true);
    ui->graph2->graph(22)->rescaleValueAxis(true);
    ui->graph2->graph(23)->rescaleValueAxis(true);
    ui->graph2->graph(24)->rescaleValueAxis(true);
    ui->graph2->graph(25)->rescaleValueAxis(true);
    ui->graph2->graph(26)->rescaleValueAxis(true);
    ui->graph2->graph(27)->rescaleValueAxis(true);
    ui->graph2->graph(28)->rescaleValueAxis(true);
    ui->graph2->graph(29)->rescaleValueAxis(true);
    ui->graph2->graph(30)->rescaleValueAxis(true);
    ui->graph2->graph(31)->rescaleValueAxis(true);
    lastPointKey2 = key2;
}
// make key2 axis range scroll with the data (at a constant range size of 8):
ui->graph2->xAxis->setRange(key2+0.25, 10, Qt::AlignRight);
ui->graph2->replot();

    key2=key2+0.03;

}

void quadro::graph_3(QCustomPlot *graph3)
{
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
QMessageBox::critical(this, "", "You're using Qt < 4.7, the realtime data needs functions that are available with Qt 4.7 to work properly");
#endif

// set title of plot:
graph3->plotLayout()->insertRow(0);
graph3->plotLayout()->addElement(0, 0, new QCPPlotTitle(graph3, "Plot 3"));


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

graph3->xAxis->setTickLabelRotation(30);                      // pro osu se vzorkama
graph3->xAxis->setAutoTickStep(false);
graph3->xAxis->setTickStep(1);

graph3->axisRect()->setupFullAxesBox();

// make left and bottom axes transfer their ranges to right and top axes:
connect(graph3->xAxis, SIGNAL(rangeChanged(QCPRange)), graph3->xAxis2, SLOT(setRange(QCPRange)));
connect(graph3->yAxis, SIGNAL(rangeChanged(QCPRange)), graph3->yAxis2, SLOT(setRange(QCPRange)));

// setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
connect(&dataTimer3, SIGNAL(timeout()), this, SLOT(realtimeDataSlotGraph3()));
dataTimer3.start(0); // Interval 0 means to refresh as fast as possible
}


void quadro::realtimeDataSlotGraph3()
{
    if (key3-lastPointKey3 > 0.01)                      // at most add point every 10 ms
    {
        if(Plot3Signal1->isChecked()){
            double value0 = sin(key3*1.6+cos(key3*1.7)*2)*10 + sin(key3*1.2+0.56)*20 + 26;
            ui->graph3->graph(0)->addData(key3, value0);
        }
        if(Plot3Signal2->isChecked()){
            double value1 = cos(key3*1.6+cos(key3*1.7)*2)*10 + sin(key3*1.2+0.56)*20 + 26;
            ui->graph3->graph(1)->addData(key3, value1);
        }
        if(Plot3Signal3->isChecked()){
            double value2 = sin(key3*1.6+cos(key3*1.7)*2)*10 + sin(key3*1.2+0.56)*20 ;
            ui->graph3->graph(2)->addData(key3, value2);
        }
        if(Plot3Signal4->isChecked()){
            double value3 = sin(key3*1.6+cos(key3*1.7)*2)*10 + sin(key3*1.2+0.56)*20 +16;
            ui->graph3->graph(3)->addData(key3, value3);
        }
        if(Plot3Signal5->isChecked()){
            double value4 = sin(key3*1.6+cos(key3*1.7)*2)*10 + sin(key3*1.2+0.56)*20 + 56;
            ui->graph3->graph(4)->addData(key3, value4);
        }
        if(Plot3Signal6->isChecked()){
            double value5 = sin(key3*1.6+cos(key3*1.7)*2)*10 + sin(key3*1.2+0.56)*20 + 6;
            ui->graph3->graph(5)->addData(key3, value5);
        }
        if(Plot3Signal7->isChecked()){
            double value6 = sin(key3*1.6+cos(key3*1.7)*2)*10 + sin(key3*1.2+0.56)*20 + 36;
            ui->graph3->graph(6)->addData(key3, value6);
        }
        if(Plot3Signal8->isChecked()){
            double value7 = 5;
            ui->graph3->graph(7)->addData(key3, value7);
        }
        if(Plot3Signal9->isChecked()){
            double value8 = sin(cos(key3*1.7)*2)*10 + sin(key3*1.2+0.56)*20 + 26;
            ui->graph3->graph(8)->addData(key3, value8);
        }
        if(Plot3Signal10->isChecked()){
            double value9 = sin(key3*1.6+cos(key3*1.7))*10 + sin(key3*1.2+0.56)*20 + 26;
            ui->graph3->graph(9)->addData(key3, value9);
        }
        if(Plot3Signal11->isChecked()){
            double value10 = sin(key3*1.6+cos(key3*1.7)*2)*10 + 26;
            ui->graph3->graph(10)->addData(key3, value10);
        }
        if(Plot3Signal12->isChecked()){
            double value11 = sin(key3*1.6) + sin(key3*1.2+0.56)*20 + 26;
            ui->graph3->graph(11)->addData(key3, value11);
        }
        if(Plot3Signal13->isChecked()){
            double value12 = 10;
            ui->graph3->graph(12)->addData(key3, value12);
        }
        if(Plot3Signal14->isChecked()){
            double value13 = 2;
            ui->graph3->graph(13)->addData(key3, value13);
        }
        if(Plot3Signal15->isChecked()){
            double value14 = 20;
            ui->graph3->graph(14)->addData(key3, value14);
        }
        if(Plot3Signal16->isChecked()){
            double value15 = 15;
            ui->graph3->graph(15)->addData(key3, value15);
        }
        if(Plot3Signal17->isChecked()){
            double value16 = 16;
            ui->graph3->graph(16)->addData(key3, value16);
        }
        if(Plot3Signal18->isChecked()){
            double value17 = 17;
            ui->graph3->graph(17)->addData(key3, value17);
        }
        if(Plot3Signal19->isChecked()){
            double value18 = 18;
            ui->graph3->graph(18)->addData(key3, value18);
        }
        if(Plot3Signal20->isChecked()){
            double value19 = 19;
            ui->graph3->graph(19)->addData(key3, value19);
        }
        if(Plot3Signal21->isChecked()){
            double value20 = 20;
            ui->graph3->graph(20)->addData(key3, value20);
        }
        if(Plot3Signal22->isChecked()){
            double value21 = 21;
            ui->graph3->graph(21)->addData(key3, value21);
        }
        if(Plot3Signal23->isChecked()){
            double value22 = 22;
            ui->graph3->graph(22)->addData(key3, value22);
        }
        if(Plot3Signal24->isChecked()){
            double value23 = 23;
            ui->graph3->graph(23)->addData(key3, value23);
        }
        if(Plot3Signal25->isChecked()){
            double value24 = 24;
            ui->graph3->graph(24)->addData(key3, value24);
        }
        if(Plot3Signal26->isChecked()){
            double value25 = 25;
            ui->graph3->graph(25)->addData(key3, value25);
        }
        if(Plot3Signal27->isChecked()){
            double value26 = 26;
            ui->graph3->graph(26)->addData(key3, value26);
        }
        if(Plot3Signal28->isChecked()){
            double value27 = 27;
            ui->graph3->graph(27)->addData(key3, value27);
        }
        if(Plot3Signal29->isChecked()){
            double value28 = 28;
            ui->graph3->graph(28)->addData(key3, value28);
        }
        if(Plot3Signal30->isChecked()){
            double value29 = 29;
            ui->graph3->graph(29)->addData(key3, value29);
        }
        if(Plot3Signal31->isChecked()){
            double value30 = 30;
            ui->graph3->graph(30)->addData(key3, value30);
        }
        if(Plot3Signal32->isChecked()){
            double value31 = 31;
            ui->graph3->graph(31)->addData(key3, value31);
        }
    // remove data of lines that's outside visible range:
    ui->graph3->graph(0)->removeDataBefore(key3-10);
    ui->graph3->graph(1)->removeDataBefore(key3-10);
    ui->graph3->graph(2)->removeDataBefore(key3-10);
    ui->graph3->graph(3)->removeDataBefore(key3-10);
    ui->graph3->graph(4)->removeDataBefore(key3-10);
    ui->graph3->graph(5)->removeDataBefore(key3-10);
    ui->graph3->graph(6)->removeDataBefore(key3-10);
    ui->graph3->graph(7)->removeDataBefore(key3-10);
    ui->graph3->graph(8)->removeDataBefore(key3-10);
    ui->graph3->graph(9)->removeDataBefore(key3-10);
    ui->graph3->graph(10)->removeDataBefore(key3-10);
    ui->graph3->graph(11)->removeDataBefore(key3-10);
    ui->graph3->graph(12)->removeDataBefore(key3-10);
    ui->graph3->graph(13)->removeDataBefore(key3-10);
    ui->graph3->graph(14)->removeDataBefore(key3-10);
    ui->graph3->graph(15)->removeDataBefore(key3-10);
    ui->graph3->graph(16)->removeDataBefore(key3-10);
    ui->graph3->graph(17)->removeDataBefore(key3-10);
    ui->graph3->graph(18)->removeDataBefore(key3-10);
    ui->graph3->graph(19)->removeDataBefore(key3-10);
    ui->graph3->graph(20)->removeDataBefore(key3-10);
    ui->graph3->graph(21)->removeDataBefore(key3-10);
    ui->graph3->graph(22)->removeDataBefore(key3-10);
    ui->graph3->graph(23)->removeDataBefore(key3-10);
    ui->graph3->graph(24)->removeDataBefore(key3-10);
    ui->graph3->graph(25)->removeDataBefore(key3-10);
    ui->graph3->graph(26)->removeDataBefore(key3-10);
    ui->graph3->graph(27)->removeDataBefore(key3-10);
    ui->graph3->graph(28)->removeDataBefore(key3-10);
    ui->graph3->graph(29)->removeDataBefore(key3-10);
    ui->graph3->graph(30)->removeDataBefore(key3-10);
    ui->graph3->graph(31)->removeDataBefore(key3-10);

    // rescale value (vertical) axis to fit the current data:
    ui->graph3->graph(0)->rescaleValueAxis();
    ui->graph3->graph(1)->rescaleValueAxis(true);
    ui->graph3->graph(2)->rescaleValueAxis(true);
    ui->graph3->graph(3)->rescaleValueAxis(true);
    ui->graph3->graph(4)->rescaleValueAxis(true);
    ui->graph3->graph(5)->rescaleValueAxis(true);
    ui->graph3->graph(6)->rescaleValueAxis(true);
    ui->graph3->graph(7)->rescaleValueAxis(true);
    ui->graph3->graph(8)->rescaleValueAxis(true);
    ui->graph3->graph(9)->rescaleValueAxis(true);
    ui->graph3->graph(10)->rescaleValueAxis(true);
    ui->graph3->graph(11)->rescaleValueAxis(true);
    ui->graph3->graph(12)->rescaleValueAxis(true);
    ui->graph3->graph(13)->rescaleValueAxis(true);
    ui->graph3->graph(14)->rescaleValueAxis(true);
    ui->graph3->graph(15)->rescaleValueAxis(true);
    ui->graph3->graph(16)->rescaleValueAxis(true);
    ui->graph3->graph(17)->rescaleValueAxis(true);
    ui->graph3->graph(18)->rescaleValueAxis(true);
    ui->graph3->graph(19)->rescaleValueAxis(true);
    ui->graph3->graph(20)->rescaleValueAxis(true);
    ui->graph3->graph(21)->rescaleValueAxis(true);
    ui->graph3->graph(22)->rescaleValueAxis(true);
    ui->graph3->graph(23)->rescaleValueAxis(true);
    ui->graph3->graph(24)->rescaleValueAxis(true);
    ui->graph3->graph(25)->rescaleValueAxis(true);
    ui->graph3->graph(26)->rescaleValueAxis(true);
    ui->graph3->graph(27)->rescaleValueAxis(true);
    ui->graph3->graph(28)->rescaleValueAxis(true);
    ui->graph3->graph(29)->rescaleValueAxis(true);
    ui->graph3->graph(30)->rescaleValueAxis(true);
    ui->graph3->graph(31)->rescaleValueAxis(true);
    lastPointKey3 = key3;
}
// make key3 axis range scroll with the data (at a constant range size of 8):
ui->graph3->xAxis->setRange(key3+0.25, 10, Qt::AlignRight);
ui->graph3->replot();

    key3=key3+0.03;

}

void quadro::graph_4(QCustomPlot *graph4)
{
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
QMessageBox::critical(this, "", "You're using Qt < 4.7, the realtime data needs functions that are available with Qt 4.7 to work properly");
#endif

// set title of plot:
graph4->plotLayout()->insertRow(0);
graph4->plotLayout()->addElement(0, 0, new QCPPlotTitle(graph4, "Plot 4"));


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

graph4->xAxis->setTickLabelRotation(30);                      // pro osu se vzorkama
graph4->xAxis->setAutoTickStep(false);
graph4->xAxis->setTickStep(1);

graph4->axisRect()->setupFullAxesBox();

// make left and bottom axes transfer their ranges to right and top axes:
connect(graph4->xAxis, SIGNAL(rangeChanged(QCPRange)), graph4->xAxis2, SLOT(setRange(QCPRange)));
connect(graph4->yAxis, SIGNAL(rangeChanged(QCPRange)), graph4->yAxis2, SLOT(setRange(QCPRange)));

// setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
connect(&dataTimer4, SIGNAL(timeout()), this, SLOT(realtimeDataSlotGraph4()));
dataTimer4.start(0); // Interval 0 means to refresh as fast as possible
}


void quadro::realtimeDataSlotGraph4()
{
    if (key4-lastPointKey4 > 0.01)                      // at most add point every 10 ms
    {
        if(Plot4Signal1->isChecked()){
            double value0 = sin(key4*1.6+cos(key4*1.7)*2)*10 + sin(key4*1.2+0.56)*20 + 26;
            ui->graph4->graph(0)->addData(key4, value0);
        }
        if(Plot4Signal2->isChecked()){
            double value1 = cos(key4*1.6+cos(key4*1.7)*2)*10 + sin(key4*1.2+0.56)*20 + 26;
            ui->graph4->graph(1)->addData(key4, value1);
        }
        if(Plot4Signal3->isChecked()){
            double value2 = sin(key4*1.6+cos(key4*1.7)*2)*10 + sin(key4*1.2+0.56)*20 ;
            ui->graph4->graph(2)->addData(key4, value2);
        }
        if(Plot4Signal4->isChecked()){
            double value3 = sin(key4*1.6+cos(key4*1.7)*2)*10 + sin(key4*1.2+0.56)*20 +16;
            ui->graph4->graph(3)->addData(key4, value3);
        }
        if(Plot4Signal5->isChecked()){
            double value4 = sin(key4*1.6+cos(key4*1.7)*2)*10 + sin(key4*1.2+0.56)*20 + 56;
            ui->graph4->graph(4)->addData(key4, value4);
        }
        if(Plot4Signal6->isChecked()){
            double value5 = sin(key4*1.6+cos(key4*1.7)*2)*10 + sin(key4*1.2+0.56)*20 + 6;
            ui->graph4->graph(5)->addData(key4, value5);
        }
        if(Plot4Signal7->isChecked()){
            double value6 = sin(key4*1.6+cos(key4*1.7)*2)*10 + sin(key4*1.2+0.56)*20 + 36;
            ui->graph4->graph(6)->addData(key4, value6);
        }
        if(Plot4Signal8->isChecked()){
            double value7 = 5;
            ui->graph4->graph(7)->addData(key4, value7);
        }
        if(Plot4Signal9->isChecked()){
            double value8 = sin(cos(key4*1.7)*2)*10 + sin(key4*1.2+0.56)*20 + 26;
            ui->graph4->graph(8)->addData(key4, value8);
        }
        if(Plot4Signal10->isChecked()){
            double value9 = sin(key4*1.6+cos(key4*1.7))*10 + sin(key4*1.2+0.56)*20 + 26;
            ui->graph4->graph(9)->addData(key4, value9);
        }
        if(Plot4Signal11->isChecked()){
            double value10 = sin(key4*1.6+cos(key4*1.7)*2)*10 + 26;
            ui->graph4->graph(10)->addData(key4, value10);
        }
        if(Plot4Signal12->isChecked()){
            double value11 = sin(key4*1.6) + sin(key4*1.2+0.56)*20 + 26;
            ui->graph4->graph(11)->addData(key4, value11);
        }
        if(Plot4Signal13->isChecked()){
            double value12 = 10;
            ui->graph4->graph(12)->addData(key4, value12);
        }
        if(Plot4Signal14->isChecked()){
            double value13 = 2;
            ui->graph4->graph(13)->addData(key4, value13);
        }
        if(Plot4Signal15->isChecked()){
            double value14 = 20;
            ui->graph4->graph(14)->addData(key4, value14);
        }
        if(Plot4Signal16->isChecked()){
            double value15 = 15;
            ui->graph4->graph(15)->addData(key4, value15);
        }
        if(Plot4Signal17->isChecked()){
            double value16 = 16;
            ui->graph4->graph(16)->addData(key4, value16);
        }
        if(Plot4Signal18->isChecked()){
            double value17 = 17;
            ui->graph4->graph(17)->addData(key4, value17);
        }
        if(Plot4Signal19->isChecked()){
            double value18 = 18;
            ui->graph4->graph(18)->addData(key4, value18);
        }
        if(Plot4Signal20->isChecked()){
            double value19 = 19;
            ui->graph4->graph(19)->addData(key4, value19);
        }
        if(Plot4Signal21->isChecked()){
            double value20 = 20;
            ui->graph4->graph(20)->addData(key4, value20);
        }
        if(Plot4Signal22->isChecked()){
            double value21 = 21;
            ui->graph4->graph(21)->addData(key4, value21);
        }
        if(Plot4Signal23->isChecked()){
            double value22 = 22;
            ui->graph4->graph(22)->addData(key4, value22);
        }
        if(Plot4Signal24->isChecked()){
            double value23 = 23;
            ui->graph4->graph(23)->addData(key4, value23);
        }
        if(Plot4Signal25->isChecked()){
            double value24 = 24;
            ui->graph4->graph(24)->addData(key4, value24);
        }
        if(Plot4Signal26->isChecked()){
            double value25 = 25;
            ui->graph4->graph(25)->addData(key4, value25);
        }
        if(Plot4Signal27->isChecked()){
            double value26 = 26;
            ui->graph4->graph(26)->addData(key4, value26);
        }
        if(Plot4Signal28->isChecked()){
            double value27 = 27;
            ui->graph4->graph(27)->addData(key4, value27);
        }
        if(Plot4Signal29->isChecked()){
            double value28 = 28;
            ui->graph4->graph(28)->addData(key4, value28);
        }
        if(Plot4Signal30->isChecked()){
            double value29 = 29;
            ui->graph4->graph(29)->addData(key4, value29);
        }
        if(Plot4Signal31->isChecked()){
            double value30 = 30;
            ui->graph4->graph(30)->addData(key4, value30);
        }
        if(Plot4Signal32->isChecked()){
            double value31 = 31;
            ui->graph4->graph(31)->addData(key4, value31);
        }
    // remove data of lines that's outside visible range:
    ui->graph4->graph(0)->removeDataBefore(key4-10);
    ui->graph4->graph(1)->removeDataBefore(key4-10);
    ui->graph4->graph(2)->removeDataBefore(key4-10);
    ui->graph4->graph(3)->removeDataBefore(key4-10);
    ui->graph4->graph(4)->removeDataBefore(key4-10);
    ui->graph4->graph(5)->removeDataBefore(key4-10);
    ui->graph4->graph(6)->removeDataBefore(key4-10);
    ui->graph4->graph(7)->removeDataBefore(key4-10);
    ui->graph4->graph(8)->removeDataBefore(key4-10);
    ui->graph4->graph(9)->removeDataBefore(key4-10);
    ui->graph4->graph(10)->removeDataBefore(key4-10);
    ui->graph4->graph(11)->removeDataBefore(key4-10);
    ui->graph4->graph(12)->removeDataBefore(key4-10);
    ui->graph4->graph(13)->removeDataBefore(key4-10);
    ui->graph4->graph(14)->removeDataBefore(key4-10);
    ui->graph4->graph(15)->removeDataBefore(key4-10);
    ui->graph4->graph(16)->removeDataBefore(key4-10);
    ui->graph4->graph(17)->removeDataBefore(key4-10);
    ui->graph4->graph(18)->removeDataBefore(key4-10);
    ui->graph4->graph(19)->removeDataBefore(key4-10);
    ui->graph4->graph(20)->removeDataBefore(key4-10);
    ui->graph4->graph(21)->removeDataBefore(key4-10);
    ui->graph4->graph(22)->removeDataBefore(key4-10);
    ui->graph4->graph(23)->removeDataBefore(key4-10);
    ui->graph4->graph(24)->removeDataBefore(key4-10);
    ui->graph4->graph(25)->removeDataBefore(key4-10);
    ui->graph4->graph(26)->removeDataBefore(key4-10);
    ui->graph4->graph(27)->removeDataBefore(key4-10);
    ui->graph4->graph(28)->removeDataBefore(key4-10);
    ui->graph4->graph(29)->removeDataBefore(key4-10);
    ui->graph4->graph(30)->removeDataBefore(key4-10);
    ui->graph4->graph(31)->removeDataBefore(key4-10);

    // rescale value (vertical) axis to fit the current data:
    ui->graph4->graph(0)->rescaleValueAxis();
    ui->graph4->graph(1)->rescaleValueAxis(true);
    ui->graph4->graph(2)->rescaleValueAxis(true);
    ui->graph4->graph(3)->rescaleValueAxis(true);
    ui->graph4->graph(4)->rescaleValueAxis(true);
    ui->graph4->graph(5)->rescaleValueAxis(true);
    ui->graph4->graph(6)->rescaleValueAxis(true);
    ui->graph4->graph(7)->rescaleValueAxis(true);
    ui->graph4->graph(8)->rescaleValueAxis(true);
    ui->graph4->graph(9)->rescaleValueAxis(true);
    ui->graph4->graph(10)->rescaleValueAxis(true);
    ui->graph4->graph(11)->rescaleValueAxis(true);
    ui->graph4->graph(12)->rescaleValueAxis(true);
    ui->graph4->graph(13)->rescaleValueAxis(true);
    ui->graph4->graph(14)->rescaleValueAxis(true);
    ui->graph4->graph(15)->rescaleValueAxis(true);
    ui->graph4->graph(16)->rescaleValueAxis(true);
    ui->graph4->graph(17)->rescaleValueAxis(true);
    ui->graph4->graph(18)->rescaleValueAxis(true);
    ui->graph4->graph(19)->rescaleValueAxis(true);
    ui->graph4->graph(20)->rescaleValueAxis(true);
    ui->graph4->graph(21)->rescaleValueAxis(true);
    ui->graph4->graph(22)->rescaleValueAxis(true);
    ui->graph4->graph(23)->rescaleValueAxis(true);
    ui->graph4->graph(24)->rescaleValueAxis(true);
    ui->graph4->graph(25)->rescaleValueAxis(true);
    ui->graph4->graph(26)->rescaleValueAxis(true);
    ui->graph4->graph(27)->rescaleValueAxis(true);
    ui->graph4->graph(28)->rescaleValueAxis(true);
    ui->graph4->graph(29)->rescaleValueAxis(true);
    ui->graph4->graph(30)->rescaleValueAxis(true);
    ui->graph4->graph(31)->rescaleValueAxis(true);
    lastPointKey4 = key4;
}

// make key4 axis range scroll with the data (at a constant range size of 8):
ui->graph4->xAxis->setRange(key4+0.25, 10, Qt::AlignRight);
ui->graph4->replot();

    key4=key4+0.03;

}


void quadro::on_actionConnect_quadrocopter_triggered()
{
    bool ok;
          numberOfQuadro = QInputDialog::getInt(this, tr("QInputDialog::getInteger()"),
                                       tr("Quadrocopter:"), 1, 0, 4, 1, &ok);
          if(ok) QMessageBox::information(this,"Connect","connection was successful on a quadrocopter: "+QString::number(numberOfQuadro));
          else QMessageBox::warning(this,"Error","connection wasn unsuccessful");
}

void quadro::createScrollCheckBox()
{
    QVBoxLayout *checkLayout = new QVBoxLayout();
    Plot1Signal1 = new QCheckBox("Ground distance estimated");
    connect(Plot1Signal1,SIGNAL(clicked()),this,SLOT(Plot1Signal1_clicked()));
    Plot1Signal2 = new QCheckBox("Ground distance");
    connect(Plot1Signal2,SIGNAL(clicked()),this,SLOT(Plot1Signal2_clicked()));
    Plot1Signal3 = new QCheckBox("Elevator speed");
    connect(Plot1Signal3,SIGNAL(clicked()),this,SLOT(Plot1Signal3_clicked()));
    Plot1Signal4 = new QCheckBox("Aileron_speed");
    connect(Plot1Signal4,SIGNAL(clicked()),this,SLOT(Plot1Signal4_clicked()));
    Plot1Signal5 = new QCheckBox("Elevator speed estimated");
    connect(Plot1Signal5,SIGNAL(clicked()),this,SLOT(Plot1Signal5_clicked()));
    Plot1Signal6 = new QCheckBox("Aileron speed estimated");
    connect(Plot1Signal6,SIGNAL(clicked()),this,SLOT(Plot1Signal6_clicked()));
    Plot1Signal7 = new QCheckBox("Elevator position estimated");
    connect(Plot1Signal7,SIGNAL(clicked()),this,SLOT(Plot1Signal7_clicked()));
    Plot1Signal8 = new QCheckBox("Aileron position estimated");
    connect(Plot1Signal8,SIGNAL(clicked()),this,SLOT(Plot1Signal8_clicked()));
    Plot1Signal9 = new QCheckBox("Throttle controller output");
    connect(Plot1Signal9,SIGNAL(clicked()),this,SLOT(Plot1Signal9_clicked()));
    Plot1Signal10 = new QCheckBox("Throttle speed");
    connect(Plot1Signal10,SIGNAL(clicked()),this,SLOT(Plot1Signal10_clicked()));
    Plot1Signal11 = new QCheckBox("Aileron velocity controller output");
    connect(Plot1Signal11,SIGNAL(clicked()),this,SLOT(Plot1Signal11_clicked()));
    Plot1Signal12 = new QCheckBox("Elevator velocity controller output");
    connect(Plot1Signal12,SIGNAL(clicked()),this,SLOT(Plot1Signal12_clicked()));
    Plot1Signal13 = new QCheckBox("Aileron position controller output");
    connect(Plot1Signal13,SIGNAL(clicked()),this,SLOT(Plot1Signal13_clicked()));
    Plot1Signal14 = new QCheckBox("Elevator position controller output");
    connect(Plot1Signal14,SIGNAL(clicked()),this,SLOT(Plot1Signal14_clicked()));
    Plot1Signal15 = new QCheckBox("Throttle setpoint");
    connect(Plot1Signal15,SIGNAL(clicked()),this,SLOT(Plot1Signal15_clicked()));
    Plot1Signal16 = new QCheckBox("Elevator position setpoint");
    connect(Plot1Signal16,SIGNAL(clicked()),this,SLOT(Plot1Signal16_clicked()));
    Plot1Signal17 = new QCheckBox("Aileron position setpoint");
    connect(Plot1Signal17,SIGNAL(clicked()),this,SLOT(Plot1Signal17_clicked()));
    Plot1Signal18 = new QCheckBox("Elevator velocity setpoint");
    connect(Plot1Signal18,SIGNAL(clicked()),this,SLOT(Plot1Signal18_clicked()));
    Plot1Signal19 = new QCheckBox("Aileron velocity setpoint");
    connect(Plot1Signal19,SIGNAL(clicked()),this,SLOT(Plot1Signal19_clicked()));
    Plot1Signal20 = new QCheckBox("Elevator speed estimated 2");
    connect(Plot1Signal20,SIGNAL(clicked()),this,SLOT(Plot1Signal20_clicked()));
    Plot1Signal21 = new QCheckBox("Aileron speed estimated 2");
    connect(Plot1Signal21,SIGNAL(clicked()),this,SLOT(Plot1Signal21_clicked()));
    Plot1Signal22 = new QCheckBox("Elevator ACC");
    connect(Plot1Signal22,SIGNAL(clicked()),this,SLOT(Plot1Signal22_clicked()));
    Plot1Signal23 = new QCheckBox("Aileron ACC");
    connect(Plot1Signal23,SIGNAL(clicked()),this,SLOT(Plot1Signal23_clicked()));
    Plot1Signal24 = new QCheckBox("Valid gumstix");
    connect(Plot1Signal24,SIGNAL(clicked()),this,SLOT(Plot1Signal24_clicked()));
    Plot1Signal25 = new QCheckBox("Elevator desired speed position cont");
    connect(Plot1Signal25,SIGNAL(clicked()),this,SLOT(Plot1Signal25_clicked()));
    Plot1Signal26 = new QCheckBox("Aileron desired speed position cont");
    connect(Plot1Signal26,SIGNAL(clicked()),this,SLOT(Plot1Signal26_clicked()));
    Plot1Signal27 = new QCheckBox("Elevator desired speed pos cont leader");
    connect(Plot1Signal27,SIGNAL(clicked()),this,SLOT(Plot1Signal27_clicked()));
    Plot1Signal28 = new QCheckBox("Aileron desired speed pos cont leader");
    connect(Plot1Signal28,SIGNAL(clicked()),this,SLOT(Plot1Signal28_clicked()));
    Plot1Signal29 = new QCheckBox("Output throttle");
    connect(Plot1Signal29,SIGNAL(clicked()),this,SLOT(Plot1Signal29_clicked()));
    Plot1Signal30 = new QCheckBox("Output elevator");
    connect(Plot1Signal30,SIGNAL(clicked()),this,SLOT(Plot1Signal30_clicked()));
    Plot1Signal31 = new QCheckBox("Output aileron");
    connect(Plot1Signal31,SIGNAL(clicked()),this,SLOT(Plot1Signal31_clicked()));
    Plot1Signal32 = new QCheckBox("Output rudder");
    connect(Plot1Signal32,SIGNAL(clicked()),this,SLOT(Plot1Signal32_clicked()));
    Plot1SignalAll = new QCheckBox("All signals");
    connect(Plot1SignalAll,SIGNAL(clicked()),this,SLOT(Plot1SignalAll_clicked()));

    Plot2Signal1 = new QCheckBox("Ground distance estimated");
    connect(Plot2Signal1,SIGNAL(clicked()),this,SLOT(Plot2Signal1_clicked()));
    Plot2Signal2 = new QCheckBox("Ground distance");
    connect(Plot2Signal2,SIGNAL(clicked()),this,SLOT(Plot2Signal2_clicked()));
    Plot2Signal3 = new QCheckBox("Elevator speed");
    connect(Plot2Signal3,SIGNAL(clicked()),this,SLOT(Plot2Signal3_clicked()));
    Plot2Signal4 = new QCheckBox("Aileron_speed");
    connect(Plot2Signal4,SIGNAL(clicked()),this,SLOT(Plot2Signal4_clicked()));
    Plot2Signal5 = new QCheckBox("Elevator speed estimated");
    connect(Plot2Signal5,SIGNAL(clicked()),this,SLOT(Plot2Signal5_clicked()));
    Plot2Signal6 = new QCheckBox("Aileron speed estimated");
    connect(Plot2Signal6,SIGNAL(clicked()),this,SLOT(Plot2Signal6_clicked()));
    Plot2Signal7 = new QCheckBox("Elevator position estimated");
    connect(Plot2Signal7,SIGNAL(clicked()),this,SLOT(Plot2Signal7_clicked()));
    Plot2Signal8 = new QCheckBox("Aileron position estimated");
    connect(Plot2Signal8,SIGNAL(clicked()),this,SLOT(Plot2Signal8_clicked()));
    Plot2Signal9 = new QCheckBox("Throttle controller output");
    connect(Plot2Signal9,SIGNAL(clicked()),this,SLOT(Plot2Signal9_clicked()));
    Plot2Signal10 = new QCheckBox("Throttle speed");
    connect(Plot2Signal10,SIGNAL(clicked()),this,SLOT(Plot2Signal10_clicked()));
    Plot2Signal11 = new QCheckBox("Aileron velocity controller output");
    connect(Plot2Signal11,SIGNAL(clicked()),this,SLOT(Plot2Signal11_clicked()));
    Plot2Signal12 = new QCheckBox("Elevator velocity controller output");
    connect(Plot2Signal12,SIGNAL(clicked()),this,SLOT(Plot2Signal12_clicked()));
    Plot2Signal13 = new QCheckBox("Aileron position controller output");
    connect(Plot2Signal13,SIGNAL(clicked()),this,SLOT(Plot2Signal13_clicked()));
    Plot2Signal14 = new QCheckBox("Elevator position controller output");
    connect(Plot2Signal14,SIGNAL(clicked()),this,SLOT(Plot2Signal14_clicked()));
    Plot2Signal15 = new QCheckBox("Throttle setpoint");
    connect(Plot2Signal15,SIGNAL(clicked()),this,SLOT(Plot2Signal15_clicked()));
    Plot2Signal16 = new QCheckBox("Elevator position setpoint");
    connect(Plot2Signal16,SIGNAL(clicked()),this,SLOT(Plot2Signal16_clicked()));
    Plot2Signal17 = new QCheckBox("Aileron position setpoint");
    connect(Plot2Signal17,SIGNAL(clicked()),this,SLOT(Plot2Signal17_clicked()));
    Plot2Signal18 = new QCheckBox("Elevator velocity setpoint");
    connect(Plot2Signal18,SIGNAL(clicked()),this,SLOT(Plot2Signal18_clicked()));
    Plot2Signal19 = new QCheckBox("Aileron velocity setpoint");
    connect(Plot2Signal19,SIGNAL(clicked()),this,SLOT(Plot2Signal19_clicked()));
    Plot2Signal20 = new QCheckBox("Elevator speed estimated 2");
    connect(Plot2Signal20,SIGNAL(clicked()),this,SLOT(Plot2Signal20_clicked()));
    Plot2Signal21 = new QCheckBox("Aileron speed estimated 2");
    connect(Plot2Signal21,SIGNAL(clicked()),this,SLOT(Plot2Signal21_clicked()));
    Plot2Signal22 = new QCheckBox("Elevator ACC");
    connect(Plot2Signal22,SIGNAL(clicked()),this,SLOT(Plot2Signal22_clicked()));
    Plot2Signal23 = new QCheckBox("Aileron ACC");
    connect(Plot2Signal23,SIGNAL(clicked()),this,SLOT(Plot2Signal23_clicked()));
    Plot2Signal24 = new QCheckBox("Valid gumstix");
    connect(Plot2Signal24,SIGNAL(clicked()),this,SLOT(Plot2Signal24_clicked()));
    Plot2Signal25 = new QCheckBox("Elevator desired speed position cont");
    connect(Plot2Signal25,SIGNAL(clicked()),this,SLOT(Plot2Signal25_clicked()));
    Plot2Signal26 = new QCheckBox("Aileron desired speed position cont");
    connect(Plot2Signal26,SIGNAL(clicked()),this,SLOT(Plot2Signal26_clicked()));
    Plot2Signal27 = new QCheckBox("Elevator desired speed pos cont leader");
    connect(Plot2Signal27,SIGNAL(clicked()),this,SLOT(Plot2Signal27_clicked()));
    Plot2Signal28 = new QCheckBox("Aileron desired speed pos cont leader");
    connect(Plot2Signal28,SIGNAL(clicked()),this,SLOT(Plot2Signal28_clicked()));
    Plot2Signal29 = new QCheckBox("Output throttle");
    connect(Plot2Signal29,SIGNAL(clicked()),this,SLOT(Plot2Signal29_clicked()));
    Plot2Signal30 = new QCheckBox("Output elevator");
    connect(Plot2Signal30,SIGNAL(clicked()),this,SLOT(Plot2Signal30_clicked()));
    Plot2Signal31 = new QCheckBox("Output aileron");
    connect(Plot2Signal31,SIGNAL(clicked()),this,SLOT(Plot2Signal31_clicked()));
    Plot2Signal32 = new QCheckBox("Output rudder");
    connect(Plot2Signal32,SIGNAL(clicked()),this,SLOT(Plot2Signal32_clicked()));
    Plot2SignalAll = new QCheckBox("All signals");
    connect(Plot2SignalAll,SIGNAL(clicked()),this,SLOT(Plot2SignalAll_clicked()));

    Plot3Signal1 = new QCheckBox("Ground distance estimated");
    connect(Plot3Signal1,SIGNAL(clicked()),this,SLOT(Plot3Signal1_clicked()));
    Plot3Signal2 = new QCheckBox("Ground distance");
    connect(Plot3Signal2,SIGNAL(clicked()),this,SLOT(Plot3Signal2_clicked()));
    Plot3Signal3 = new QCheckBox("Elevator speed");
    connect(Plot3Signal3,SIGNAL(clicked()),this,SLOT(Plot3Signal3_clicked()));
    Plot3Signal4 = new QCheckBox("Aileron_speed");
    connect(Plot3Signal4,SIGNAL(clicked()),this,SLOT(Plot3Signal4_clicked()));
    Plot3Signal5 = new QCheckBox("Elevator speed estimated");
    connect(Plot3Signal5,SIGNAL(clicked()),this,SLOT(Plot3Signal5_clicked()));
    Plot3Signal6 = new QCheckBox("Aileron speed estimated");
    connect(Plot3Signal6,SIGNAL(clicked()),this,SLOT(Plot3Signal6_clicked()));
    Plot3Signal7 = new QCheckBox("Elevator position estimated");
    connect(Plot3Signal7,SIGNAL(clicked()),this,SLOT(Plot3Signal7_clicked()));
    Plot3Signal8 = new QCheckBox("Aileron position estimated");
    connect(Plot3Signal8,SIGNAL(clicked()),this,SLOT(Plot3Signal8_clicked()));
    Plot3Signal9 = new QCheckBox("Throttle controller output");
    connect(Plot3Signal9,SIGNAL(clicked()),this,SLOT(Plot3Signal9_clicked()));
    Plot3Signal10 = new QCheckBox("Throttle speed");
    connect(Plot3Signal10,SIGNAL(clicked()),this,SLOT(Plot3Signal10_clicked()));
    Plot3Signal11 = new QCheckBox("Aileron velocity controller output");
    connect(Plot3Signal11,SIGNAL(clicked()),this,SLOT(Plot3Signal11_clicked()));
    Plot3Signal12 = new QCheckBox("Elevator velocity controller output");
    connect(Plot3Signal12,SIGNAL(clicked()),this,SLOT(Plot3Signal12_clicked()));
    Plot3Signal13 = new QCheckBox("Aileron position controller output");
    connect(Plot3Signal13,SIGNAL(clicked()),this,SLOT(Plot3Signal13_clicked()));
    Plot3Signal14 = new QCheckBox("Elevator position controller output");
    connect(Plot3Signal14,SIGNAL(clicked()),this,SLOT(Plot3Signal14_clicked()));
    Plot3Signal15 = new QCheckBox("Throttle setpoint");
    connect(Plot3Signal15,SIGNAL(clicked()),this,SLOT(Plot3Signal15_clicked()));
    Plot3Signal16 = new QCheckBox("Elevator position setpoint");
    connect(Plot3Signal16,SIGNAL(clicked()),this,SLOT(Plot3Signal16_clicked()));
    Plot3Signal17 = new QCheckBox("Aileron position setpoint");
    connect(Plot3Signal17,SIGNAL(clicked()),this,SLOT(Plot3Signal17_clicked()));
    Plot3Signal18 = new QCheckBox("Elevator velocity setpoint");
    connect(Plot3Signal18,SIGNAL(clicked()),this,SLOT(Plot3Signal18_clicked()));
    Plot3Signal19 = new QCheckBox("Aileron velocity setpoint");
    connect(Plot3Signal19,SIGNAL(clicked()),this,SLOT(Plot3Signal19_clicked()));
    Plot3Signal20 = new QCheckBox("Elevator speed estimated 2");
    connect(Plot3Signal20,SIGNAL(clicked()),this,SLOT(Plot3Signal20_clicked()));
    Plot3Signal21 = new QCheckBox("Aileron speed estimated 2");
    connect(Plot3Signal21,SIGNAL(clicked()),this,SLOT(Plot3Signal21_clicked()));
    Plot3Signal22 = new QCheckBox("Elevator ACC");
    connect(Plot3Signal22,SIGNAL(clicked()),this,SLOT(Plot3Signal22_clicked()));
    Plot3Signal23 = new QCheckBox("Aileron ACC");
    connect(Plot3Signal23,SIGNAL(clicked()),this,SLOT(Plot3Signal23_clicked()));
    Plot3Signal24 = new QCheckBox("Valid gumstix");
    connect(Plot3Signal24,SIGNAL(clicked()),this,SLOT(Plot3Signal24_clicked()));
    Plot3Signal25 = new QCheckBox("Elevator desired speed position cont");
    connect(Plot3Signal25,SIGNAL(clicked()),this,SLOT(Plot3Signal25_clicked()));
    Plot3Signal26 = new QCheckBox("Aileron desired speed position cont");
    connect(Plot3Signal26,SIGNAL(clicked()),this,SLOT(Plot3Signal26_clicked()));
    Plot3Signal27 = new QCheckBox("Elevator desired speed pos cont leader");
    connect(Plot3Signal27,SIGNAL(clicked()),this,SLOT(Plot3Signal27_clicked()));
    Plot3Signal28 = new QCheckBox("Aileron desired speed pos cont leader");
    connect(Plot3Signal28,SIGNAL(clicked()),this,SLOT(Plot3Signal28_clicked()));
    Plot3Signal29 = new QCheckBox("Output throttle");
    connect(Plot3Signal29,SIGNAL(clicked()),this,SLOT(Plot3Signal29_clicked()));
    Plot3Signal30 = new QCheckBox("Output elevator");
    connect(Plot3Signal30,SIGNAL(clicked()),this,SLOT(Plot3Signal30_clicked()));
    Plot3Signal31 = new QCheckBox("Output aileron");
    connect(Plot3Signal31,SIGNAL(clicked()),this,SLOT(Plot3Signal31_clicked()));
    Plot3Signal32 = new QCheckBox("Output rudder");
    connect(Plot3Signal32,SIGNAL(clicked()),this,SLOT(Plot3Signal32_clicked()));
    Plot3SignalAll = new QCheckBox("All signals");
    connect(Plot3SignalAll,SIGNAL(clicked()),this,SLOT(Plot3SignalAll_clicked()));

    Plot4Signal1 = new QCheckBox("Ground distance estimated");
    connect(Plot4Signal1,SIGNAL(clicked()),this,SLOT(Plot4Signal1_clicked()));
    Plot4Signal2 = new QCheckBox("Ground distance");
    connect(Plot4Signal2,SIGNAL(clicked()),this,SLOT(Plot4Signal2_clicked()));
    Plot4Signal3 = new QCheckBox("Elevator speed");
    connect(Plot4Signal3,SIGNAL(clicked()),this,SLOT(Plot4Signal3_clicked()));
    Plot4Signal4 = new QCheckBox("Aileron_speed");
    connect(Plot4Signal4,SIGNAL(clicked()),this,SLOT(Plot4Signal4_clicked()));
    Plot4Signal5 = new QCheckBox("Elevator speed estimated");
    connect(Plot4Signal5,SIGNAL(clicked()),this,SLOT(Plot4Signal5_clicked()));
    Plot4Signal6 = new QCheckBox("Aileron speed estimated");
    connect(Plot4Signal6,SIGNAL(clicked()),this,SLOT(Plot4Signal6_clicked()));
    Plot4Signal7 = new QCheckBox("Elevator position estimated");
    connect(Plot4Signal7,SIGNAL(clicked()),this,SLOT(Plot4Signal7_clicked()));
    Plot4Signal8 = new QCheckBox("Aileron position estimated");
    connect(Plot4Signal8,SIGNAL(clicked()),this,SLOT(Plot4Signal8_clicked()));
    Plot4Signal9 = new QCheckBox("Throttle controller output");
    connect(Plot4Signal9,SIGNAL(clicked()),this,SLOT(Plot4Signal9_clicked()));
    Plot4Signal10 = new QCheckBox("Throttle speed");
    connect(Plot4Signal10,SIGNAL(clicked()),this,SLOT(Plot4Signal10_clicked()));
    Plot4Signal11 = new QCheckBox("Aileron velocity controller output");
    connect(Plot4Signal11,SIGNAL(clicked()),this,SLOT(Plot4Signal11_clicked()));
    Plot4Signal12 = new QCheckBox("Elevator velocity controller output");
    connect(Plot4Signal12,SIGNAL(clicked()),this,SLOT(Plot4Signal12_clicked()));
    Plot4Signal13 = new QCheckBox("Aileron position controller output");
    connect(Plot4Signal13,SIGNAL(clicked()),this,SLOT(Plot4Signal13_clicked()));
    Plot4Signal14 = new QCheckBox("Elevator position controller output");
    connect(Plot4Signal14,SIGNAL(clicked()),this,SLOT(Plot4Signal14_clicked()));
    Plot4Signal15 = new QCheckBox("Throttle setpoint");
    connect(Plot4Signal15,SIGNAL(clicked()),this,SLOT(Plot4Signal15_clicked()));
    Plot4Signal16 = new QCheckBox("Elevator position setpoint");
    connect(Plot4Signal16,SIGNAL(clicked()),this,SLOT(Plot4Signal16_clicked()));
    Plot4Signal17 = new QCheckBox("Aileron position setpoint");
    connect(Plot4Signal17,SIGNAL(clicked()),this,SLOT(Plot4Signal17_clicked()));
    Plot4Signal18 = new QCheckBox("Elevator velocity setpoint");
    connect(Plot4Signal18,SIGNAL(clicked()),this,SLOT(Plot4Signal18_clicked()));
    Plot4Signal19 = new QCheckBox("Aileron velocity setpoint");
    connect(Plot4Signal19,SIGNAL(clicked()),this,SLOT(Plot4Signal19_clicked()));
    Plot4Signal20 = new QCheckBox("Elevator speed estimated 2");
    connect(Plot4Signal20,SIGNAL(clicked()),this,SLOT(Plot4Signal20_clicked()));
    Plot4Signal21 = new QCheckBox("Aileron speed estimated 2");
    connect(Plot4Signal21,SIGNAL(clicked()),this,SLOT(Plot4Signal21_clicked()));
    Plot4Signal22 = new QCheckBox("Elevator ACC");
    connect(Plot4Signal22,SIGNAL(clicked()),this,SLOT(Plot4Signal22_clicked()));
    Plot4Signal23 = new QCheckBox("Aileron ACC");
    connect(Plot4Signal23,SIGNAL(clicked()),this,SLOT(Plot4Signal23_clicked()));
    Plot4Signal24 = new QCheckBox("Valid gumstix");
    connect(Plot4Signal24,SIGNAL(clicked()),this,SLOT(Plot4Signal24_clicked()));
    Plot4Signal25 = new QCheckBox("Elevator desired speed position cont");
    connect(Plot4Signal25,SIGNAL(clicked()),this,SLOT(Plot4Signal25_clicked()));
    Plot4Signal26 = new QCheckBox("Aileron desired speed position cont");
    connect(Plot4Signal26,SIGNAL(clicked()),this,SLOT(Plot4Signal26_clicked()));
    Plot4Signal27 = new QCheckBox("Elevator desired speed pos cont leader");
    connect(Plot4Signal27,SIGNAL(clicked()),this,SLOT(Plot4Signal27_clicked()));
    Plot4Signal28 = new QCheckBox("Aileron desired speed pos cont leader");
    connect(Plot4Signal28,SIGNAL(clicked()),this,SLOT(Plot4Signal28_clicked()));
    Plot4Signal29 = new QCheckBox("Output throttle");
    connect(Plot4Signal29,SIGNAL(clicked()),this,SLOT(Plot4Signal29_clicked()));
    Plot4Signal30 = new QCheckBox("Output elevator");
    connect(Plot4Signal30,SIGNAL(clicked()),this,SLOT(Plot4Signal30_clicked()));
    Plot4Signal31 = new QCheckBox("Output aileron");
    connect(Plot4Signal31,SIGNAL(clicked()),this,SLOT(Plot4Signal31_clicked()));
    Plot4Signal32 = new QCheckBox("Output rudder");
    connect(Plot4Signal32,SIGNAL(clicked()),this,SLOT(Plot4Signal32_clicked()));
    Plot4SignalAll = new QCheckBox("All signals");
    connect(Plot4SignalAll,SIGNAL(clicked()),this,SLOT(Plot4SignalAll_clicked()));

    checkLayout->addWidget(Plot1Signal1);
    checkLayout->addWidget(Plot1Signal2);
    checkLayout->addWidget(Plot1Signal3);
    checkLayout->addWidget(Plot1Signal4);
    checkLayout->addWidget(Plot1Signal5);
    checkLayout->addWidget(Plot1Signal6);
    checkLayout->addWidget(Plot1Signal7);
    checkLayout->addWidget(Plot1Signal8);
    checkLayout->addWidget(Plot1Signal9);
    checkLayout->addWidget(Plot1Signal10);
    checkLayout->addWidget(Plot1Signal11);
    checkLayout->addWidget(Plot1Signal12);
    checkLayout->addWidget(Plot1Signal13);
    checkLayout->addWidget(Plot1Signal14);
    checkLayout->addWidget(Plot1Signal15);
    checkLayout->addWidget(Plot1Signal16);
    checkLayout->addWidget(Plot1Signal17);
    checkLayout->addWidget(Plot1Signal18);
    checkLayout->addWidget(Plot1Signal19);
    checkLayout->addWidget(Plot1Signal20);
    checkLayout->addWidget(Plot1Signal21);
    checkLayout->addWidget(Plot1Signal22);
    checkLayout->addWidget(Plot1Signal23);
    checkLayout->addWidget(Plot1Signal24);
    checkLayout->addWidget(Plot1Signal25);
    checkLayout->addWidget(Plot1Signal26);
    checkLayout->addWidget(Plot1Signal27);
    checkLayout->addWidget(Plot1Signal28);
    checkLayout->addWidget(Plot1Signal29);
    checkLayout->addWidget(Plot1Signal30);
    checkLayout->addWidget(Plot1Signal31);
    checkLayout->addWidget(Plot1Signal32);
    checkLayout->addWidget(Plot1SignalAll);

    checkLayout->addWidget(Plot2Signal1);
    checkLayout->addWidget(Plot2Signal2);
    checkLayout->addWidget(Plot2Signal3);
    checkLayout->addWidget(Plot2Signal4);
    checkLayout->addWidget(Plot2Signal5);
    checkLayout->addWidget(Plot2Signal6);
    checkLayout->addWidget(Plot2Signal7);
    checkLayout->addWidget(Plot2Signal8);
    checkLayout->addWidget(Plot2Signal9);
    checkLayout->addWidget(Plot2Signal10);
    checkLayout->addWidget(Plot2Signal11);
    checkLayout->addWidget(Plot2Signal12);
    checkLayout->addWidget(Plot2Signal13);
    checkLayout->addWidget(Plot2Signal14);
    checkLayout->addWidget(Plot2Signal15);
    checkLayout->addWidget(Plot2Signal16);
    checkLayout->addWidget(Plot2Signal17);
    checkLayout->addWidget(Plot2Signal18);
    checkLayout->addWidget(Plot2Signal19);
    checkLayout->addWidget(Plot2Signal20);
    checkLayout->addWidget(Plot2Signal21);
    checkLayout->addWidget(Plot2Signal22);
    checkLayout->addWidget(Plot2Signal23);
    checkLayout->addWidget(Plot2Signal24);
    checkLayout->addWidget(Plot2Signal25);
    checkLayout->addWidget(Plot2Signal26);
    checkLayout->addWidget(Plot2Signal27);
    checkLayout->addWidget(Plot2Signal28);
    checkLayout->addWidget(Plot2Signal29);
    checkLayout->addWidget(Plot2Signal30);
    checkLayout->addWidget(Plot2Signal31);
    checkLayout->addWidget(Plot2Signal32);
    checkLayout->addWidget(Plot2SignalAll);

    checkLayout->addWidget(Plot3Signal1);
    checkLayout->addWidget(Plot3Signal2);
    checkLayout->addWidget(Plot3Signal3);
    checkLayout->addWidget(Plot3Signal4);
    checkLayout->addWidget(Plot3Signal5);
    checkLayout->addWidget(Plot3Signal6);
    checkLayout->addWidget(Plot3Signal7);
    checkLayout->addWidget(Plot3Signal8);
    checkLayout->addWidget(Plot3Signal9);
    checkLayout->addWidget(Plot3Signal10);
    checkLayout->addWidget(Plot3Signal11);
    checkLayout->addWidget(Plot3Signal12);
    checkLayout->addWidget(Plot3Signal13);
    checkLayout->addWidget(Plot3Signal14);
    checkLayout->addWidget(Plot3Signal15);
    checkLayout->addWidget(Plot3Signal16);
    checkLayout->addWidget(Plot3Signal17);
    checkLayout->addWidget(Plot3Signal18);
    checkLayout->addWidget(Plot3Signal19);
    checkLayout->addWidget(Plot3Signal20);
    checkLayout->addWidget(Plot3Signal21);
    checkLayout->addWidget(Plot3Signal22);
    checkLayout->addWidget(Plot3Signal23);
    checkLayout->addWidget(Plot3Signal24);
    checkLayout->addWidget(Plot3Signal25);
    checkLayout->addWidget(Plot3Signal26);
    checkLayout->addWidget(Plot3Signal27);
    checkLayout->addWidget(Plot3Signal28);
    checkLayout->addWidget(Plot3Signal29);
    checkLayout->addWidget(Plot3Signal30);
    checkLayout->addWidget(Plot3Signal31);
    checkLayout->addWidget(Plot3Signal32);
    checkLayout->addWidget(Plot3SignalAll);

    checkLayout->addWidget(Plot4Signal1);
    checkLayout->addWidget(Plot4Signal2);
    checkLayout->addWidget(Plot4Signal3);
    checkLayout->addWidget(Plot4Signal4);
    checkLayout->addWidget(Plot4Signal5);
    checkLayout->addWidget(Plot4Signal6);
    checkLayout->addWidget(Plot4Signal7);
    checkLayout->addWidget(Plot4Signal8);
    checkLayout->addWidget(Plot4Signal9);
    checkLayout->addWidget(Plot4Signal10);
    checkLayout->addWidget(Plot4Signal11);
    checkLayout->addWidget(Plot4Signal12);
    checkLayout->addWidget(Plot4Signal13);
    checkLayout->addWidget(Plot4Signal14);
    checkLayout->addWidget(Plot4Signal15);
    checkLayout->addWidget(Plot4Signal16);
    checkLayout->addWidget(Plot4Signal17);
    checkLayout->addWidget(Plot4Signal18);
    checkLayout->addWidget(Plot4Signal19);
    checkLayout->addWidget(Plot4Signal20);
    checkLayout->addWidget(Plot4Signal21);
    checkLayout->addWidget(Plot4Signal22);
    checkLayout->addWidget(Plot4Signal23);
    checkLayout->addWidget(Plot4Signal24);
    checkLayout->addWidget(Plot4Signal25);
    checkLayout->addWidget(Plot4Signal26);
    checkLayout->addWidget(Plot4Signal27);
    checkLayout->addWidget(Plot4Signal28);
    checkLayout->addWidget(Plot4Signal29);
    checkLayout->addWidget(Plot4Signal30);
    checkLayout->addWidget(Plot4Signal31);
    checkLayout->addWidget(Plot4Signal32);
    checkLayout->addWidget(Plot4SignalAll);

    ui->scrollContents->setLayout(checkLayout);

}

void quadro::setAllCheckFalse()
{
            Plot1Signal1->setVisible(false);
            Plot1Signal2->setVisible(false);
            Plot1Signal3->setVisible(false);
            Plot1Signal4->setVisible(false);
            Plot1Signal5->setVisible(false);
            Plot1Signal6->setVisible(false);
            Plot1Signal7->setVisible(false);
            Plot1Signal8->setVisible(false);
            Plot1Signal9->setVisible(false);
            Plot1Signal10->setVisible(false);
            Plot1Signal11->setVisible(false);
            Plot1Signal12->setVisible(false);
            Plot1Signal13->setVisible(false);
            Plot1Signal14->setVisible(false);
            Plot1Signal15->setVisible(false);
            Plot1Signal16->setVisible(false);
            Plot1Signal17->setVisible(false);
            Plot1Signal18->setVisible(false);
            Plot1Signal19->setVisible(false);
            Plot1Signal20->setVisible(false);
            Plot1Signal21->setVisible(false);
            Plot1Signal22->setVisible(false);
            Plot1Signal23->setVisible(false);
            Plot1Signal24->setVisible(false);
            Plot1Signal25->setVisible(false);
            Plot1Signal26->setVisible(false);
            Plot1Signal27->setVisible(false);
            Plot1Signal28->setVisible(false);
            Plot1Signal29->setVisible(false);
            Plot1Signal30->setVisible(false);
            Plot1Signal31->setVisible(false);
            Plot1Signal32->setVisible(false);
            Plot1SignalAll->setVisible(false);

            Plot2Signal1->setVisible(false);
            Plot2Signal2->setVisible(false);
            Plot2Signal3->setVisible(false);
            Plot2Signal4->setVisible(false);
            Plot2Signal5->setVisible(false);
            Plot2Signal6->setVisible(false);
            Plot2Signal7->setVisible(false);
            Plot2Signal8->setVisible(false);
            Plot2Signal9->setVisible(false);
            Plot2Signal10->setVisible(false);
            Plot2Signal11->setVisible(false);
            Plot2Signal12->setVisible(false);
            Plot2Signal13->setVisible(false);
            Plot2Signal14->setVisible(false);
            Plot2Signal15->setVisible(false);
            Plot2Signal16->setVisible(false);
            Plot2Signal17->setVisible(false);
            Plot2Signal18->setVisible(false);
            Plot2Signal19->setVisible(false);
            Plot2Signal20->setVisible(false);
            Plot2Signal21->setVisible(false);
            Plot2Signal22->setVisible(false);
            Plot2Signal23->setVisible(false);
            Plot2Signal24->setVisible(false);
            Plot2Signal25->setVisible(false);
            Plot2Signal26->setVisible(false);
            Plot2Signal27->setVisible(false);
            Plot2Signal28->setVisible(false);
            Plot2Signal29->setVisible(false);
            Plot2Signal30->setVisible(false);
            Plot2Signal31->setVisible(false);
            Plot2Signal32->setVisible(false);
            Plot2SignalAll->setVisible(false);

            Plot3Signal1->setVisible(false);
            Plot3Signal2->setVisible(false);
            Plot3Signal3->setVisible(false);
            Plot3Signal4->setVisible(false);
            Plot3Signal5->setVisible(false);
            Plot3Signal6->setVisible(false);
            Plot3Signal7->setVisible(false);
            Plot3Signal8->setVisible(false);
            Plot3Signal9->setVisible(false);
            Plot3Signal10->setVisible(false);
            Plot3Signal11->setVisible(false);
            Plot3Signal12->setVisible(false);
            Plot3Signal13->setVisible(false);
            Plot3Signal14->setVisible(false);
            Plot3Signal15->setVisible(false);
            Plot3Signal16->setVisible(false);
            Plot3Signal17->setVisible(false);
            Plot3Signal18->setVisible(false);
            Plot3Signal19->setVisible(false);
            Plot3Signal20->setVisible(false);
            Plot3Signal21->setVisible(false);
            Plot3Signal22->setVisible(false);
            Plot3Signal23->setVisible(false);
            Plot3Signal24->setVisible(false);
            Plot3Signal25->setVisible(false);
            Plot3Signal26->setVisible(false);
            Plot3Signal27->setVisible(false);
            Plot3Signal28->setVisible(false);
            Plot3Signal29->setVisible(false);
            Plot3Signal30->setVisible(false);
            Plot3Signal31->setVisible(false);
            Plot3Signal32->setVisible(false);
            Plot3SignalAll->setVisible(false);

            Plot4Signal1->setVisible(false);
            Plot4Signal2->setVisible(false);
            Plot4Signal3->setVisible(false);
            Plot4Signal4->setVisible(false);
            Plot4Signal5->setVisible(false);
            Plot4Signal6->setVisible(false);
            Plot4Signal7->setVisible(false);
            Plot4Signal8->setVisible(false);
            Plot4Signal9->setVisible(false);
            Plot4Signal10->setVisible(false);
            Plot4Signal11->setVisible(false);
            Plot4Signal12->setVisible(false);
            Plot4Signal13->setVisible(false);
            Plot4Signal14->setVisible(false);
            Plot4Signal15->setVisible(false);
            Plot4Signal16->setVisible(false);
            Plot4Signal17->setVisible(false);
            Plot4Signal18->setVisible(false);
            Plot4Signal19->setVisible(false);
            Plot4Signal20->setVisible(false);
            Plot4Signal21->setVisible(false);
            Plot4Signal22->setVisible(false);
            Plot4Signal23->setVisible(false);
            Plot4Signal24->setVisible(false);
            Plot4Signal25->setVisible(false);
            Plot4Signal26->setVisible(false);
            Plot4Signal27->setVisible(false);
            Plot4Signal28->setVisible(false);
            Plot4Signal29->setVisible(false);
            Plot4Signal30->setVisible(false);
            Plot4Signal31->setVisible(false);
            Plot4Signal32->setVisible(false);
            Plot4SignalAll->setVisible(false);
        }
void quadro::on_comboBox_activated(int index)
{
    if(index==0){
        setAllCheckFalse();
        Plot1Signal1->setVisible(true);
        Plot1Signal2->setVisible(true);
        Plot1Signal3->setVisible(true);
        Plot1Signal4->setVisible(true);
        Plot1Signal5->setVisible(true);
        Plot1Signal6->setVisible(true);
        Plot1Signal7->setVisible(true);
        Plot1Signal8->setVisible(true);
        Plot1Signal9->setVisible(true);
        Plot1Signal10->setVisible(true);
        Plot1Signal11->setVisible(true);
        Plot1Signal12->setVisible(true);
        Plot1Signal13->setVisible(true);
        Plot1Signal14->setVisible(true);
        Plot1Signal15->setVisible(true);
        Plot1Signal16->setVisible(true);
        Plot1Signal17->setVisible(true);
        Plot1Signal18->setVisible(true);
        Plot1Signal19->setVisible(true);
        Plot1Signal20->setVisible(true);
        Plot1Signal21->setVisible(true);
        Plot1Signal22->setVisible(true);
        Plot1Signal23->setVisible(true);
        Plot1Signal24->setVisible(true);
        Plot1Signal25->setVisible(true);
        Plot1Signal26->setVisible(true);
        Plot1Signal27->setVisible(true);
        Plot1Signal28->setVisible(true);
        Plot1Signal29->setVisible(true);
        Plot1Signal30->setVisible(true);
        Plot1Signal31->setVisible(true);
        Plot1Signal32->setVisible(true);
        Plot1SignalAll->setVisible(true);
    }
    else if(index==1){
        setAllCheckFalse();
        Plot2Signal1->setVisible(true);
        Plot2Signal2->setVisible(true);
        Plot2Signal3->setVisible(true);
        Plot2Signal4->setVisible(true);
        Plot2Signal5->setVisible(true);
        Plot2Signal6->setVisible(true);
        Plot2Signal7->setVisible(true);
        Plot2Signal8->setVisible(true);
        Plot2Signal9->setVisible(true);
        Plot2Signal10->setVisible(true);
        Plot2Signal11->setVisible(true);
        Plot2Signal12->setVisible(true);
        Plot2Signal13->setVisible(true);
        Plot2Signal14->setVisible(true);
        Plot2Signal15->setVisible(true);
        Plot2Signal16->setVisible(true);
        Plot2Signal17->setVisible(true);
        Plot2Signal18->setVisible(true);
        Plot2Signal19->setVisible(true);
        Plot2Signal20->setVisible(true);
        Plot2Signal21->setVisible(true);
        Plot2Signal22->setVisible(true);
        Plot2Signal23->setVisible(true);
        Plot2Signal24->setVisible(true);
        Plot2Signal25->setVisible(true);
        Plot2Signal26->setVisible(true);
        Plot2Signal27->setVisible(true);
        Plot2Signal28->setVisible(true);
        Plot2Signal29->setVisible(true);
        Plot2Signal30->setVisible(true);
        Plot2Signal31->setVisible(true);
        Plot2Signal32->setVisible(true);
        Plot2SignalAll->setVisible(true);
    }
    else if(index==2){
        setAllCheckFalse();
        Plot3Signal1->setVisible(true);
        Plot3Signal2->setVisible(true);
        Plot3Signal3->setVisible(true);
        Plot3Signal4->setVisible(true);
        Plot3Signal5->setVisible(true);
        Plot3Signal6->setVisible(true);
        Plot3Signal7->setVisible(true);
        Plot3Signal8->setVisible(true);
        Plot3Signal9->setVisible(true);
        Plot3Signal10->setVisible(true);
        Plot3Signal11->setVisible(true);
        Plot3Signal12->setVisible(true);
        Plot3Signal13->setVisible(true);
        Plot3Signal14->setVisible(true);
        Plot3Signal15->setVisible(true);
        Plot3Signal16->setVisible(true);
        Plot3Signal17->setVisible(true);
        Plot3Signal18->setVisible(true);
        Plot3Signal19->setVisible(true);
        Plot3Signal20->setVisible(true);
        Plot3Signal21->setVisible(true);
        Plot3Signal22->setVisible(true);
        Plot3Signal23->setVisible(true);
        Plot3Signal24->setVisible(true);
        Plot3Signal25->setVisible(true);
        Plot3Signal26->setVisible(true);
        Plot3Signal27->setVisible(true);
        Plot3Signal28->setVisible(true);
        Plot3Signal29->setVisible(true);
        Plot3Signal30->setVisible(true);
        Plot3Signal31->setVisible(true);
        Plot3Signal32->setVisible(true);
        Plot3SignalAll->setVisible(true);
    }
    else if(index==3){
        setAllCheckFalse();
        Plot4Signal1->setVisible(true);
        Plot4Signal2->setVisible(true);
        Plot4Signal3->setVisible(true);
        Plot4Signal4->setVisible(true);
        Plot4Signal5->setVisible(true);
        Plot4Signal6->setVisible(true);
        Plot4Signal7->setVisible(true);
        Plot4Signal8->setVisible(true);
        Plot4Signal9->setVisible(true);
        Plot4Signal10->setVisible(true);
        Plot4Signal11->setVisible(true);
        Plot4Signal12->setVisible(true);
        Plot4Signal13->setVisible(true);
        Plot4Signal14->setVisible(true);
        Plot4Signal15->setVisible(true);
        Plot4Signal16->setVisible(true);
        Plot4Signal17->setVisible(true);
        Plot4Signal18->setVisible(true);
        Plot4Signal19->setVisible(true);
        Plot4Signal20->setVisible(true);
        Plot4Signal21->setVisible(true);
        Plot4Signal22->setVisible(true);
        Plot4Signal23->setVisible(true);
        Plot4Signal24->setVisible(true);
        Plot4Signal25->setVisible(true);
        Plot4Signal26->setVisible(true);
        Plot4Signal27->setVisible(true);
        Plot4Signal28->setVisible(true);
        Plot4Signal29->setVisible(true);
        Plot4Signal30->setVisible(true);
        Plot4Signal31->setVisible(true);
        Plot4Signal32->setVisible(true);
        Plot4SignalAll->setVisible(true);
    }
}

void quadro::Plot1SignalAll_clicked()
{
    if(Plot1SignalAll->isChecked()){
        Plot1Signal1->setChecked(true);
        Plot1Signal2->setChecked(true);
        Plot1Signal3->setChecked(true);
        Plot1Signal4->setChecked(true);
        Plot1Signal5->setChecked(true);
        Plot1Signal6->setChecked(true);
        Plot1Signal7->setChecked(true);
        Plot1Signal8->setChecked(true);
        Plot1Signal9->setChecked(true);
        Plot1Signal10->setChecked(true);
        Plot1Signal11->setChecked(true);
        Plot1Signal12->setChecked(true);
        Plot1Signal13->setChecked(true);
        Plot1Signal14->setChecked(true);
        Plot1Signal15->setChecked(true);
        Plot1Signal16->setChecked(true);
        Plot1Signal17->setChecked(true);
        Plot1Signal18->setChecked(true);
        Plot1Signal19->setChecked(true);
        Plot1Signal20->setChecked(true);
        Plot1Signal21->setChecked(true);
        Plot1Signal22->setChecked(true);
        Plot1Signal23->setChecked(true);
        Plot1Signal24->setChecked(true);
        Plot1Signal25->setChecked(true);
        Plot1Signal26->setChecked(true);
        Plot1Signal27->setChecked(true);
        Plot1Signal28->setChecked(true);
        Plot1Signal29->setChecked(true);
        Plot1Signal30->setChecked(true);
        Plot1Signal31->setChecked(true);
        Plot1Signal32->setChecked(true);

    }else{
        Plot1Signal1->setChecked(false);
        Plot1Signal2->setChecked(false);
        Plot1Signal3->setChecked(false);
        Plot1Signal4->setChecked(false);
        Plot1Signal5->setChecked(false);
        Plot1Signal6->setChecked(false);
        Plot1Signal7->setChecked(false);
        Plot1Signal8->setChecked(false);
        Plot1Signal9->setChecked(false);
        Plot1Signal10->setChecked(false);
        Plot1Signal11->setChecked(false);
        Plot1Signal12->setChecked(false);
        Plot1Signal13->setChecked(false);
        Plot1Signal14->setChecked(false);
        Plot1Signal15->setChecked(false);
        Plot1Signal16->setChecked(false);
        Plot1Signal17->setChecked(false);
        Plot1Signal18->setChecked(false);
        Plot1Signal19->setChecked(false);
        Plot1Signal20->setChecked(false);
        Plot1Signal21->setChecked(false);
        Plot1Signal22->setChecked(false);
        Plot1Signal23->setChecked(false);
        Plot1Signal24->setChecked(false);
        Plot1Signal25->setChecked(false);
        Plot1Signal26->setChecked(false);
        Plot1Signal27->setChecked(false);
        Plot1Signal28->setChecked(false);
        Plot1Signal29->setChecked(false);
        Plot1Signal30->setChecked(false);
        Plot1Signal31->setChecked(false);
        Plot1Signal32->setChecked(false);

    }
}

void quadro::Plot1Signal1_clicked()
{
    if(!Plot1Signal1->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal2_clicked()
{
    if(!Plot1Signal2->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal3_clicked()
{
    if(!Plot1Signal3->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal4_clicked()
{
    if(!Plot1Signal4->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal5_clicked()
{
    if(!Plot1Signal5->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal6_clicked()
{
    if(!Plot1Signal6->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal7_clicked()
{
    if(!Plot1Signal7->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal8_clicked()
{
    if(!Plot1Signal8->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal9_clicked()
{
    if(!Plot1Signal9->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal10_clicked()
{
    if(!Plot1Signal10->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal11_clicked()
{
    if(!Plot1Signal11->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal12_clicked()
{
    if(!Plot1Signal12->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal13_clicked()
{
    if(!Plot1Signal13->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal14_clicked()
{
    if(!Plot1Signal14->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal15_clicked()
{
    if(!Plot1Signal15->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal16_clicked()
{
    if(!Plot1Signal16->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal17_clicked()
{
    if(!Plot1Signal17->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal18_clicked()
{
    if(!Plot1Signal18->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal19_clicked()
{
    if(!Plot1Signal19->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal20_clicked()
{
    if(!Plot1Signal20->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal21_clicked()
{
    if(!Plot1Signal21->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal22_clicked()
{
    if(!Plot1Signal22->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal23_clicked()
{
    if(!Plot1Signal23->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal24_clicked()
{
    if(!Plot1Signal24->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal25_clicked()
{
    if(!Plot1Signal25->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal26_clicked()
{
    if(!Plot1Signal26->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal27_clicked()
{
    if(!Plot1Signal27->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal28_clicked()
{
    if(!Plot1Signal28->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal29_clicked()
{
    if(!Plot1Signal29->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal30_clicked()
{
    if(!Plot1Signal30->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal31_clicked()
{
    if(!Plot1Signal31->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}
void quadro::Plot1Signal32_clicked()
{
    if(!Plot1Signal32->isChecked()){
        Plot1SignalAll->setChecked(false);
    }
}

void quadro::Plot2SignalAll_clicked()
{
    if(Plot2SignalAll->isChecked()){
        Plot2Signal1->setChecked(true);
        Plot2Signal2->setChecked(true);
        Plot2Signal3->setChecked(true);
        Plot2Signal4->setChecked(true);
        Plot2Signal5->setChecked(true);
        Plot2Signal6->setChecked(true);
        Plot2Signal7->setChecked(true);
        Plot2Signal8->setChecked(true);
        Plot2Signal9->setChecked(true);
        Plot2Signal10->setChecked(true);
        Plot2Signal11->setChecked(true);
        Plot2Signal12->setChecked(true);
        Plot2Signal13->setChecked(true);
        Plot2Signal14->setChecked(true);
        Plot2Signal15->setChecked(true);
        Plot2Signal16->setChecked(true);
        Plot2Signal17->setChecked(true);
        Plot2Signal18->setChecked(true);
        Plot2Signal19->setChecked(true);
        Plot2Signal20->setChecked(true);
        Plot2Signal21->setChecked(true);
        Plot2Signal22->setChecked(true);
        Plot2Signal23->setChecked(true);
        Plot2Signal24->setChecked(true);
        Plot2Signal25->setChecked(true);
        Plot2Signal26->setChecked(true);
        Plot2Signal27->setChecked(true);
        Plot2Signal28->setChecked(true);
        Plot2Signal29->setChecked(true);
        Plot2Signal30->setChecked(true);
        Plot2Signal31->setChecked(true);
        Plot2Signal32->setChecked(true);

    }else{
        Plot2Signal1->setChecked(false);
        Plot2Signal2->setChecked(false);
        Plot2Signal3->setChecked(false);
        Plot2Signal4->setChecked(false);
        Plot2Signal5->setChecked(false);
        Plot2Signal6->setChecked(false);
        Plot2Signal7->setChecked(false);
        Plot2Signal8->setChecked(false);
        Plot2Signal9->setChecked(false);
        Plot2Signal10->setChecked(false);
        Plot2Signal11->setChecked(false);
        Plot2Signal12->setChecked(false);
        Plot2Signal13->setChecked(false);
        Plot2Signal14->setChecked(false);
        Plot2Signal15->setChecked(false);
        Plot2Signal16->setChecked(false);
        Plot2Signal17->setChecked(false);
        Plot2Signal18->setChecked(false);
        Plot2Signal19->setChecked(false);
        Plot2Signal20->setChecked(false);
        Plot2Signal21->setChecked(false);
        Plot2Signal22->setChecked(false);
        Plot2Signal23->setChecked(false);
        Plot2Signal24->setChecked(false);
        Plot2Signal25->setChecked(false);
        Plot2Signal26->setChecked(false);
        Plot2Signal27->setChecked(false);
        Plot2Signal28->setChecked(false);
        Plot2Signal29->setChecked(false);
        Plot2Signal30->setChecked(false);
        Plot2Signal31->setChecked(false);
        Plot2Signal32->setChecked(false);

    }
}

void quadro::Plot2Signal1_clicked()
{
    if(!Plot2Signal1->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal2_clicked()
{
    if(!Plot2Signal2->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal3_clicked()
{
    if(!Plot2Signal3->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal4_clicked()
{
    if(!Plot2Signal4->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal5_clicked()
{
    if(!Plot2Signal5->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal6_clicked()
{
    if(!Plot2Signal6->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal7_clicked()
{
    if(!Plot2Signal7->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal8_clicked()
{
    if(!Plot2Signal8->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal9_clicked()
{
    if(!Plot2Signal9->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal10_clicked()
{
    if(!Plot2Signal10->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal11_clicked()
{
    if(!Plot2Signal11->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal12_clicked()
{
    if(!Plot2Signal12->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal13_clicked()
{
    if(!Plot2Signal13->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal14_clicked()
{
    if(!Plot2Signal14->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal15_clicked()
{
    if(!Plot2Signal15->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal16_clicked()
{
    if(!Plot2Signal16->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal17_clicked()
{
    if(!Plot2Signal17->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal18_clicked()
{
    if(!Plot2Signal18->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal19_clicked()
{
    if(!Plot2Signal19->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal20_clicked()
{
    if(!Plot2Signal20->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal21_clicked()
{
    if(!Plot2Signal21->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal22_clicked()
{
    if(!Plot2Signal22->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal23_clicked()
{
    if(!Plot2Signal23->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal24_clicked()
{
    if(!Plot2Signal24->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal25_clicked()
{
    if(!Plot2Signal25->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal26_clicked()
{
    if(!Plot2Signal26->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal27_clicked()
{
    if(!Plot2Signal27->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal28_clicked()
{
    if(!Plot2Signal28->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal29_clicked()
{
    if(!Plot2Signal29->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal30_clicked()
{
    if(!Plot2Signal30->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal31_clicked()
{
    if(!Plot2Signal31->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}
void quadro::Plot2Signal32_clicked()
{
    if(!Plot2Signal32->isChecked()){
        Plot2SignalAll->setChecked(false);
    }
}

void quadro::Plot3SignalAll_clicked()
{
    if(Plot3SignalAll->isChecked()){
        Plot3Signal1->setChecked(true);
        Plot3Signal2->setChecked(true);
        Plot3Signal3->setChecked(true);
        Plot3Signal4->setChecked(true);
        Plot3Signal5->setChecked(true);
        Plot3Signal6->setChecked(true);
        Plot3Signal7->setChecked(true);
        Plot3Signal8->setChecked(true);
        Plot3Signal9->setChecked(true);
        Plot3Signal10->setChecked(true);
        Plot3Signal11->setChecked(true);
        Plot3Signal12->setChecked(true);
        Plot3Signal13->setChecked(true);
        Plot3Signal14->setChecked(true);
        Plot3Signal15->setChecked(true);
        Plot3Signal16->setChecked(true);
        Plot3Signal17->setChecked(true);
        Plot3Signal18->setChecked(true);
        Plot3Signal19->setChecked(true);
        Plot3Signal20->setChecked(true);
        Plot3Signal21->setChecked(true);
        Plot3Signal22->setChecked(true);
        Plot3Signal23->setChecked(true);
        Plot3Signal24->setChecked(true);
        Plot3Signal25->setChecked(true);
        Plot3Signal26->setChecked(true);
        Plot3Signal27->setChecked(true);
        Plot3Signal28->setChecked(true);
        Plot3Signal29->setChecked(true);
        Plot3Signal30->setChecked(true);
        Plot3Signal31->setChecked(true);
        Plot3Signal32->setChecked(true);

    }else{
        Plot3Signal1->setChecked(false);
        Plot3Signal2->setChecked(false);
        Plot3Signal3->setChecked(false);
        Plot3Signal4->setChecked(false);
        Plot3Signal5->setChecked(false);
        Plot3Signal6->setChecked(false);
        Plot3Signal7->setChecked(false);
        Plot3Signal8->setChecked(false);
        Plot3Signal9->setChecked(false);
        Plot3Signal10->setChecked(false);
        Plot3Signal11->setChecked(false);
        Plot3Signal12->setChecked(false);
        Plot3Signal13->setChecked(false);
        Plot3Signal14->setChecked(false);
        Plot3Signal15->setChecked(false);
        Plot3Signal16->setChecked(false);
        Plot3Signal17->setChecked(false);
        Plot3Signal18->setChecked(false);
        Plot3Signal19->setChecked(false);
        Plot3Signal20->setChecked(false);
        Plot3Signal21->setChecked(false);
        Plot3Signal22->setChecked(false);
        Plot3Signal23->setChecked(false);
        Plot3Signal24->setChecked(false);
        Plot3Signal25->setChecked(false);
        Plot3Signal26->setChecked(false);
        Plot3Signal27->setChecked(false);
        Plot3Signal28->setChecked(false);
        Plot3Signal29->setChecked(false);
        Plot3Signal30->setChecked(false);
        Plot3Signal31->setChecked(false);
        Plot3Signal32->setChecked(false);

    }
}

void quadro::Plot3Signal1_clicked()
{
    if(!Plot3Signal1->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal2_clicked()
{
    if(!Plot3Signal2->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal3_clicked()
{
    if(!Plot3Signal3->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal4_clicked()
{
    if(!Plot3Signal4->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal5_clicked()
{
    if(!Plot3Signal5->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal6_clicked()
{
    if(!Plot3Signal6->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal7_clicked()
{
    if(!Plot3Signal7->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal8_clicked()
{
    if(!Plot3Signal8->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal9_clicked()
{
    if(!Plot3Signal9->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal10_clicked()
{
    if(!Plot3Signal10->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal11_clicked()
{
    if(!Plot3Signal11->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal12_clicked()
{
    if(!Plot3Signal12->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal13_clicked()
{
    if(!Plot3Signal13->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal14_clicked()
{
    if(!Plot3Signal14->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal15_clicked()
{
    if(!Plot3Signal15->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal16_clicked()
{
    if(!Plot3Signal16->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal17_clicked()
{
    if(!Plot3Signal17->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal18_clicked()
{
    if(!Plot3Signal18->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal19_clicked()
{
    if(!Plot3Signal19->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal20_clicked()
{
    if(!Plot3Signal20->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal21_clicked()
{
    if(!Plot3Signal21->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal22_clicked()
{
    if(!Plot3Signal22->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal23_clicked()
{
    if(!Plot3Signal23->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal24_clicked()
{
    if(!Plot3Signal24->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal25_clicked()
{
    if(!Plot3Signal25->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal26_clicked()
{
    if(!Plot3Signal26->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal27_clicked()
{
    if(!Plot3Signal27->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal28_clicked()
{
    if(!Plot3Signal28->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal29_clicked()
{
    if(!Plot3Signal29->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal30_clicked()
{
    if(!Plot3Signal30->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal31_clicked()
{
    if(!Plot3Signal31->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}
void quadro::Plot3Signal32_clicked()
{
    if(!Plot3Signal32->isChecked()){
        Plot3SignalAll->setChecked(false);
    }
}

void quadro::Plot4SignalAll_clicked()
{
    if(Plot4SignalAll->isChecked()){
        Plot4Signal1->setChecked(true);
        Plot4Signal2->setChecked(true);
        Plot4Signal3->setChecked(true);
        Plot4Signal4->setChecked(true);
        Plot4Signal5->setChecked(true);
        Plot4Signal6->setChecked(true);
        Plot4Signal7->setChecked(true);
        Plot4Signal8->setChecked(true);
        Plot4Signal9->setChecked(true);
        Plot4Signal10->setChecked(true);
        Plot4Signal11->setChecked(true);
        Plot4Signal12->setChecked(true);
        Plot4Signal13->setChecked(true);
        Plot4Signal14->setChecked(true);
        Plot4Signal15->setChecked(true);
        Plot4Signal16->setChecked(true);
        Plot4Signal17->setChecked(true);
        Plot4Signal18->setChecked(true);
        Plot4Signal19->setChecked(true);
        Plot4Signal20->setChecked(true);
        Plot4Signal21->setChecked(true);
        Plot4Signal22->setChecked(true);
        Plot4Signal23->setChecked(true);
        Plot4Signal24->setChecked(true);
        Plot4Signal25->setChecked(true);
        Plot4Signal26->setChecked(true);
        Plot4Signal27->setChecked(true);
        Plot4Signal28->setChecked(true);
        Plot4Signal29->setChecked(true);
        Plot4Signal30->setChecked(true);
        Plot4Signal31->setChecked(true);
        Plot4Signal32->setChecked(true);

    }else{
        Plot4Signal1->setChecked(false);
        Plot4Signal2->setChecked(false);
        Plot4Signal3->setChecked(false);
        Plot4Signal4->setChecked(false);
        Plot4Signal5->setChecked(false);
        Plot4Signal6->setChecked(false);
        Plot4Signal7->setChecked(false);
        Plot4Signal8->setChecked(false);
        Plot4Signal9->setChecked(false);
        Plot4Signal10->setChecked(false);
        Plot4Signal11->setChecked(false);
        Plot4Signal12->setChecked(false);
        Plot4Signal13->setChecked(false);
        Plot4Signal14->setChecked(false);
        Plot4Signal15->setChecked(false);
        Plot4Signal16->setChecked(false);
        Plot4Signal17->setChecked(false);
        Plot4Signal18->setChecked(false);
        Plot4Signal19->setChecked(false);
        Plot4Signal20->setChecked(false);
        Plot4Signal21->setChecked(false);
        Plot4Signal22->setChecked(false);
        Plot4Signal23->setChecked(false);
        Plot4Signal24->setChecked(false);
        Plot4Signal25->setChecked(false);
        Plot4Signal26->setChecked(false);
        Plot4Signal27->setChecked(false);
        Plot4Signal28->setChecked(false);
        Plot4Signal29->setChecked(false);
        Plot4Signal30->setChecked(false);
        Plot4Signal31->setChecked(false);
        Plot4Signal32->setChecked(false);

    }
}

void quadro::Plot4Signal1_clicked()
{
    if(!Plot4Signal1->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal2_clicked()
{
    if(!Plot4Signal2->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal3_clicked()
{
    if(!Plot4Signal3->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal4_clicked()
{
    if(!Plot4Signal4->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal5_clicked()
{
    if(!Plot4Signal5->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal6_clicked()
{
    if(!Plot4Signal6->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal7_clicked()
{
    if(!Plot4Signal7->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal8_clicked()
{
    if(!Plot4Signal8->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal9_clicked()
{
    if(!Plot4Signal9->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal10_clicked()
{
    if(!Plot4Signal10->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal11_clicked()
{
    if(!Plot4Signal11->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal12_clicked()
{
    if(!Plot4Signal12->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal13_clicked()
{
    if(!Plot4Signal13->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal14_clicked()
{
    if(!Plot4Signal14->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal15_clicked()
{
    if(!Plot4Signal15->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal16_clicked()
{
    if(!Plot4Signal16->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal17_clicked()
{
    if(!Plot4Signal17->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal18_clicked()
{
    if(!Plot4Signal18->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal19_clicked()
{
    if(!Plot4Signal19->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal20_clicked()
{
    if(!Plot4Signal20->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal21_clicked()
{
    if(!Plot4Signal21->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal22_clicked()
{
    if(!Plot4Signal22->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal23_clicked()
{
    if(!Plot4Signal23->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal24_clicked()
{
    if(!Plot4Signal24->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal25_clicked()
{
    if(!Plot4Signal25->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal26_clicked()
{
    if(!Plot4Signal26->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal27_clicked()
{
    if(!Plot4Signal27->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal28_clicked()
{
    if(!Plot4Signal28->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal29_clicked()
{
    if(!Plot4Signal29->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal30_clicked()
{
    if(!Plot4Signal30->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal31_clicked()
{
    if(!Plot4Signal31->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}
void quadro::Plot4Signal32_clicked()
{
    if(!Plot4Signal32->isChecked()){
        Plot4SignalAll->setChecked(false);
    }
}

  void quadro::detectStatus()
{
      connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataStatus()));
      dataTimer.start(5000); // Interval 0 means to refresh as fast as possible
}

  void quadro::realtimeDataStatus()
  {
        ui->controllersStatus->setText("On");
        ui->landingStatus->setText(QString::number(numberOfQuadro));

  }// nasledne doplnit o funkce na volani statusu todo

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
    ui->scrollLegend->setLayout(legendLay);

  }

void quadro::on_actionDebug_triggered()
{

}
