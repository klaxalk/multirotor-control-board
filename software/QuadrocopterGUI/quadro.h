#ifndef QUADRO_H
#define QUADRO_H

#include <QMainWindow>
#include <QCheckBox>
#include "qcustomplot.h"
#include "connectquadro.h"

namespace Ui {
class quadro;
}

class quadro : public QMainWindow
{
    Q_OBJECT

public:
    explicit quadro(QWidget *parent = 0);
    ~quadro();
    void startGraph();
    void createScrollCheckBox();
    void setAllCheckFalse();
    void detectStatus();
    void createLegend();
    void setQuadro(int number);
    void graph_1(QCustomPlot *graph1);
    void graph_2(QCustomPlot *graph2);
    void graph_3(QCustomPlot *graph3);
    void graph_4(QCustomPlot *graph4);


    private slots:
      void realtimeDataSlotGraph1();
      void realtimeDataSlotGraph2();
      void realtimeDataSlotGraph3();
      void realtimeDataSlotGraph4();
      void realtimeDataStatus();

      void on_actionConnect_quadrocopter_triggered();
      void on_comboBox_activated(int index);
      void Plot1SignalAll_clicked();
      void Plot2SignalAll_clicked();
      void Plot3SignalAll_clicked();
      void Plot4SignalAll_clicked();

      void Plot1Signal1_clicked();
      void Plot1Signal2_clicked();
      void Plot1Signal3_clicked();
      void Plot1Signal4_clicked();
      void Plot1Signal5_clicked();
      void Plot1Signal6_clicked();
      void Plot1Signal7_clicked();
      void Plot1Signal8_clicked();
      void Plot1Signal9_clicked();
      void Plot1Signal10_clicked();
      void Plot1Signal11_clicked();
      void Plot1Signal12_clicked();
      void Plot1Signal13_clicked();
      void Plot1Signal14_clicked();
      void Plot1Signal15_clicked();
      void Plot1Signal16_clicked();
      void Plot1Signal17_clicked();
      void Plot1Signal18_clicked();
      void Plot1Signal19_clicked();
      void Plot1Signal20_clicked();
      void Plot1Signal21_clicked();
      void Plot1Signal22_clicked();
      void Plot1Signal23_clicked();
      void Plot1Signal24_clicked();
      void Plot1Signal25_clicked();
      void Plot1Signal26_clicked();
      void Plot1Signal27_clicked();
      void Plot1Signal28_clicked();
      void Plot1Signal29_clicked();
      void Plot1Signal30_clicked();
      void Plot1Signal31_clicked();
      void Plot1Signal32_clicked();

      void Plot2Signal1_clicked();
      void Plot2Signal2_clicked();
      void Plot2Signal3_clicked();
      void Plot2Signal4_clicked();
      void Plot2Signal5_clicked();
      void Plot2Signal6_clicked();
      void Plot2Signal7_clicked();
      void Plot2Signal8_clicked();
      void Plot2Signal9_clicked();
      void Plot2Signal10_clicked();
      void Plot2Signal11_clicked();
      void Plot2Signal12_clicked();
      void Plot2Signal13_clicked();
      void Plot2Signal14_clicked();
      void Plot2Signal15_clicked();
      void Plot2Signal16_clicked();
      void Plot2Signal17_clicked();
      void Plot2Signal18_clicked();
      void Plot2Signal19_clicked();
      void Plot2Signal20_clicked();
      void Plot2Signal21_clicked();
      void Plot2Signal22_clicked();
      void Plot2Signal23_clicked();
      void Plot2Signal24_clicked();
      void Plot2Signal25_clicked();
      void Plot2Signal26_clicked();
      void Plot2Signal27_clicked();
      void Plot2Signal28_clicked();
      void Plot2Signal29_clicked();
      void Plot2Signal30_clicked();
      void Plot2Signal31_clicked();
      void Plot2Signal32_clicked();

      void Plot3Signal1_clicked();
      void Plot3Signal2_clicked();
      void Plot3Signal3_clicked();
      void Plot3Signal4_clicked();
      void Plot3Signal5_clicked();
      void Plot3Signal6_clicked();
      void Plot3Signal7_clicked();
      void Plot3Signal8_clicked();
      void Plot3Signal9_clicked();
      void Plot3Signal10_clicked();
      void Plot3Signal11_clicked();
      void Plot3Signal12_clicked();
      void Plot3Signal13_clicked();
      void Plot3Signal14_clicked();
      void Plot3Signal15_clicked();
      void Plot3Signal16_clicked();
      void Plot3Signal17_clicked();
      void Plot3Signal18_clicked();
      void Plot3Signal19_clicked();
      void Plot3Signal20_clicked();
      void Plot3Signal21_clicked();
      void Plot3Signal22_clicked();
      void Plot3Signal23_clicked();
      void Plot3Signal24_clicked();
      void Plot3Signal25_clicked();
      void Plot3Signal26_clicked();
      void Plot3Signal27_clicked();
      void Plot3Signal28_clicked();
      void Plot3Signal29_clicked();
      void Plot3Signal30_clicked();
      void Plot3Signal31_clicked();
      void Plot3Signal32_clicked();

      void Plot4Signal1_clicked();
      void Plot4Signal2_clicked();
      void Plot4Signal3_clicked();
      void Plot4Signal4_clicked();
      void Plot4Signal5_clicked();
      void Plot4Signal6_clicked();
      void Plot4Signal7_clicked();
      void Plot4Signal8_clicked();
      void Plot4Signal9_clicked();
      void Plot4Signal10_clicked();
      void Plot4Signal11_clicked();
      void Plot4Signal12_clicked();
      void Plot4Signal13_clicked();
      void Plot4Signal14_clicked();
      void Plot4Signal15_clicked();
      void Plot4Signal16_clicked();
      void Plot4Signal17_clicked();
      void Plot4Signal18_clicked();
      void Plot4Signal19_clicked();
      void Plot4Signal20_clicked();
      void Plot4Signal21_clicked();
      void Plot4Signal22_clicked();
      void Plot4Signal23_clicked();
      void Plot4Signal24_clicked();
      void Plot4Signal25_clicked();
      void Plot4Signal26_clicked();
      void Plot4Signal27_clicked();
      void Plot4Signal28_clicked();
      void Plot4Signal29_clicked();
      void Plot4Signal30_clicked();
      void Plot4Signal31_clicked();
      void Plot4Signal32_clicked();

      void on_actionDebug_triggered();

private:
    Ui::quadro *ui;
    connectquadro *connectQuad;
    QTimer dataTimer;
    QTimer dataTimer1;
    QTimer dataTimer2;
    QTimer dataTimer3;
    QTimer dataTimer4;

    QCheckBox *Plot1Signal1;
    QCheckBox *Plot1Signal2;
    QCheckBox *Plot1Signal3;
    QCheckBox *Plot1Signal4;
    QCheckBox *Plot1Signal5;
    QCheckBox *Plot1Signal6;
    QCheckBox *Plot1Signal7;
    QCheckBox *Plot1Signal8;
    QCheckBox *Plot1Signal9;
    QCheckBox *Plot1Signal10;
    QCheckBox *Plot1Signal11;
    QCheckBox *Plot1Signal12;
    QCheckBox *Plot1Signal13;
    QCheckBox *Plot1Signal14;
    QCheckBox *Plot1Signal15;
    QCheckBox *Plot1Signal16;
    QCheckBox *Plot1Signal17;
    QCheckBox *Plot1Signal18;
    QCheckBox *Plot1Signal19;
    QCheckBox *Plot1Signal20;
    QCheckBox *Plot1Signal21;
    QCheckBox *Plot1Signal22;
    QCheckBox *Plot1Signal23;
    QCheckBox *Plot1Signal24;
    QCheckBox *Plot1Signal25;
    QCheckBox *Plot1Signal26;
    QCheckBox *Plot1Signal27;
    QCheckBox *Plot1Signal28;
    QCheckBox *Plot1Signal29;
    QCheckBox *Plot1Signal30;
    QCheckBox *Plot1Signal31;
    QCheckBox *Plot1Signal32;
    QCheckBox *Plot1SignalAll;

    QCheckBox *Plot2Signal1;
    QCheckBox *Plot2Signal2;
    QCheckBox *Plot2Signal3;
    QCheckBox *Plot2Signal4;
    QCheckBox *Plot2Signal5;
    QCheckBox *Plot2Signal6;
    QCheckBox *Plot2Signal7;
    QCheckBox *Plot2Signal8;
    QCheckBox *Plot2Signal9;
    QCheckBox *Plot2Signal10;
    QCheckBox *Plot2Signal11;
    QCheckBox *Plot2Signal12;
    QCheckBox *Plot2Signal13;
    QCheckBox *Plot2Signal14;
    QCheckBox *Plot2Signal15;
    QCheckBox *Plot2Signal16;
    QCheckBox *Plot2Signal17;
    QCheckBox *Plot2Signal18;
    QCheckBox *Plot2Signal19;
    QCheckBox *Plot2Signal20;
    QCheckBox *Plot2Signal21;
    QCheckBox *Plot2Signal22;
    QCheckBox *Plot2Signal23;
    QCheckBox *Plot2Signal24;
    QCheckBox *Plot2Signal25;
    QCheckBox *Plot2Signal26;
    QCheckBox *Plot2Signal27;
    QCheckBox *Plot2Signal28;
    QCheckBox *Plot2Signal29;
    QCheckBox *Plot2Signal30;
    QCheckBox *Plot2Signal31;
    QCheckBox *Plot2Signal32;
    QCheckBox *Plot2SignalAll;

    QCheckBox *Plot3Signal1;
    QCheckBox *Plot3Signal2;
    QCheckBox *Plot3Signal3;
    QCheckBox *Plot3Signal4;
    QCheckBox *Plot3Signal5;
    QCheckBox *Plot3Signal6;
    QCheckBox *Plot3Signal7;
    QCheckBox *Plot3Signal8;
    QCheckBox *Plot3Signal9;
    QCheckBox *Plot3Signal10;
    QCheckBox *Plot3Signal11;
    QCheckBox *Plot3Signal12;
    QCheckBox *Plot3Signal13;
    QCheckBox *Plot3Signal14;
    QCheckBox *Plot3Signal15;
    QCheckBox *Plot3Signal16;
    QCheckBox *Plot3Signal17;
    QCheckBox *Plot3Signal18;
    QCheckBox *Plot3Signal19;
    QCheckBox *Plot3Signal20;
    QCheckBox *Plot3Signal21;
    QCheckBox *Plot3Signal22;
    QCheckBox *Plot3Signal23;
    QCheckBox *Plot3Signal24;
    QCheckBox *Plot3Signal25;
    QCheckBox *Plot3Signal26;
    QCheckBox *Plot3Signal27;
    QCheckBox *Plot3Signal28;
    QCheckBox *Plot3Signal29;
    QCheckBox *Plot3Signal30;
    QCheckBox *Plot3Signal31;
    QCheckBox *Plot3Signal32;
    QCheckBox *Plot3SignalAll;

    QCheckBox *Plot4Signal1;
    QCheckBox *Plot4Signal2;
    QCheckBox *Plot4Signal3;
    QCheckBox *Plot4Signal4;
    QCheckBox *Plot4Signal5;
    QCheckBox *Plot4Signal6;
    QCheckBox *Plot4Signal7;
    QCheckBox *Plot4Signal8;
    QCheckBox *Plot4Signal9;
    QCheckBox *Plot4Signal10;
    QCheckBox *Plot4Signal11;
    QCheckBox *Plot4Signal12;
    QCheckBox *Plot4Signal13;
    QCheckBox *Plot4Signal14;
    QCheckBox *Plot4Signal15;
    QCheckBox *Plot4Signal16;
    QCheckBox *Plot4Signal17;
    QCheckBox *Plot4Signal18;
    QCheckBox *Plot4Signal19;
    QCheckBox *Plot4Signal20;
    QCheckBox *Plot4Signal21;
    QCheckBox *Plot4Signal22;
    QCheckBox *Plot4Signal23;
    QCheckBox *Plot4Signal24;
    QCheckBox *Plot4Signal25;
    QCheckBox *Plot4Signal26;
    QCheckBox *Plot4Signal27;
    QCheckBox *Plot4Signal28;
    QCheckBox *Plot4Signal29;
    QCheckBox *Plot4Signal30;
    QCheckBox *Plot4Signal31;
    QCheckBox *Plot4Signal32;
    QCheckBox *Plot4SignalAll;
};

#endif // QUADRO_H
