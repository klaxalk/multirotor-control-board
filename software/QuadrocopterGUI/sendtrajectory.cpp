#include "sendtrajectory.h"
#include "ui_sendtrajectory.h"
#include "send.h"
#include "commands.h"
#include "time.h"

unsigned char sizeTr;// 1=1 bod, 2=2 body
int* timeTraj;
float* elevatorPos;
float* aileronPos;
float* throttlePos;
double now;

sendTrajectory::sendTrajectory(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::sendTrajectory)
{
    ui->setupUi(this);
    setTextOfLine();
}

sendTrajectory::~sendTrajectory()
{
    delete ui;
}

void sendTrajectory::setTextOfLine(){
    ui->time0->setText("0");
    ui->time0->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->time1->setText("0");
    ui->time1->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->time2->setText("0");
    ui->time2->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->time3->setText("0");
    ui->time3->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->time4->setText("0");
    ui->time4->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->time5->setText("0");
    ui->time5->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->time6->setText("0");
    ui->time6->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->time7->setText("0");
    ui->time7->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->time8->setText("0");
    ui->time8->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->time9->setText("0");
    ui->time9->setValidator( new QDoubleValidator(-100, 100, 2, this));

    ui->elevator0->setText("0");
    ui->elevator0->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->elevator1->setText("0");
    ui->elevator1->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->elevator2->setText("0");
    ui->elevator2->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->elevator3->setText("0");
    ui->elevator3->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->elevator4->setText("0");
    ui->elevator4->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->elevator5->setText("0");
    ui->elevator4->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->elevator6->setText("0");
    ui->elevator6->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->elevator7->setText("0");
    ui->elevator7->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->elevator8->setText("0");
    ui->elevator8->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->elevator9->setText("0");
    ui->elevator9->setValidator( new QDoubleValidator(-100, 100, 2, this));

    ui->aileron0->setText("0");
    ui->aileron0->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->aileron1->setText("0");
    ui->aileron1->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->aileron2->setText("0");
    ui->aileron2->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->aileron3->setText("0");
    ui->aileron3->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->aileron4->setText("0");
    ui->aileron4->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->aileron5->setText("0");
    ui->aileron5->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->aileron6->setText("0");
    ui->aileron6->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->aileron7->setText("0");
    ui->aileron7->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->aileron8->setText("0");
    ui->aileron8->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->aileron9->setText("0");
    ui->aileron9->setValidator( new QDoubleValidator(-100, 100, 2, this));

    ui->throttle0->setText("0");
    ui->throttle0->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->throttle1->setText("0");
    ui->throttle1->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->throttle2->setText("0");
    ui->throttle2->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->throttle3->setText("0");
    ui->throttle3->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->throttle4->setText("0");
    ui->throttle4->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->throttle5->setText("0");
    ui->throttle5->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->throttle6->setText("0");
    ui->throttle6->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->throttle7->setText("0");
    ui->throttle7->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->throttle8->setText("0");
    ui->throttle8->setValidator( new QDoubleValidator(-100, 100, 2, this));
    ui->throttle9->setText("0");
    ui->throttle9->setValidator( new QDoubleValidator(-100, 100, 2, this));
}
void sendTrajectory::setKopter(unsigned char KOPTERS){
    kopterTr=KOPTERS;
}

void sendTrajectory::on_pushButton_clicked()
{
     now = time (NULL);
    int count=0;
    if(ui->time0->text().toFloat()!=0)
    {
        *(timeTraj+count)=(int)(now+ui->time0->text().toFloat());
        *(elevatorPos+count)=ui->elevator0->text().toFloat();
        *(aileronPos+count)=ui->aileron0->text().toFloat();
        *(throttlePos+count)=ui->throttle0->text().toFloat();
        count++;
    }
    if(ui->time1->text().toFloat()!=0)
    {
        *(timeTraj+count)=(int)(now+ui->time1->text().toFloat());
        *(elevatorPos+count)=ui->elevator1->text().toFloat();
        *(aileronPos+count)=ui->aileron1->text().toFloat();
        *(throttlePos+count)=ui->throttle1->text().toFloat();
        count++;
    }
    if(ui->time2->text().toFloat()!=0)
    {
        *(timeTraj+count)=(int)(now+ui->time2->text().toFloat());
        *(elevatorPos+count)=ui->elevator2->text().toFloat();
        *(aileronPos+count)=ui->aileron2->text().toFloat();
        *(throttlePos+count)=ui->throttle2->text().toFloat();
        count++;
    }
    if(ui->time3->text().toFloat()!=0)
    {
        *(timeTraj+count)=(int)(now+ui->time3->text().toFloat());
        *(elevatorPos+count)=ui->elevator3->text().toFloat();
        *(aileronPos+count)=ui->aileron3->text().toFloat();
        *(throttlePos+count)=ui->throttle3->text().toFloat();
        count++;
    }
    if(ui->time4->text().toFloat()!=0)
    {
        *(timeTraj+count)=(int)(now+ui->time4->text().toFloat());
        *(elevatorPos+count)=ui->elevator4->text().toFloat();
        *(aileronPos+count)=ui->aileron4->text().toFloat();
        *(throttlePos+count)=ui->throttle4->text().toFloat();
        count++;
    }
    if(ui->time5->text().toFloat()!=0)
    {
        *(timeTraj+count)=(int)(now+ui->time5->text().toFloat());
        *(elevatorPos+count)=ui->elevator5->text().toFloat();
        *(aileronPos+count)=ui->aileron5->text().toFloat();
        *(throttlePos+count)=ui->throttle5->text().toFloat();
        count++;
    }
    if(ui->time6->text().toFloat()!=0)
    {
        *(timeTraj+count)=(int)(now+ui->time6->text().toFloat());
        *(elevatorPos+count)=ui->elevator6->text().toFloat();
        *(aileronPos+count)=ui->aileron6->text().toFloat();
        *(throttlePos+count)=ui->throttle6->text().toFloat();
        count++;
    }
    if(ui->time7->text().toFloat()!=0)
    {
        *(timeTraj+count)=(int)(now+ui->time7->text().toFloat());
        *(elevatorPos+count)=ui->elevator7->text().toFloat();
        *(aileronPos+count)=ui->aileron7->text().toFloat();
        *(throttlePos+count)=ui->throttle7->text().toFloat();
        count++;
    }
    if(ui->time8->text().toFloat()!=0)
    {
        *(timeTraj+count)=(int)(now+ui->time8->text().toFloat());
        *(elevatorPos+count)=ui->elevator8->text().toFloat();
        *(aileronPos+count)=ui->aileron8->text().toFloat();
        *(throttlePos+count)=ui->throttle8->text().toFloat();
        count++;
    }
    if(ui->time9->text().toFloat()!=0)
    {
        *(timeTraj+count)=(int)(now+ui->time9->text().toFloat());
        *(elevatorPos+count)=ui->elevator9->text().toFloat();
        *(aileronPos+count)=ui->aileron9->text().toFloat();
        *(throttlePos+count)=ui->throttle9->text().toFloat();
        count++;
    }

    if(count>0)
    {
        if(kopterTr==KOPTERS.K1){
            kopterTimeRequest(ADDRESS.K1,ADDRESS.UNKNOWN16,(int) now,0x00);
        }else if(kopterTr==KOPTERS.K2){
            kopterTimeRequest(ADDRESS.K2,ADDRESS.UNKNOWN16,(int) now,0x00);
        }else if(kopterTr==KOPTERS.K3){
            kopterTimeRequest(ADDRESS.K3,ADDRESS.UNKNOWN16,(int) now,0x00);
        }else if(kopterTr==KOPTERS.KC1){
            kopterTimeRequest(ADDRESS.KC1,ADDRESS.UNKNOWN16,(int) now,0x00);
        }

        sizeTr=(unsigned char)count;
        trajectoryAddPoint(kopterTr,sizeTr,timeTraj,elevatorPos,aileronPos,throttlePos);
    }

    this->close();
}
