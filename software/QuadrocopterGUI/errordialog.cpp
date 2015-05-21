#include "errordialog.h"
#include "ui_errordialog.h"
#include "failuredetection.h"

errorDialog::errorDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::errorDialog)
{
    ui->setupUi(this);
    //writeErrors();
}

errorDialog::~errorDialog()
{
    delete ui;
}

void errorDialog::writeErrors()
{
    errorDetected=false;
    for(int i;i<10;i++)
    {
        //if(m_failure->getErrors(i)!=0)
       // {
       //     errorDetected=true;
            //call switch
       // }
    }
    if(errorDetected==false) ui->textBrowser->append("No errors");
    ui->textBrowser->append("tady budou error texty");
    ui->textBrowser->append("jednou urcite");
    ui->textBrowser->append("prisaham");
}

void errorDialog::setFailure(failuredetection *fail)
{
    m_failure=fail;
}

void errorDialog::on_pushButton_clicked()
{
    this->close();
}


