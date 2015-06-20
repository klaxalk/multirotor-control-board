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
    for(int i;i<3;i++)
    {
        if(m_failure->getErrors(i)!=0)
        {
            errorDetected=true;
            switch ( i ) {

            case 0 :
                ui->textBrowser->append("Evelator error");
                break;

            case 1 :
                ui->textBrowser->append("Aileron error");
                break;
            case 2 :
                ui->textBrowser->append("Altitude error");
                break;
            default :
                ui->textBrowser->append("necum");

            }
        }
    }
    if(errorDetected==false) ui->textBrowser->append("No errors");
}

void errorDialog::setFailure(failuredetection *fail)
{
    m_failure=fail;
}

void errorDialog::on_pushButton_clicked()
{
    this->close();
}


