#ifndef ERRORDIALOG_H
#define ERRORDIALOG_H

#include <QDialog>
#include "failuredetection.h"

namespace Ui {
class errorDialog;
}

class errorDialog : public QDialog
{
    Q_OBJECT

public:
    explicit errorDialog(QWidget *parent = 0);
    ~errorDialog();
    failuredetection* m_failure;
    void setFailure(failuredetection *fail);
    void writeErrors();

private slots:
    void on_pushButton_clicked();

private:
    Ui::errorDialog *ui;
    bool errorDetected;

};

#endif // ERRORDIALOG_H
