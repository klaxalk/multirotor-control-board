#ifndef SENDTRAJECTORY_H
#define SENDTRAJECTORY_H

#include <QDialog>

namespace Ui {
class sendTrajectory;
}

class sendTrajectory : public QDialog
{
    Q_OBJECT

public:
    explicit sendTrajectory(QWidget *parent = 0);
    ~sendTrajectory();
    void setTextOfLine();
    void setKopter(unsigned char KOPTER);

private slots:
    void on_pushButton_clicked();

private:
    Ui::sendTrajectory *ui;
    unsigned char kopterTr;
};

#endif // SENDTRAJECTORY_H
