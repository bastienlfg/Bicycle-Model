//Written by Bastien LAFARGUE
//Last update 01/11/21 15:37 GMT+1

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }

extern double LF;
extern double LR;
extern double MASS;
extern double IZZ;
extern double CF;
extern double CR;
extern double AMPL;
extern double FREQ;
extern double VX;
extern QString ST_T;


QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked(std::vector <double> lat_vel, std::vector <double> yaw_rate,std::vector <double>time,size_t step);

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
