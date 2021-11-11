//Written by Bastien LAFARGUE
//Last update 01/11/21 15:37 GMT+1

#include <iostream>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "resolving.h"
#include "equation.h"
#include "integ_meth.h"
#include "autoscale_graph.h"
#include "fstream"

using namespace std;
using namespace boost::numeric::odeint;

double kmin1;
double kmax1;                                                                     //Values used for autoscale of the graph
double kmin2;
double kmax2;

double MASS;                                                                      //Mass (kg)
double IZZ;                                                                       //Yaw Moment of Inertia (kg.m²)
double LF;                                                                        //Lenght front axle->Cg (m)
double LR;                                                                        //Lenght rear axle->Cg  (m)
double CF;                                                                        //Front cornering stiffness (N/rad)
double CR;                                                                        //Rear cornering stiffness (N/rad)
double AMPL;                                                                      //Input amplitude (at 10s for ramp) (rad)
double FREQ;                                                                      //Input frequency (for sine wave only) (Hz)
double VX;                                                                        //Longitudinal velocity (m/s)
QString ST_T;                                                                     //Input type : step, ramp, sine (/)

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->progressBar->setValue(0);                                                 //Progress bar initialisation
    ui->comboBox->addItem("step");                                                //Add "step steer" to combobox
    ui->comboBox->addItem("sine");                                                //Add "sine steer" to combobox
    ui->comboBox->addItem("ramp");                                                //Add "ramp steer" to combobox
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
   ui->customPlot->clearPlottables();                                             //Clear plot

   vector <double> lat_vel;                                                       //Lateral velocity initialisation for output
   vector <double> yaw_rate;                                                      //Yaw rate initialisation for output
   vector <double> time;                                                          //Time initialisation for output
   size_t step;
   integ Method;
   state_type x(2);

   Method.RK4(x);                                                                 //Integration method : .Euler (Euler), .RK4 (Runge Khutta 4)

   tie(lat_vel, yaw_rate, time, step) = resolution();                             //Get data from Resolving->Vector Resolution function
   vector<vector<double>> matrix;
   vector<int> row;

   for (size_t i = 0; i <= step; i++)                                             //Output loop
   {
   cout << time[i] << '\t' << yaw_rate[i] << '\t' <<lat_vel[i] << '\n' ;          //Print time, yaw rate and lat vel in console at every step
   ui->progressBar->setValue(i);
   }

   //-------------Graph construction--------------


   QVector<double> qVec = QVector<double>(yaw_rate.begin(), yaw_rate.end());      //Vector conversion to QVector (yaw rate)
   QVector<double> gVec = QVector<double>(lat_vel.begin(), lat_vel.end());        //Vector conversion to QVector (lateral velocity)
   QVector<double> times = QVector<double>(time.begin(), time.end());             //Vector conversion to QVector (time)


   kmin1 = *std::min_element(qVec.constBegin(), qVec.constEnd());                 //Min value yaw rate
   kmax1 = *std::max_element(qVec.constBegin(), qVec.constEnd());                 //Max value yaw rate
   kmin2 = *std::min_element(gVec.constBegin(), gVec.constEnd());                 //Min value lateral velocity
   kmax2 = *std::max_element(gVec.constBegin(), gVec.constEnd());                 //Max value lateral velocity

   tie(kmin1,kmax1,kmin2,kmax2)=graph(kmin1,kmax1,kmin2,kmax2);                   //Get "k" values from autoscale program to set range

   ui->customPlot->xAxis->setRange(0, 10);                                        //Range time axis of yaw rate
   ui->customPlot->yAxis->setRange(kmin1,kmax1);                                  //Range y axis yaw rate
   ui->customPlot->xAxis2->setRange(0, 10);                                       //Range time axis of lateral velocity
   ui->customPlot->yAxis2->setRange(kmin2,kmax2);                                 //Range y axis lateral velocity

   ui->customPlot->addGraph(ui->customPlot->xAxis, ui->customPlot->yAxis);        //Graph 0 creation
   ui->customPlot->graph(0)->setPen(QPen(QColor(40, 110, 255)));                  //Grap 0 curve colour (blue)

   ui->customPlot->addGraph(ui->customPlot->xAxis2, ui->customPlot->yAxis2);      //Graph 1 creation
   ui->customPlot->graph(1)->setPen(QPen(QColor(255, 110, 40)));                  //Graph 1 curve colour (red)


   ui->customPlot->graph(0)->setData(times, qVec);                                //Values used in graph 0
   ui->customPlot->graph(1)->setData(times, gVec);                                //Values used in graph 1

   ui->customPlot->xAxis2->setVisible(false);                                     //Hide or show graph 1 time axis
   ui->customPlot->yAxis2->setVisible(true);                                      //Hide or show graph 1 y axis

   ui->customPlot->xAxis->setLabel("Time (s)");                                   //Graph 0 time axis label
   ui->customPlot->yAxis->setLabel("Yaw Rate (rad/s)");                           //Graph 0 y axis label
   ui->customPlot->yAxis2->setLabel("Lateral velocity (m/s)");                    //Graph 1 y axis label
   ui->customPlot->graph(0)->setName("Yaw Rate (rad/s)");                         //Graph 0 curve label
   ui->customPlot->graph(1)->setName("Lateral velocity (m/s)");                   //Graph 1 curve label
   ui->customPlot->legend->setVisible(true);                                      //Hide or show graphs legend


   QFont legendFont = font();                                                     //Legend font declaration
   legendFont.setPointSize(9);                                                    //Set legend font size
   ui->customPlot->legend->setFont(legendFont);                                   //Set legend font style
   ui->customPlot->legend->setBrush(QBrush(QColor(255,255,255,230)));             //Set legend font colour
   ui->customPlot->axisRect()->insetLayout()->setInsetAlignment\
           (0, Qt::AlignTop|Qt::AlignLeft);                                       //Set legend position on graph

   ui->customPlot->replot();                                                      //Update graph

}

void MainWindow::on_pushButton_2_clicked()
{
   MASS=ui->doubleSpinBox->value();
   IZZ=ui->doubleSpinBox_2->value();
   LF=ui->doubleSpinBox_3->value();
   LR=ui->doubleSpinBox_4->value();
   CF=ui->doubleSpinBox_5->value();
   CR=ui->doubleSpinBox_6->value();

   ui->label_7->setText(QString::number(MASS));
   ui->label_8->setText(QString::number(IZZ));
   ui->label_9->setText(QString::number(LF));
   ui->label_10->setText(QString::number(LR));
   ui->label_11->setText(QString::number(CF));
   ui->label_12->setText(QString::number(CR));
}

void MainWindow::on_pushButton_3_clicked()
{
   AMPL=ui->doubleSpinBox_7->value();
   FREQ=ui->doubleSpinBox_8->value();
   VX=ui->doubleSpinBox_9->value();
   ST_T=ui->comboBox->currentText();

   ui->label_39->setText(ST_T);
   ui->label_44->setText(QString::number(AMPL));
   ui->label_47->setText(QString::number(FREQ));
}



void MainWindow::on_pushButton_4_clicked(vector <double> lat_vel, vector <double> yaw_rate,vector <double>time,size_t step)
{
    QString file_name=ui->lineEdit->text();
    ofstream myFile;
    myFile.open("open.csv");
    for (size_t i = 0; i <= step; i++)                                             //Output loop
    {
    myFile << time[i] << "," << yaw_rate[i] <<  "," <<lat_vel[i] << endl ;          //Print time, yaw rate and lat vel in console at every step
    }
}

