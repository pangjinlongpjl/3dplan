#pragma once
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "readtiff.h"
#include "readprimitives.h"

#include <QMainWindow>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    QTimer *timer;
//public slots:
//    void mytimerSlot();//
//    void startTimer();
//    void endTimer();

private:
    Ui::MainWindow *ui;
    int cnt=0;
    ReadTiff *rt;
    readPrimitives *rp;

};
#endif // MAINWINDOW_H
