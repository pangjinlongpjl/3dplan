#pragma once
#ifndef READTIFF_H
#define READTIFF_H

#include "QThread"
#include <gdal/gdal.h>
#include "QDebug"
#include "Eigen/Eigen"
#include "utils.hpp"

using namespace std;

class ReadTiff:public QThread
{
    Q_OBJECT
public:
    ReadTiff();
    void fromTiff();
    void setPath(QString path);
    template <typename T>
    QVector<QVector<double>> construct2DArray( QVector< T > original, int m, int n );
    template <typename T>
    vector<vector<double>> construct2DArray( vector< T > original, int m, int n );

    Eigen::MatrixXf construct2DArray( Eigen::VectorXf original, int m, int n );
    QPair<Eigen::MatrixXf,Eigen::MatrixXf>  meshgrid(
        Eigen::VectorXf &vecX, Eigen::VectorXf &vecY);


public:
    Eigen::MatrixXf datax;
    Eigen::MatrixXf datay;
    Eigen::MatrixXf dataz;

    int demXsize;
    int demYsize;

    Environment env;

};

#endif // READTIFF_H
