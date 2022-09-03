#pragma once
#ifndef READPRIMITIVES_H
#define READPRIMITIVES_H

#include <QThread>
#include <Eigen/Eigen>
#include <json.hpp>
#include "utils.hpp"

class readPrimitives: public QThread
{
    Q_OBJECT
public:
    readPrimitives(std::string fp="");
    void generateMPs(); //

public:
    Eigen::MatrixXd pmset;
    MovementPrimitives ms;  // 运动基元
    std::string filepath;   // 运动基元的路径


};

#endif // READPRIMITIVES_H
