//
// Created by lrm on 22-7-29.
//

#ifndef INC_3DPLAN_COMMON_H
#define INC_3DPLAN_COMMON_H

//#include "sbpl/utils/utils.h"
#include "vector"
#include "set"

class sbpl_3Dcell_t
{
public:
    sbpl_3Dcell_t()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    sbpl_3Dcell_t(int x_, int y_, int z_)
    {
        x = x_;
        y = y_;
        z = z_;
    }

    bool operator==(const sbpl_3Dcell_t cell) const
    {
        return x == cell.x && y == cell.y && z == cell.z;
    }

    bool operator<(const sbpl_3Dcell_t cell) const
    {
        return x < cell.x || (x == cell.x && (y < cell.y || (y==cell.y && ( z < cell.z))));
    }

    int x;
    int y;
    int z;
};

class sbpl_3Dpt_t
{
public:
    sbpl_3Dpt_t()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    sbpl_3Dpt_t(double x_, double y_, double z_)
    {
        x = x_;
        y = y_;
        z = z_;
    }

    bool operator==(const sbpl_3Dpt_t p) const
    {
        return x == p.x && y == p.y && z == p.z;
    }

    bool operator<(const sbpl_3Dpt_t p) const
    {
        return x < p.x || (x == p.x && (y < p.y|| y==p.y && (z < p. z)));
    }

    double x;
    double y;
    double z;
};

class sbpl_xyz_theta_cell_t // 起点与终点的表示是int
{
public:
    sbpl_xyz_theta_cell_t()
    {
        x = 0;
        y = 0;
        z = 0;
        theta = 0;
    }

    sbpl_xyz_theta_cell_t(int x_, int y_, int z_, int theta_)
    {
        x = x_;
        y = y_;
        z = z_;
        theta = theta_;
    }

    bool operator==(const sbpl_xyz_theta_cell_t cell) const
    {
        return x == cell.x && y == cell.y && z == cell.z && theta == cell.theta;
    }

    bool operator<(const sbpl_xyz_theta_cell_t cell) const
    {
        return x < cell.x || (x == cell.x && (y < cell.y
            || (y == cell.y && ( z < cell.z
            || (z==cell.z && theta < cell.theta)))));
    }

    int x;
    int y;
    int z;
    int theta;
};

class sbpl_xyz_theta_pt_t   // 记录每条运动基元的状态
{
public:
    sbpl_xyz_theta_pt_t()
    {
        x = 0;
        y = 0;
        z = 0;
        theta = 0;
    }

    sbpl_xyz_theta_pt_t(double x_, double y_, double z_, double theta_)
    {
        x = x_;
        y = y_;
        z = z_;
        theta = theta_;
    }

    bool operator==(const sbpl_xyz_theta_pt_t p) const
    {
        return x == p.x && y == p.y && z==p.z && theta == p.theta;
    }

    bool operator<(const sbpl_xyz_theta_pt_t p) const
    {
        return x < p.x || (x == p.x && (y < p.y
                || (y == p.y && ( z<p.z
                || z==p.z && theta < p.theta))));
    }

    double x;
    double y;
    double z;
    double theta;
};




#endif //INC_3DPLAN_COMMON_H
