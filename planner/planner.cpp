#include "planner.h"

Planner::Planner()
{

}


void Planner::setGoal(QPointF g){
    this->g = g;
}

void Planner::setStart(QPointF s){
    this->s = s;
}
