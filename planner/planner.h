#ifndef PLANNER_H
#define PLANNER_H
#include <QObject>
#include <QThread>
#include <QPointF>

#include <sbpl/planners/planner.h>

class Planner:public QThread
{
    Q_OBJECT
public:
    Planner();
    void setStart(QPointF s);
    void setGoal(QPointF g);
    QPointF getStart(){ return s;};
    QPointF getGoal(){ return g;};
private:
    QPointF s, g;

};

#endif // PLANNER_H
