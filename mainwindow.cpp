
#include <QDebug>

#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "surfacegraph.h"
#include "QPlot3D.h"    // 三维曲线
#include "readtiff.h"
#include <iostream>

#include <QBoxLayout>
#include <QVector>
#include <QVector2D>
#include <QMessageBox>

#include <QtDataVisualization/QtDataVisualization>
using namespace QtDataVisualization;

using namespace std;
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
//    connect(ui->start,SIGNAL(clicked()),SLOT(startTimer()));
//    connect(ui->end, SIGNAL(clicked()), SLOT(endTimer()));
////    if(timer!=NULL) delete timer;
//    timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(mytimerSlot()));
//    timer->start(500);
//    oj289

    QVector<QVector<double>> path;
    path.push_back({1,1});
    path.push_back({2,2});
    path.push_back({3,3});
    qDebug() << path[0] << endl;

    rt = new ReadTiff();
    rp = new readPrimitives();

    QLinearGradient grBtoY(0, 0, 1, 100);
    grBtoY.setColorAt(1.0, Qt::black);
    grBtoY.setColorAt(0.67, Qt::blue);
    grBtoY.setColorAt(0.33, Qt::red);
    grBtoY.setColorAt(0.0, Qt::yellow);
    QPixmap pm(24, 100);
    QPainter pmp(&pm);
    pmp.setBrush(QBrush(grBtoY));
    pmp.setPen(Qt::NoPen);
    pmp.drawRect(0, 0, 24, 100);

    ui->gradientBtoYPB->setIcon(QIcon(pm));
    ui->gradientBtoYPB->setIconSize(QSize(24, 100));

    QLinearGradient grGtoR(0, 0, 1, 100);
    grGtoR.setColorAt(1.0, Qt::darkGreen);
    grGtoR.setColorAt(0.5, Qt::yellow);
    grGtoR.setColorAt(0.2, Qt::red);
    grGtoR.setColorAt(0.0, Qt::darkRed);
    pmp.setBrush(QBrush(grGtoR));
    pmp.drawRect(0, 0, 24, 100);
    ui->gradientGtoRPB->setIcon(QIcon(pm));
    ui->gradientGtoRPB->setIconSize(QSize(24, 100));


    Q3DSurface *graph = new Q3DSurface();
    QWidget *container = QWidget::createWindowContainer(graph);
    //! [0]

    if (!graph->hasContext()) {
        QMessageBox msgBox;
        msgBox.setText("Couldn't initialize the OpenGL context.");
        msgBox.exec();
//        return -1;
    }

    QSize screenSize = graph->screen()->size();
    container->setMinimumSize(QSize(screenSize.width() / 2, screenSize.height() / 1.6));
    container->setMaximumSize(screenSize);
    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    container->setFocusPolicy(Qt::StrongFocus);




    //! [1]
//    QWidget *widget = new QWidget;
    QHBoxLayout *hLayout = new QHBoxLayout(ui->widget);
//    QVBoxLayout *vLayout = new QVBoxLayout();
    hLayout->addWidget(container, 1);
//    hLayout->addLayout(vLayout);
//    vLayout->setAlignment(Qt::AlignTop);
    //! [1]

    ui->widget->setWindowTitle(QStringLiteral("Surface example"));

    //创建数据对象
    QSurfaceDataArray *data = new QSurfaceDataArray;
//    QSurfaceDataRow *dataRow1 = new QSurfaceDataRow;
//    QSurfaceDataRow *dataRow2 = new QSurfaceDataRow;
    //为数据对象赋值
    for(int i=0; i<rt->demYsize; i+=10){
        QSurfaceDataRow *dataRow = new QSurfaceDataRow;
        for (int j = 0; j < rt->demXsize; j+=10) {
//            if(rt.dataz(i,j)<=0) continue;
            *dataRow<<QVector3D(rt->datax(i,j),rt->dataz(i,j),rt->datay(i,j));
//            *data << QVector3D(rt.datax(i,j),rt.dataz(i,j),rt.datay(i,j));
        }
         *data << dataRow;
    }

    QValue3DAxis *axisX=new QtDataVisualization::QValue3DAxis;
    axisX->setTitle("East--West");
    axisX->setTitleVisible(true);
    axisX->setLabelFormat("%.1f Meter");
    axisX->setAutoAdjustRange(true);
    axisX->setRange(rt->datax.minCoeff(),rt->datax.maxCoeff());
    graph->setAxisX(axisX);

    QValue3DAxis *axisY=new QtDataVisualization::QValue3DAxis;
    axisY->setTitle("Degree");
    axisY->setTitleVisible(true);
    axisY->setRange(rt->dataz.minCoeff(),rt->dataz.maxCoeff());
    graph->setAxisY(axisY);

    QtDataVisualization::QValue3DAxis *axisZ=new QtDataVisualization::QValue3DAxis;
    axisZ->setTitle("South--North");
    axisZ->setTitleVisible(true);
    axisZ->setRange(rt->datay.minCoeff(),rt->datay.maxCoeff());
    graph->setAxisZ(axisZ);

    //创建序列，加入数据，添加到Q3DSurface
    QSurface3DSeries *series = new QSurface3DSeries;
    series->dataProxy()->resetArray(data);
    graph->addSeries(series);

    //! [7]
    QLinearGradient gr;
    gr.setColorAt(0.0, Qt::black);
    gr.setColorAt(0.33, Qt::blue);
    gr.setColorAt(0.67, Qt::red);
    gr.setColorAt(1.0, Qt::yellow);

    graph->seriesList().at(0)->setBaseGradient(gr);
    graph->seriesList().at(0)->setColorStyle(Q3DTheme::ColorStyleRangeGradient);
    graph->show();

    /**
     * @brief plot
     */
    QPlot3D *plot = ui->widget_2;   // 必须用指针，且需要初始化
    auto dat=rp->ms.mps;
    vector<vector<float>> color = {
                {0.f, 0.00f, 0.00f, 0.00f},
                {0.f,0.25f, 0.80f, 0.54f},
                {0.f, 0.83f, 0.14f, 0.14f},
                 {0.f, 1.00f, 0.54f, 0.00f},
                {0.f, 0.00f, 0.00f, 0.00f},
                 {0.f, 0.47f, 0.25f, 0.80f},
                 {0.f,0.25f, 0.80f, 0.54f},
                {0.f, 0.83f, 0.14f, 0.14f},
                {0.f, 1.00f, 0.54f, 0.00f},
                {0.f, 0.47f, 0.25f, 0.80f},
                {0.f,0.25f, 0.80f, 0.54f}
        };
    int car=0;
    for (auto it : dat) {
        QCurve3D *curve = new QCurve3D; // 必须用指针，且需要初始化
        for (int x = 0; x < it.rows(); ++x) {
                curve->addData(it(x,0),it(x,1), it(x,2));
        }
        QColor c;
        c.setRgbF(color[car][0],color[car][1],color[car][2],color[car][3]);
        curve->setColor(c);
        car++;
        car = car%color.size();
        curve->setLineWidth(2);
        plot->addCurve(curve);
    }

    ui->widget_2->show();

}

MainWindow::~MainWindow()
{
    delete ui;
}

//void MainWindow::mytimerSlot(){
//    ui->lcdNumber->display(cnt);
//    cnt++;
//}

//void MainWindow::startTimer(){
//    qDebug()<<"start timer" << endl;
//    if(!timer->isActive())
//        timer->start(500);  // timer interval
//}

//void MainWindow::endTimer(){
//    qDebug()<<"stop timer" << endl;
//    if(timer->isActive())
//        timer->stop();
//}
