#include "readtiff.h"
//#include "iostream"
#include <QTime>
#include <QMessageBox>
using namespace std;

ReadTiff::ReadTiff()
{
    //计算程序运行时间方法
     QTime timedebuge;//声明一个时钟对象
     timedebuge.start();//开始计时

//    fromTiff();
    auto [j, isErr]=readJson("/home/lrm/workspace/learnQt/env.json");    // json 不适合读写数据量大的文件
    if( isErr == false ){
        fromTiff();
        qDebug()<<"第一段程序耗时："<<timedebuge.elapsed()/1000.0<<"s";//输出计时
        return ;
    }
    env=j;
    demXsize = env.size.first;
    demYsize = env.size.second;
    Eigen::MatrixXf Z = convert_vvd_to_matrix(env.map);
    Eigen::VectorXf xx = Eigen::VectorXf::LinSpaced( env.size.first, env.xrange.first, env.xrange.second);
    Eigen::VectorXf yy = Eigen::VectorXf::LinSpaced( env.size.second, env.yrange.first, env.yrange.second);
    auto [X, Y] = meshgrid(xx, yy);
//    std::cout << X << endl;

    datax = X;
    datay = Y;
    dataz = Z;

    qDebug()<<"第一段程序耗时："<<timedebuge.elapsed()/1000.0<<"s";//输出计时
}

void ReadTiff::fromTiff(){
    GDALAllRegister();  //

    QString file_path = "/home/lrm/workspace/demPlan/data/moon2.tif";
    GDALDatasetH *poDataset;
    poDataset = (GDALDatasetH *) GDALOpen(file_path.toStdString().c_str(), GA_ReadOnly);
    if( poDataset == NULL ){
        qDebug() << "Data File Read Failed! \n  ";
        QMessageBox *m= new QMessageBox();
        m->setText("打开tiff文件失败");
        m->show();
        return ;
//        return 0;
    }
    int nImgSizeX = GDALGetRasterXSize(poDataset);
    int nImgSizeY = GDALGetRasterYSize(poDataset);
    qDebug() << "img size:" << nImgSizeX << " * " << nImgSizeY << endl;

    /*
     *  图像行列号和地理空间坐标之间的变换关系
     *  Xgeo = GT(0) + Xpixel*GT(1) + Yline*GT(2)
     *  Ygeo = GT(3) + Xpixel*GT(4) + Yline*GT(5)
     */
    double tran[6];
    GDALGetGeoTransform(poDataset, tran);   //
    double xmin= tran[0];
    double xmax = tran[0] + nImgSizeX*tran[1] + nImgSizeY*tran[2];
    double ymin = tran[3];
    double ymax = tran[3] + nImgSizeX*tran[4] + nImgSizeY*tran[5];
    qDebug() << "x:[" << xmin << "," << xmax << "]" << endl;
    qDebug() << "y:[" << ymin << "," << ymax << "]" << endl;


    auto band = GDALGetRasterBand(poDataset, 1);
    auto DataType = GDALGetDataTypeName(GDALGetRasterDataType(band));
    qDebug() << "Data Type:" << DataType << endl;

    Eigen::VectorXf mat(nImgSizeY*nImgSizeX);
    CPLErr ce = GDALRasterIO(band, GF_Read,0,0, nImgSizeX,
                            nImgSizeY, mat.data(), nImgSizeX, nImgSizeY, GDT_Float32, 0, 0);
    if(ce == CPLErr::CE_Failure){
        qDebug()<< "read failse" << endl;
    }
    mat = (mat.array()<0).select(0, mat);   // <0 =0
//    std::cout << mat.transpose() << endl;

    Eigen::MatrixXf Z = construct2DArray(mat, nImgSizeY, nImgSizeX);
    Eigen::VectorXf xx = Eigen::VectorXf::LinSpaced( nImgSizeX, xmin, xmax);
    Eigen::VectorXf yy = Eigen::VectorXf::LinSpaced( nImgSizeY, ymin, ymax);
    auto [X, Y] = meshgrid(xx, yy);
//    std::cout << X << endl;

    datax = X;
    datay = Y;
    dataz = Z;

    demXsize = nImgSizeX;
    demYsize = nImgSizeY;

    env.map = convert_matrix_to_vv(Z);

    std::copy(std::begin(tran),std::end(tran),std::begin(env.tran));
    env.size={nImgSizeX, nImgSizeY};
    env.xrange={xmin,xmax};
    env.yrange={ymin,ymax};
    json j;
    j = env;
    writeJson("env.json", j);
}

void ReadTiff::setPath(QString path){

}


template <typename T>
QVector<QVector<double>> ReadTiff::construct2DArray( QVector< T > original, int m, int n )
{
    if( original.size() != m * n )
        return {};

    Eigen::MatrixXf ary2( m, n );
    QVector<QVector<double>> ans(m, QVector<double>(n));
    auto data = original.data();

    for( int i=0; i < m; ++i, data += n ){
        ary2.row(i)= Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(data, n);
        Eigen::VectorXd vd(n);
//        cout << ary2.row(i) << endl;
        vd = ary2.row(i).cast<double>();
        vd = (vd.array()<0).select(0, vd);   // <0 =0
//        cout << vd.transpose() << endl;
        ans[i] = QVector<double>::fromStdVector(std::vector<double>(vd.data(),vd.data()+n));
    }
    return ans;
}

template <typename T>
vector<vector<double>> ReadTiff::construct2DArray( vector< T > original, int m, int n )
{
    if( original.size() != m * n )
        return {};

    Eigen::MatrixXf ary2( m, n );
    vector<vector<double>> ans(m, vector<double>(n));
    auto data = original.data();

    for( int i=0; i < m; ++i, data += n ){
        ary2.row(i)= Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(data, n);
        Eigen::VectorXd vd(n);
//        cout << ary2.row(i) << endl;
        vd = ary2.row(i).cast<double>();
        vd = (vd.array()<0).select(0, vd);   // <0 =0
//        cout << vd.transpose() << endl;
        ans[i] = vector<double>(vd.data(),vd.data()+n);
    }
    return ans;
}


Eigen::MatrixXf ReadTiff::construct2DArray( Eigen::VectorXf original, int m, int n )
{
    if( original.size() != m * n )
        return {};

    Eigen::MatrixXf ary2( m, n );
//    vector<vector<double>> ans(m, vector<double>(n));
    auto data = original.data();

    for( int i=0; i < m; ++i, data += n ){
        ary2.row(i)= Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(data, n);
    }
    return ary2;
}

QPair<Eigen::MatrixXf,Eigen::MatrixXf>  ReadTiff::meshgrid(
                                    Eigen::VectorXf &vecX, Eigen::VectorXf &vecY)
{

    int vecXLength = vecX.size();
    int vecYLength = vecY.size();
    Eigen::MatrixXf meshX(vecYLength,vecXLength);
    Eigen::MatrixXf meshY(vecYLength,vecXLength);

    for (int i = 0; i < vecYLength; ++i)
    {
        meshX.row(i) = vecX;
    }

    for (int i = 0; i < vecXLength; ++i)
    {
        meshY.col(i) = vecY.transpose();
    }
    return {meshX, meshY};
}
