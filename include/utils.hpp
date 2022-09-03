#pragma once
#ifndef UTILS_HPP
#define UTILS_HPP
#include "json.hpp"
#include <vector>
#include <fstream>
#include <fstream>
#include <Eigen/Eigen>
using json = nlohmann::json;
using namespace Eigen;
using namespace std;

// 将 vector<vector<>> 转换为 matrix
template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> convert_vvd_to_matrix(vector<vector<T> > vvd) {

    const std::size_t n_rows = vvd.size();
    const std::size_t n_cols = vvd.at(0).size();
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> result(n_rows, n_cols);
    result.row(0) = VectorXf::Map(&vvd[0][0], n_cols);

    for (std::size_t i = 1; i < n_rows; i++) {
        if (n_cols != vvd.at(i).size()) {
            char buffer[200];
            snprintf(buffer, 200,
                     "vvd[%ld] size (%ld) does not match vvd[0] size (%ld)",
                     i, vvd.at(i).size(), n_cols);
            string err_mesg(buffer);
            throw std::invalid_argument(err_mesg);
        }
        result.row(i) =Eigen::Matrix<T, 1, Eigen::Dynamic>::Map(&vvd[i][0], n_cols);
    }
    return result;
}

// 将 matrix 转换为 vector<vector<>>
template <typename T>
vector<vector<T>> convert_matrix_to_vv(Eigen::Matrix<T,Eigen::Dynamic, Eigen::Dynamic> mat){
    const std::size_t n_rows = mat.rows();
//    const std::size_t n_cols = mat.cols();
    vector<vector<T>> vv;//(n_rows, vector<T>(n_cols));
    for (std::size_t i = 0; i < n_rows; ++i) {
        //
        Eigen::Matrix<T, 1, Eigen::Dynamic> v=mat.row(i); // 不能直接用mat.row(i)的地址，因为是按列来保存的
        vv.push_back(vector<T>(v.data(), v.data()+v.size()));
    }
    return vv;
}



class MovementPrimitives{

public:
    vector<MatrixXf> mps;   // 运动基元  N*3

    /**
     * @brief to_json
     * @param json
     * @param 运动基元
     */
    friend inline void to_json(json& j, const MovementPrimitives& t){
        vector<vector<vector<float>>> data;
        for (auto it : t.mps) {     // 先转换为vector类型再保存
            const std::size_t n_rows = it.rows();
//            const std::size_t n_cols = it.cols();
            vector<vector<float>> vv;//(n_rows, vector<T>(n_cols));
            for (std::size_t i = 0; i < n_rows; ++i) {
                // qtcreator 没有 Eigen::Vector<T,Eigen::Dynamic>
                Eigen::VectorXf v=it.row(i); // 不能直接用mat.row(i)的地址，因为是按列来保存的
                vv.push_back(vector<float>(v.data(), v.data()+v.size()));
            }
            data.push_back(vv);
        }
        j = json{{"mps", data}};
    }

    /**
     * @brief from_json
     * @param json
     * @param 运动基元
     */
    friend inline void from_json(const json& j, MovementPrimitives& t){
        vector<vector<vector<float>>> arg;
        j.at("mps").get_to<vector<vector<vector<float>>>>(arg);
        for(auto d1: arg){
            const std::size_t n_rows = d1.size();
            const std::size_t n_cols = d1.at(0).size();
            Eigen::MatrixXf result(n_rows, n_cols);
            result.row(0) = VectorXf::Map(&d1[0][0], n_cols);

            for (std::size_t i = 1; i < n_rows; i++) {
                // qtcreator 没有 Eigen::Vector<T,Eigen::Dynamic>
                result.row(i) =Eigen::VectorXf::Map(&d1[i][0], n_cols);
            }
            t.mps.push_back(result);
        }
    }

};


class Environment{

public:
    vector<vector<float>> map;
    /*
     *  图像行列号和地理空间坐标之间的变换关系
     *  Xgeo = GT(0) + Xpixel*GT(1) + Yline*GT(2)
     *  Ygeo = GT(3) + Xpixel*GT(4) + Yline*GT(5)
     */
    float tran[6];
    pair<int,int> size;    // img size x, y;
    pair<float,float> xrange;
    pair<float,float> yrange;

    friend inline void to_json(json& j, const Environment& env){
        j = json{
                {"map", env.map},
                {"tran", env.tran},
                {"size", env.size},
                {"xrange",env.xrange},
                {"yrange",env.yrange}
        };
    }

    friend inline void from_json(const json& j, Environment& env){
        j.at("map").get_to<vector<vector<float>>>(env.map);
        j.at("tran").get_to(env.tran);
        j.at("size").get_to(env.size);
        j.at("xrange").get_to(env.xrange);
        j.at("yrange").get_to(env.yrange);
    }

};

/**
 * @brief readJson
 * @param path
 * @return json and readfile flag
 */
pair<json, bool> readJson(string path);

/**
 * @brief writeJson
 * @param file name
 * @param json
 */
void writeJson(string path, json j);

/**
 * @brief transform  类似python的  a = [data for i in b function()]
 * @param x:data
 * @param fn:function
 * @return
 */
Eigen::VectorXd transform(Eigen::VectorXd &x, std::function<double(double)> fn);


#endif
