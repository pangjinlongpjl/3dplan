#include "utils.hpp"
#include "Eigen/Eigen"
#include "Eigen/Dense"

/**
 * @brief readJson
 * @param file name
 * @return json
 */
pair<json, bool> readJson(string path){
    json j;
    std::ifstream file(path);
    if (!file.is_open()) {
        file.clear();
        return {j,false};
    }
    file >>  j;
    return {j, true};
}

/**
 * @brief writeJson
 * @param file name
 * @param json
 */
void writeJson(string path, json j){
    std::ofstream file(path,
    std::ios::trunc | std::ios::out | std::ios::in);
    file <<j;
}

/**
 * @brief transform  类似python的  a = [data for i in b function()]
 * @param x:data
 * @param fn:function
 * @return
 */
Eigen::VectorXd transform(Eigen::VectorXd &x, std::function<double(double)> fn){
    Eigen::VectorXd ans(x.size());
    for (int i = 0; i < x.size(); ++i) {
        ans(i) = fn(x(i));
    }
    return ans;
}

