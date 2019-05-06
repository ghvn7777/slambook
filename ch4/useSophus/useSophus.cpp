#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.h"
#include "sophus/se3.h"

int main (int argc, char** argv) {
    //角轴 以 (0, 0, 1) 为轴，转 pi / 2, 沿Z轴转90度的旋转矩阵
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

    // 这些表达方式是等价的
    Sophus::SO3 SO3_R(R);              // Sophus::SO(3)可以直接从旋转矩阵构造
    Sophus::SO3 SO3_v(0, 0, M_PI / 2); // 从旋转向量构造，方向为旋转轴方向，模长为角度 theta 大小
    Eigen::Quaterniond q(R);           // 或者四元数
    Sophus::SO3 SO3_q(q);

    // 输出SO(3)时，以so(3)形式输出
    cout << "SO(3) from matrix: " << SO3_R << endl;
    cout << "SO(3) from vector: " << SO3_v << endl;
    cout << "SO(3) from quaternion: " << SO3_q << endl;
    // 使用对数映射获得它的李代数
    Eigen::Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl; //transpose 取转置，只是为了输出美观

    // hat 为向量到反对称矩阵
    cout << "so3 hat = " << endl << Sophus::SO3::hat(so3) << endl;

    // 相对的，vee为反对称到向量
    cout << "so3 hat vee = " << Sophus::SO3::vee(Sophus::SO3::hat(so3)).transpose() << endl;

    // 增量扰动模型的更新
    Eigen::Vector3d update_so3(1e-4, 0, 0); //假设更新量是这个
    // 左乘更新, (实际上是 exp(so3^)，程序帮我们做了这步)，P68: R = exp(phi^)
    Sophus::SO3 SO3_update = Sophus::SO3::exp(update_so3) * SO3_R;
    cout << "SO3 updated = " << SO3_update << endl;

    cout << "************我是分割线*************" << endl;

    Eigen::Vector3d t(1, 0, 0); // 沿X轴平移1
    Sophus::SE3 SE3_Rt(R, t);   // 用矩阵构造 SE(3)
    Sophus::SE3 SE3_qt(q, t);   // 用向量构造 SE(3)
    cout << "SE3 from R, t = " << endl << SE3_Rt << endl;
    cout << "SE3 from q, t = " << endl << SE3_qt << endl;

    // 李代数se(3) 是一个六维向量，方便起见先typedef一下
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_Rt.log();

    // se3 = [0.785398 -0.785398 0 0 0 1.5708]
    // 平移部分不是 1, 0, 0 的原因是平移部分不是李群部分的那个平移，差了一个雅克比
    cout << "se3 = " << se3.transpose() << endl; //平移在前，旋转在后
    cout << "se3 hat = " << endl << Sophus::SE3::hat(se3) << endl; // 向量转反对称矩阵（注意在 se3 上矩阵其实不是反对称的，只是我们拓展了 so3 的叫法）
    cout << "se3 hat vee = " << Sophus::SE3::vee(Sophus::SE3::hat(se3)).transpose() << endl; // 矩阵转向量
    cout << "se3 exp = " << Sophus::SE3::exp(se3) << endl; // 李代数转李群 (实际上是 exp(se3^)，程序帮我们做了这步)
    cout << "se3 log = " << Sophus::SE3::exp(se3).log() << endl; // 李群转李代数
    // se3 更新
    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4d;
    Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3) * SE3_Rt;
    cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;

    return 0;
}
