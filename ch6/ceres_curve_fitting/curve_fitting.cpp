#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

using namespace std;
// 修改版，优化变量可以传递多个
// 代价函数的计算模型
struct CurveFittingCostFunction {
    CurveFittingCostFunction(double x, double y): _x(x), _y(y) {}

    // 残差的计算
    template <typename T>
    bool operator()(const T* const a,
                    const T* const b,
                    const T* const c,
                    T* residual) const {   // 残差
        // y-exp(ax^2+bx+c)
        residual[0] = T(_y) - ceres::exp(a[0] * T(_x) * T(_x) + b[0] * T(_x) + c[0]);
        return true;
    }
    const double _x, _y;    // x, y 数据
};

int main(int argc, char** argv)
{
    double a = 1.0;
    double b = 2.0;
    double c = 1.0;                     // 真实参数值

    int N = 100;                        // 数据点个数
    double w_sigma = 1.0;               // 噪声Sigma值
    cv::RNG rng;                        // OpenCV随机数产生器
    double abc[3] = {0, 0, 0};          // abc参数的估计值

    std::vector<double> x_data, y_data;      // 数据

    std::cout << "generating data: " << "\n";
    for (int i = 0; i < N; i++) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(a * x * x + b * x + c) + rng.gaussian(w_sigma));
        std::cout << x_data[i] << " " << y_data[i] << "\n";
    }

    Problem problem;
    for (int i = 0; i < N; i++) {
        problem.AddResidualBlock(new AutoDiffCostFunction<CurveFittingCostFunction, 1, 1, 1, 1>(
                new CurveFittingCostFunction(x_data[i], y_data[i])
                                 ),
                                 nullptr,
                                 &abc[0], &abc[1], &abc[2]);
    }

    // 配置求解器
    Solver::Options options;                       // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;   // 输出到 cout

    Solver::Summary summary;                // 优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);  // 开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    std::cout << "solve time cost = " << time_used.count() << " seconds.\n";

    // 输出结果
    std::cout << summary.BriefReport() << "\n";
    std::cout <<"estimated a, b, c = ";
    for (auto a : abc) std::cout << a <<" ";
    std::cout << "\n";

    return 0;
}
