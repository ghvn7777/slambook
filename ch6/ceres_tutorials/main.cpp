#include <iostream>
#include <ceres/ceres.h>

struct CostFunctior {
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};

class QuadraticCostFunction : public ceres::SizedCostFunction<1, 1> {
  public:
    virtual ~QuadraticCostFunction() {}
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const {
        const double x = parameters[0][0];
        residuals[0] = 10 - x;

        if (jacobians != NULL && jacobians[0] != NULL) {
            jacobians[0][0] = -1;
        }
        return true;
    }
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    double initial_x = 5.0;
    double x = initial_x;

    ceres::Problem problem;

    ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<CostFunctior, 1, 1>(new CostFunctior);
    problem.AddResidualBlock(cost_function, NULL, &x);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x
              << " -> " << x << "\n";
    return 0;
}