#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
#include <ctime>
#include <climits>

#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace std;
using namespace g2o;
namespace fs = boost::filesystem;

/********************************************
 * 本节演示了RGBD上的稀疏直接法
 ********************************************/

// 一次测量的值，包括一个世界坐标系下三维点与一个灰度值
struct Measurement
{
    Measurement ( Eigen::Vector3d p, float g ) : pos_world ( p ), grayscale ( g ) {}
    Eigen::Vector3d pos_world;
    float grayscale;
};

// 像素坐标到相机坐标系
inline Eigen::Vector3d project2Dto3D(int x, int y, int d, float fx, float fy, float cx, float cy, float scale) {
    float zz = float (d) / scale;
    float xx = zz * (x - cx) / fx;
    float yy = zz * (y - cy) / fy;
    return Eigen::Vector3d(xx, yy, zz);
}

inline Eigen::Vector2d project3Dto2D(float x, float y, float z, float fx, float fy, float cx, float cy) {
    float u = fx * x / z + cx;
    float v = fy * y / z + cy;
    return Eigen::Vector2d(u, v);
}

// 直接法估计位姿
// 输入：测量值（空间点的灰度），新的灰度图，相机内参； 输出：相机位姿
// 返回：true为成功，false失败
bool poseEstimationDirect(const vector<Measurement>& measurements,
                          cv::Mat* gray, Eigen::Matrix3f& K, Eigen::Isometry3d& Tcw);

// project a 3d point into an image plane, the error is photometric error
// an unary edge with one vertex SE3Expmap (the pose of camera)
class EdgeSE3ProjectDirect: public BaseUnaryEdge<1, double, VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectDirect() {}

    EdgeSE3ProjectDirect(Eigen::Vector3d point, float fx, float fy, float cx, float cy, cv::Mat* image)
            : x_world_(point), fx_(fx), fy_(fy), cx_(cx), cy_(cy), image_(image) {}

    virtual void computeError() {
        const VertexSE3Expmap* v  = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector3d x_local = v->estimate().map(x_world_); // 世界坐标系到相机坐标系 rp + t
        float x, y;
        x = static_cast<float>(x_local[0] * fx_ / x_local[2] + cx_);
        y = static_cast<float>(x_local[1] * fy_ / x_local[2] + cy_);
        // 和 x = x_local[0] * fx_ / x_local[2] + cx_; 一样

        // check x,y is in the image
        if (x - 4 < 0 || (x + 4) > image_->cols || (y - 4) < 0 || (y + 4) > image_->rows) {
            _error (0, 0) = 0.0; // 边缘灰度值不稳定，不考虑
            this->setLevel(1);
        } else {
            _error(0, 0) = getPixelValue(x, y) - _measurement;
        }
    }

    // plus in manifold
    virtual void linearizeOplus() {
        if (level() == 1) { // 边缘不考虑
            _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
            return;
        }

        VertexSE3Expmap* vtx = static_cast<VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector3d xyz_trans = vtx->estimate().map(x_world_);   // q in book 世界坐标系到相机坐标系

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0 / xyz_trans[2];
        double invz_2 = invz * invz;

        float u = x * fx_* invz + cx_; // 相机坐标系到像素坐标系
        float v = y * fy_* invz + cy_;

        // jacobian from se3 to u,v
        // NOTE that in g2o the Lie algebra is (\omega, \epsilon), where \omega is so(3) and \epsilon the translation
        Eigen::Matrix<double, 2, 6> jacobian_uv_ksai;

        jacobian_uv_ksai(0, 0) = -x * y * invz_2 * fx_;
        jacobian_uv_ksai(0, 1) = (1 + (x * x * invz_2)) * fx_;
        jacobian_uv_ksai(0, 2) = -y * invz * fx_;
        jacobian_uv_ksai(0, 3) = invz * fx_;
        jacobian_uv_ksai(0, 4) = 0;
        jacobian_uv_ksai(0, 5) = -x * invz_2 * fx_;

        jacobian_uv_ksai(1, 0) = -(1 + y * y * invz_2) * fy_;
        jacobian_uv_ksai(1, 1) = x * y * invz_2 * fy_;
        jacobian_uv_ksai(1, 2) = x * invz * fy_;
        jacobian_uv_ksai(1, 3) = 0;
        jacobian_uv_ksai(1, 4) = invz * fy_;
        jacobian_uv_ksai(1, 5) = -y * invz_2 * fy_;

        Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;

        jacobian_pixel_uv(0, 0) = (getPixelValue(u + 1, v) - getPixelValue(u - 1, v)) / 2;
        jacobian_pixel_uv(0, 1) = (getPixelValue(u, v + 1) - getPixelValue(u, v - 1)) / 2;

        _jacobianOplusXi = jacobian_pixel_uv * jacobian_uv_ksai;
    }

    // dummy read and write functions because we don't care...
    virtual bool read(std::istream& in) {}
    virtual bool write(std::ostream& out) const {}

protected:
    // get a gray scale value from reference image (bilinear interpolated)
    // https://blog.csdn.net/xbinworld/article/details/65660665
    inline float getPixelValue(float x, float y) {
        // cout << "image step: " << image_->step << endl;
        // image width: 640, height: 480, image_step: 640
        uchar* data = &image_->data[int(y) * image_->step + int(x)];
        float xx = x - floor(x);
        float yy = y - floor(y);
        return float((1 - xx) * (1 - yy) * data[0] +
                     xx * (1 - yy) * data[1] +
                     (1 - xx) * yy * data[image_->step] +
                     xx * yy * data[image_->step + 1]);
    }

public:
    Eigen::Vector3d x_world_;                 // 3D point in world frame
    float cx_ = 0, cy_ = 0, fx_ = 0, fy_ = 0; // Camera intrinsics
    cv::Mat* image_ = nullptr;                // reference image
};

int main (int argc, char** argv)
{
    if (argc != 2) {
        cout << "usage: useLK path_to_dataset" << endl;
        return 1;
    }

    srand((unsigned int)time(0));
    fs::path path_to_dataset(argv[1]);
    fs::path file ("associate.txt");
    fs::path associate_file = path_to_dataset / file;

    cout << "path: " << associate_file.string() << endl;

    ifstream fin(associate_file.string());

    string rgb_file, depth_file, time_rgb, time_depth;
    cv::Mat color, depth, gray;
    vector<Measurement> measurements;

    // 相机内参
    float cx = 325.5;
    float cy = 253.5;
    float fx = 518.0;
    float fy = 519.0;
    float depth_scale = 1000.0;
    Eigen::Matrix3f K;
    K << fx, 0.f, cx, 0.f, fy, cy, 0.f, 0.f, 1.0f;

    Eigen::Isometry3d Tcw = Eigen::Isometry3d::Identity();

    cv::Mat prev_color;
    // 我们以第一个图像为参考，对后续图像和参考图像做直接法
    for (int index = 0; index < 10; index++) {
        cout << "*********** loop " << index << " ************" << endl;

        fin >> time_rgb >> rgb_file >> time_depth >> depth_file;

        color = cv::imread((path_to_dataset / rgb_file).string());
        depth = cv::imread((path_to_dataset / depth_file).string(), -1);
        if (color.data == nullptr || depth.data == nullptr) {
            continue;
        }
        cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);

        if (index == 0) {
            // select the pixels with high gradiants
            for (int x = 10; x < gray.cols - 10; x++) {
                for (int y = 10; y < gray.rows - 10; y++) {
                    Eigen::Vector2d delta(
                        gray.ptr<uchar>(y)[x + 1] - gray.ptr<uchar>(y)[x - 1],
                        gray.ptr<uchar>(y + 1)[x] - gray.ptr<uchar>(y - 1)[x]
                    );
                    if (delta.norm() < 50) {
                        continue;
                    }

                    ushort d = depth.ptr<ushort>(y)[x];
                    if (d == 0) {
                        continue;
                    }

                    Eigen::Vector3d p3d = project2Dto3D(x, y, d, fx, fy, cx, cy, depth_scale);
                    auto grayscale = float(gray.ptr<uchar>(y)[x]);
                    measurements.push_back(Measurement(p3d, grayscale));
                }
            }
            prev_color = color.clone();
            cout<<"add total "<<measurements.size()<<" measurements."<<endl;
            continue;
        }

        // 使用直接法计算相机运动
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        poseEstimationDirect(measurements, &gray, K, Tcw);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "direct method costs time: " << time_used.count() << " seconds." << endl;
        cout << "Tcw = " << endl << Tcw.matrix() << endl << "-------" << endl;

        // plot the feature points
        cv::Mat img_show(color.rows * 2, color.cols, CV_8UC3);
        prev_color.copyTo(img_show(cv::Rect(0, 0, color.cols, color.rows)));
        color.copyTo(img_show(cv::Rect(0, color.rows, color.cols, color.rows)));
        for (Measurement m : measurements) {
            if (rand() > RAND_MAX / 5) {
                continue;
            }

            Eigen::Vector3d p = m.pos_world; // 第一帧相机坐标系
            // 第一帧的相机坐标系到像素坐标系
            Eigen::Vector2d pixel_prev = project3Dto2D(p(0, 0), p(1, 0), p(2, 0), fx, fy, cx, cy);
            Eigen::Vector3d p2 = Tcw * m.pos_world;  // 当前帧相机坐标系
            // 当前帧的相机坐标系到像素坐标系
            Eigen::Vector2d pixel_now = project3Dto2D(p2(0, 0), p2(1, 0), p2(2, 0), fx, fy, cx, cy);
            if (pixel_now(0, 0) < 0 || pixel_now(0, 0) >= color.cols ||
                pixel_now(1, 0) < 0 || pixel_now(1, 0) >= color.rows) {
                continue;
            }

            float b = 0;
            float g = 190;
            float r = 0;
            img_show.ptr<uchar>(pixel_prev(1, 0))[int(pixel_prev(0, 0)) * 3] = b;
            img_show.ptr<uchar>(pixel_prev(1, 0))[int(pixel_prev(0, 0)) * 3 + 1] = g;
            img_show.ptr<uchar>(pixel_prev(1, 0))[int(pixel_prev(0, 0)) * 3 + 2] = r;

            img_show.ptr<uchar>(pixel_now(1, 0) + color.rows)[int(pixel_now(0, 0)) * 3] = b;
            img_show.ptr<uchar>(pixel_now(1, 0) + color.rows)[int(pixel_now(0, 0)) * 3 + 1] = g;
            img_show.ptr<uchar>(pixel_now(1, 0) + color.rows)[int(pixel_now(0, 0)) * 3 + 2] = r;
            cv::circle(img_show, cv::Point2d(pixel_prev(0, 0), pixel_prev (1, 0)), 4, cv::Scalar ( b,g,r ), 2 );
            cv::circle(img_show, cv::Point2d(pixel_now (0, 0), pixel_now  (1, 0) + color.rows), 4, cv::Scalar(b, g, r), 2 );
        }
        cv::imshow("result", img_show);
        cv::waitKey(0);

    }
    return 0;
}

bool poseEstimationDirect(const vector<Measurement>& measurements,
                          cv::Mat* gray, Eigen::Matrix3f& K, Eigen::Isometry3d& Tcw) {
    // 初始化g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;  // 求解的向量是6＊1的
    DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>();
    DirectBlock* solver_ptr = new DirectBlock(linearSolver);
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); // L-M
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // 因为第一帧的相机坐标系和世界坐标系重合
    // 所以 T_cw 即可以看成世界坐标系到当前帧相机坐标系的转换
    // 又可以看成第一帧相机坐标系到当前帧相机坐标系的转换
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setEstimate(g2o::SE3Quat(Tcw.rotation(), Tcw.translation()));
    pose->setId(0);
    optimizer.addVertex(pose);

    // 添加边
    int id = 1;
    for (Measurement m : measurements) {
        EdgeSE3ProjectDirect* edge = new EdgeSE3ProjectDirect(
                m.pos_world,           //该点在世界坐标系的 pose
                K(0, 0), K(1, 1), K(0, 2), K(1, 2), gray // gray 是当前帧的图片
        );
        edge->setVertex(0, pose);
        edge->setMeasurement(m.grayscale); // 该点的灰度值
        edge->setInformation(Eigen::Matrix<double,1,1>::Identity());
        edge->setId(id++);
        optimizer.addEdge(edge);
    }

    cout << "edges in graph: " << optimizer.edges().size() << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);
    Tcw = pose->estimate();
}
