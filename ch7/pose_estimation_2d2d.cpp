#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// #include "extra.h" // use this if in OpenCV2 
using namespace std;
using namespace cv;

/**********************************************
 * 本程序演示了如何使用2D-2D的特征匹配估计相机运动   *
 **********************************************/

void find_feature_matches(const Mat& img_1,
                          const Mat& img_2,
                          std::vector<KeyPoint>& keypoints_1,
                          std::vector<KeyPoint>& keypoints_2,
                          std::vector<DMatch>& matches);

void pose_estimation_2d2d(std::vector<KeyPoint> keypoints_1,
                          std::vector<KeyPoint> keypoints_2,
                          std::vector<DMatch> matches,
                          Mat& R, Mat& t);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d& p, const Mat& K);

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        cout << "usage: pose_estimation_2d2d img1 img2" << endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到了 " << matches.size() << " 组匹配点" << endl;

    // -- 估计两张图像间运动
    Mat R, t;
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

    // -- 验证 E = t^R * scale
    // 对极约束是等式为 0 的约束，所以乘上一个非零标量，等式依然成立
    // 这里 t^R 不一定相等的原因是 E 通过奇异值分解来计算的
    // 对角矩阵取值为 diag(1, 1, 0) * scale。 不唯一，导致算出的 t^R 相差一个尺度因子
    Mat t_x = (Mat_<double>(3, 3) <<
                    0, -t.at<double>(2, 0), t.at<double>(1, 0),
                    t.at<double>(2, 0), 0, -t.at<double>(0, 0),
                    -t.at<double>(1.0), t.at<double>(0, 0), 0);

    cout << " t^R =" << endl << t_x * R << endl;

    //-- 验证对极约束
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    for (DMatch m: matches) {
        Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        Mat     y1  = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
        Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
        Mat     y2  = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
        Mat     d   = y2.t() * t_x * R * y1;
        cout << "epipolar constraint = " << d << endl;
    }
    return 0;
}

void find_feature_matches(const Mat& img_1,
                          const Mat& img_2,
                          std::vector<KeyPoint>& keypoints_1,
                          std::vector<KeyPoint>& keypoints_2,
                          std::vector<DMatch>& matches)
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create(); // used in OpenCV3
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2 
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descriptors_1, descriptors_2, match);

    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++) {
        double dist = match[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }

    printf ("-- Max dist : %f\n", max_dist);
    printf ("-- Min dist : %f\n", min_dist);

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++) {
        if(match[i].distance <= max (2 * min_dist, 30.0)) {
            matches.push_back(match[i]);
        }
    }
}


Point2d pixel2cam(const Point2d& p, const Mat& K)
{
    // u = fx * (X / Z) + cx
    // v = fy * (Y / Z) + cy
    // 下面默认 Z = 1 了，求的是归一化坐标
    return Point2d((p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                   (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}


void pose_estimation_2d2d(std::vector<KeyPoint> keypoints_1,
                          std::vector<KeyPoint> keypoints_2,
                          std::vector<DMatch> matches,
                          Mat& R, Mat& t)
{
    // 相机内参,TUM Freiburg2
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for (auto &matche : matches) {
        // queryIdx 为 query 描述子的索引, match 函数中前面的那个描述子
        points1.push_back(keypoints_1[matche.queryIdx].pt); // pt 为关键点的点坐标
        // trainIdx 为 train 描述子的索引, match 函数中后面的那个描述子
        points2.push_back(keypoints_2[matche.trainIdx].pt);
    }

    // -- 计算基础矩阵
    Mat fundamental_matrix;
    // 书上基础矩阵需要内参的原因是给的变量是归一化坐标，这里直接给的是像素坐标，就不用传入内参转换了
    fundamental_matrix = findFundamentalMat(points1, points2, CV_FM_8POINT);
    cout << "fundamental_matrix is" << endl << fundamental_matrix << endl;

    // -- 计算本质矩阵
    // https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    Point2d principal_point(325.1, 249.7);	// 相机光心, TUM dataset 标定值 cx, cy
    double focal_length = 521;			    // 相机焦距, TUM dataset 标定值 fx, fy (这里默认是 fx = fy 的）
    Mat essential_matrix;
    // 本质矩阵需要的是归一化坐标，所以要传入相机内参将像素坐标转换成归一化坐标
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
    cout << "essential_matrix is" << endl << essential_matrix << endl;

    //-- 计算单应矩阵
    Mat homography_matrix;
    homography_matrix = findHomography(points1, points2, RANSAC, 3);
    cout << "homography_matrix is" << endl << homography_matrix << endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    cout << "R is" << endl << R <<endl;
    cout << "t is" << endl << t <<endl;
    
}
