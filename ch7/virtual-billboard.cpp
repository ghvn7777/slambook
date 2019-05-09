/****************************
 * 实现虚拟广告牌的效果。
 * 提供两张图，一张是“计算机视觉life”公众号的logo，另外一张是带广告牌的原图，请用单应矩阵实现将原图中广告牌替换为提供的logo的效果。
 * 利用OpenCV函数，通过鼠标点击来选择要替换的广告牌的四个顶点。
 *
* 本程序学习目标：
 * 理解掌握单应矩阵的使用
 *
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2018.11
****************************/

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

struct userdata {
    Mat im;
    vector<Point2f> points;
};


void mouseHandler(int event, int x, int y, int flags, void* data_ptr) {
    if (event == EVENT_LBUTTONDOWN) {
        userdata *data = ((userdata *) data_ptr);
        circle(data->im, Point(x, y), 3, Scalar(0, 255, 255), 5, CV_AA);
        imshow("Image", data->im);
        if (data->points.size() < 4) {
            data->points.push_back(Point2f(x,y));
        }
    }
}



int main(int argc, char** argv) {
    // Read in the image.
    //Mat im_src = imread("first-image.jpg");
    Mat im_src = imread("cvlife.jpg");
    Size size = im_src.size();

    // Create a vector of points.
    vector<Point2f> pts_src;
    pts_src.push_back(Point2f(0,0));
    pts_src.push_back(Point2f(size.width - 1, 0));
    pts_src.push_back(Point2f(size.width - 1, size.height -1));
    pts_src.push_back(Point2f(0, size.height - 1 ));

    // Destination image
    //Mat im_dst = imread("times-square.jpg");
    Mat im_dst = imread("ad.jpg");

    // Set data for mouse handler
    Mat im_temp = im_dst.clone();
    userdata data;
    data.im = im_temp;

    //show the image
    imshow("Image", im_temp);
    cout << "Click on four corners of a billboard and then press ENTER" << endl;
    //set the callback function for any mouse event
    setMouseCallback("Image", mouseHandler, &data);
    waitKey(0);

    // ----------  开始你的代码  --------------
    Mat output_img;
    Mat h = findHomography(pts_src, data.points);
    warpPerspective(im_src, output_img, h, im_dst.size());
    // imshow("test", output_img);

    Mat m1 = Mat::zeros(im_src.size(), CV_8UC1);
    m1 = Scalar(255);
    Mat mask_output;
    warpPerspective(m1, mask_output, h, im_dst.size());
    // namedWindow("Mask Result", CV_WINDOW_FREERATIO);
    // imshow("Mask Result", mask_output);

    Mat result1;
    add(output_img, output_img, result1, mask_output);
    // imshow("result1", result1);

    Mat result2;
    bitwise_not(mask_output, mask_output);
    add(im_dst, result1, result2, mask_output);
	  // imshow("result2", result2);

    Mat result;
    add(output_img, result2, result);
    // ----------  结束你的代码  --------------
    // Display image.
    imshow("Image", result);
    waitKey(0);
    return 0;
}
