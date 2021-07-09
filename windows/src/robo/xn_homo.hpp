#pragma once
// #include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>

namespace xn {
namespace homo {

struct HomoTransform {
  cv::Mat tvec;
  cv::Mat rotation;
};

bool readCameraInfo(std::string filename, cv::Mat &cam_mat,
                    cv::Mat &distortion_coeffs);

void GetHomoTransform(cv::Mat &H, HomoTransform &out);

void inverseProjectPoint(const cv::Point2i &uv, cv::Point3d &out,
                         const cv::Mat &cam_mat,
                         const HomoTransform &transform);

static std::vector<cv::Point3d> Face3D({
    cv::Point3d(6.825897, 6.760612, 4.402142),
    cv::Point3d(1.330353, 7.122144, 6.903745),   //#29 left brow right corner
    cv::Point3d(-1.330353, 7.122144, 6.903745),  //#34 right brow left corner
    cv::Point3d(-6.825897, 6.760612, 4.402142),  //#38 right brow right corner
    cv::Point3d(5.311432, 5.485328, 3.987654),   //#13 left eye left corner
    cv::Point3d(1.789930, 5.393625, 4.413414),   //#17 left eye right corner
    cv::Point3d(-1.789930, 5.393625, 4.413414),  //#25 right eye left corner
    cv::Point3d(-5.311432, 5.485328, 3.987654),  //#21 right eye right corner
    cv::Point3d(2.005628, 1.409845, 6.165652),   //#55 nose left corner
    cv::Point3d(-2.005628, 1.409845, 6.165652),  //#49 nose right corner
    cv::Point3d(2.774015, -2.080775, 5.048531),  //#43 mouth left corner
    cv::Point3d(-2.774015, -2.080775, 5.048531), //#39 mouth right corner
    cv::Point3d(0.000000, -3.116408,
                6.097667),                     //#45 mouth central bottom corner
    cv::Point3d(0.000000, -7.415691, 4.070434) //#6 chin corner
});

static std::vector<cv::Point3d> facecube_src(
    {cv::Point3d(10.0, 10.0, 10.0), cv::Point3d(10.0, 10.0, -10.0),
     cv::Point3d(10.0, -10.0, -10.0), cv::Point3d(10.0, -10.0, 10.0),
     cv::Point3d(-10.0, 10.0, 10.0), cv::Point3d(-10.0, 10.0, -10.0),
     cv::Point3d(-10.0, -10.0, -10.0), cv::Point3d(-10.0, -10.0, 10.0),
     cv::Point3d(0, 0, 0)});

} // namespace homo
} // namespace xn