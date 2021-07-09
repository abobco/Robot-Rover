#include "xn_homo.hpp"

namespace xn {
namespace homo {

bool readCameraInfo(std::string filename, cv::Mat &cam_mat,
                    cv::Mat &distortion_coeffs) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened())
    return false;
  fs["camera_matrix"] >> cam_mat;
  fs["distortion_coefficients"] >> distortion_coeffs;
  return true;
}

void GetHomoTransform(cv::Mat &H, HomoTransform &out) {
  // Normalization to ensure that ||c1|| = 1
  double norm = sqrt(H.at<double>(0, 0) * H.at<double>(0, 0) +
                     H.at<double>(1, 0) * H.at<double>(1, 0) +
                     H.at<double>(2, 0) * H.at<double>(2, 0));
  H /= norm;
  cv::Mat c1 = H.col(0);
  cv::Mat c2 = H.col(1);
  cv::Mat c3 = c1.cross(c2);
  cv::Mat tvec = H.col(2);
  cv::Mat R(3, 3, CV_64F);
  for (int i = 0; i < 3; i++) {
    R.at<double>(i, 0) = c1.at<double>(i, 0);
    R.at<double>(i, 1) = c2.at<double>(i, 0);
    R.at<double>(i, 2) = c3.at<double>(i, 0);
  }

  cv::Mat W, U, Vt;
  SVDecomp(R, W, U, Vt);
  out.rotation = U * Vt;
  out.tvec = tvec;
}

void inverseProjectPoint(const cv::Point2i &uv, cv::Point3d &out,
                         const cv::Mat &cam_mat,
                         const HomoTransform &transform) {
  cv::Mat muv = (cv::Mat_<double>(3, 1) << uv.x, uv.y, 1);

  cv::Mat ls = transform.rotation.inv() * cam_mat.inv() * muv;
  cv::Mat rs = transform.rotation.inv() * transform.tvec;

  double s = rs.at<double>(2, 0) / ls.at<double>(2, 0);
  cv::Mat temp =
      transform.rotation.inv() * (s * cam_mat.inv() * muv - transform.tvec);
  out = cv::Point3d(temp.at<double>(0), temp.at<double>(1), temp.at<double>(2));
}
} // namespace homo
} // namespace xn