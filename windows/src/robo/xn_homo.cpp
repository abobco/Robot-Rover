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

bool ChessBoardSolver::getTransform(const cv::Mat &frame) {
  std::vector<cv::Point2f> cornersTemp;
  bool found = findChessboardCorners(frame, pattern_size, cornersTemp);
  if (found) {
    corners = cornersTemp;

    for (int i = 0; i < pattern_size.height; i++)
      for (int j = 0; j < pattern_size.width; j++)
        objectPoints[i * pattern_size.width + j] =
            cv::Point3f(float(j * square_size), float(i * square_size), 0);

    for (size_t i = 0; i < objectPoints.size(); i++)
      objectPointsPlanar[i] = cv::Point2f(objectPoints[i].x, objectPoints[i].y);

    std::vector<cv::Point2f> imagePoints;
    undistortPoints(corners, imagePoints, cameraMatrix, distCoeffs);

    // estimate homography matrix
    H = findHomography(objectPointsPlanar, imagePoints);

    // estimate camera pose
    GetHomoTransform(H, T);
    T.tvec.at<double>(1) += 40;
  }
  return found;
}

void ChessBoardSolver::drawAxes(cv::Mat &frame) {
  std::vector<cv::Point3f> axes(4);
  const float ax_len = square_size * 2;
  axes[0] = objectPoints[0];
  axes[1] = objectPoints[0] + cv::Point3f(ax_len, 0, 0);
  axes[2] = objectPoints[0] + cv::Point3f(0, ax_len, 0);
  axes[3] = objectPoints[0] + cv::Point3f(0, 0, ax_len);

  std::vector<cv::Point2f> axesProjected;
  projectPoints(axes, T.rotation, T.tvec, cameraMatrix, distCoeffs,
                axesProjected);

  cv::Scalar axcolors[] = {cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0),
                           cv::Scalar(0, 0, 255)};

  for (int i = 1; i < axesProjected.size(); i++) {
    line(frame, axesProjected[0], axesProjected[i], axcolors[i - 1], 2);
  }
}

void ChessBoardSolver::drawCorners(cv::Mat &frame) {
  // std::vector<cv::Point2f> cornersProjected;
  // projectPoints(objectPoints, T.rotation, T.tvec, cameraMatrix, distCoeffs,
  //               cornersProjected);
  // cv::Point2f obj_cen(0, 0);
  // for (cv::Point2f &p : cornersProjected) {
  //   obj_cen += p;
  // }
  // obj_cen.x /= objectPoints.size();
  // obj_cen.y /= objectPoints.size();

  for (cv::Point2f &c : corners) {
    circle(frame, c, 5, cv::Scalar(0, 0, 255));
  }
}

} // namespace homo
} // namespace xn