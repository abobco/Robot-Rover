#define _CRT_SECURE_NO_WARNINGS
#define PIO_VIRTUAL
#define XN_LIT 1

#include "../XN_net.hpp"
#include "../file_io/load_settings.hpp"
#include "../xn_homo.hpp"
#include "../xn_yolo.hpp"

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <mutex>
#include <queue>
#include <thread>

using namespace xn;

struct ImageData {
  uint32_t size;
  unsigned char *buffer;

  void to_mat(cv::Mat &m) const {
    cv::Mat rawData(1, size, CV_8SC1, (void *)buffer);
    m = cv::imdecode(rawData, cv::IMREAD_UNCHANGED);
  }
};

// static scene objects
DrawNode DrawNode::scene;
std::vector<PointCloud> PointCloud::all;
int DrawNode::id_count = 0;
std::unordered_map<std::string, Shader> DrawNode::shader_map;
std::unordered_map<std::string, DrawNode *> DrawNode::node_map;

// images from picam
std::queue<ImageData> frame_queue;

// homography data
cv::Mat cameraMatrix, distCoeffs;
homo::HomoTransform trans;
std::vector<cv::Point2f> corners;
std::vector<cv::Point3f> objectPoints;
cv::Mat H;
float square_size = 21; // millimeters
bool found = false;

SOCKET sock;
std::mutex frame_mut;

void cam_io_thread(SOCKET con) {
  double timing_buf[64];
  int timing_count = 0;
  while (run) {
    auto t1 = pio::get_time();
    ImageData frame{0, NULL};
    read_int<uint32_t>(con, frame.size, false, 10);
    frame.buffer = new uchar[frame.size];
    read_buf(con, (char *)frame.buffer, frame.size, 0);
    frame_queue.push(frame);

    DUMP(frame.size);

    // double t = pio::time_diff_seconds(t1, pio::get_time());
    // std::cout << "frame rate = " << 1.0 / t << '\n';
  }
}

void yolo_thread(const json &jsettings) {
  std::mutex net_mut;
  std::string yoloclasses = jsettings["yolo_classfile"];
  std::string yolomodel = jsettings["yolo_model"];
  std::string yoloconfig = jsettings["yolo_config"];

  yolo::YoloInfo yo(cv::Size(256, 256));
  yolo::TrackingInfo track(jsettings["cam_target"], yoloclasses);
  cv::dnn::Net net = cv::dnn::Net();
  yolo::create_dnn(yo, net, yolomodel, yoloconfig);

  cv::Point2i clickpt(0, 0);
  cv::Point3d worldpt;

  cv::FileStorage fs(cv::samples::findFile(jsettings["cam_intrinsics"]),
                     cv::FileStorage::READ);
  fs["camera_matrix"] >> cameraMatrix;
  fs["distortion_coefficients"] >> distCoeffs;

  cv::Mat frame, outframe, raw_data;
  while (run) {
    while (!frame_queue.empty()) {
      ImageData f = frame_queue.front();
      frame_queue.pop();

      if (frame_queue.empty()) {
        cv::Mat rawData(1, f.size, CV_8UC1, (void *)f.buffer);
        frame = cv::imdecode(rawData, cv::IMREAD_UNCHANGED);
      }
      delete[] f.buffer;
    }
    if (frame.empty())
      continue;

    outframe = frame.clone();
    // cv::imshow("src", frame);

    if (found) {
      std::vector<cv::Point3f> axes;
      float ax_len = square_size * 2;
      axes.push_back(objectPoints[0]);
      axes.push_back(objectPoints[0] + cv::Point3f(ax_len, 0, 0));
      axes.push_back(objectPoints[0] + cv::Point3f(0, ax_len, 0));
      axes.push_back(objectPoints[0] + cv::Point3f(0, 0, ax_len));

      std::vector<cv::Point2f> axesProjected;
      std::vector<cv::Point2f> cornersProjected;
      projectPoints(objectPoints, trans.rotation, trans.tvec, cameraMatrix,
                    distCoeffs, cornersProjected);
      projectPoints(axes, trans.rotation, trans.tvec, cameraMatrix, distCoeffs,
                    axesProjected);

      cv::Point2f obj_cen(0, 0);
      for (cv::Point2f &p : cornersProjected) {
        obj_cen += p;
      }
      obj_cen.x /= objectPoints.size();
      obj_cen.y /= objectPoints.size();

      cv::Scalar axcolors[] = {cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0),
                               cv::Scalar(0, 0, 255)};

      for (int i = 1; i < axesProjected.size(); i++) {
        line(outframe, axesProjected[0], axesProjected[i], axcolors[i - 1], 2);
      }

      for (cv::Point2f &c : corners) {
        circle(outframe, c, 5, cv::Scalar(0, 0, 255));
      }
    }

    yolo::preprocess(frame, yo, net);
    std::vector<cv::Mat> outs;
    net.forward(outs, yo.out_names);
    yolo::postprocess(outframe, outs, net, yo, track);
    std::vector<double> layersTimes;
    double freq = cv::getTickFrequency() / 1000;
    double t = net.getPerfProfile(layersTimes) / freq;
    std::string label = cv::format("Inference time: %.2f ms", t);
    cv::putText(outframe, label, cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX,
                0.4, cv::Scalar(0, 255, 0));

    if (track.in_frame) {
      if (found)
        homo::inverseProjectPoint(track.base, worldpt, cameraMatrix, trans);
      circle(outframe, track.base, 5, cv::Scalar(0, 0, 255), 2);
      putText(outframe,
              cv::format("(%.3f, %.3f, %.3f)", worldpt.x, worldpt.y, worldpt.z),
              track.base,
              // clickpt,
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
    }

    cv::imshow("yolo", outframe);
    if (cv::waitKey(1) == 27)
      break;
  }
}

void homo_thread() {
  while (run) {
    std::vector<cv::Point2f> cornersTemp;
    cv::Mat frame;
    cv::Size pattern_size = cv::Size(4, 3);

    // while (frame_queue.empty()) {
    //   time_sleep(0.1);
    // }

    if (frame_queue.empty())
      continue;
    frame_queue.front().to_mat(frame);

    if (findChessboardCorners(frame, pattern_size, cornersTemp)) {
      found = true;
      corners = cornersTemp;

      // get corner points in the object frame
      objectPoints.clear();
      for (int i = 0; i < pattern_size.height; i++)
        for (int j = 0; j < pattern_size.width; j++)
          objectPoints.push_back(
              cv::Point3f(float(j * square_size), float(i * square_size), 0));

      std::vector<cv::Point2f> objectPointsPlanar;
      for (size_t i = 0; i < objectPoints.size(); i++)
        objectPointsPlanar.push_back(
            cv::Point2f(objectPoints[i].x, objectPoints[i].y));

      std::vector<cv::Point2f> imagePoints;
      undistortPoints(corners, imagePoints, cameraMatrix, distCoeffs);

      // estimate homography matrix
      H = findHomography(objectPointsPlanar, imagePoints);

      // estimate camera pose
      homo::GetHomoTransform(H, trans);
      trans.tvec.at<double>(1) += 40;
      // trans.tvec.at<double>(0) -= 20;
    } else {
      found = false;
    }
  }
}

int main(int argc, const char **argv) {

  json jset = load_settings_json("../config/xn_settings.json");
  sock = xn::accept_connection_blocking(5002);
  std::vector<std::thread> threads;
  threads.push_back(std::thread(cam_io_thread, sock));
  threads.push_back(std::thread(yolo_thread, jset));
  threads.push_back(std::thread(homo_thread));

  for (auto &t : threads)
    t.join();

  //   while (cv::waitKey(1) != 27) {
  //     uint32_t s;

  //     xn::read_int(sock, s, false);

  //     unsigned char *data = new unsigned char[s];

  //     xn::read_buf(sock, (char *)data, s, 10);
  //     cv::Mat rawData(1, s, CV_8SC1, (void *)data);
  //     cv::Mat decodedImage = cv::imdecode(rawData, cv::IMREAD_UNCHANGED);
  //     // cv::cvtColor(decodedImage, decodedImage, cv::COLOR_BGR2RGB);
  //     cv::imshow("img", decodedImage);
  //   }

  return 0;
}