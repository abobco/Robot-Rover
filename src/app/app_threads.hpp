#include "../robo/xn_homo.hpp"
#include "../xn_robot_host_app.hpp"

#include "../robo/xn_car_virtual.hpp"
#include "../robo/xn_yolo.hpp"

#include "../util/json.hpp"
#include <mutex>
#include <queue>
#include <thread>

using json = nlohmann::json;

namespace xn {

void int_handler(int sig) {
  SIG_IGN(sig);
  AppState::get().run = 0;
  if (!AppState::get().conn_accepted)
    exit(0);
  // closesocket(AppState::get().console->sockets.esp_data);
}

void robot_io_thread(SOCKET &esp_data, float wheel_diameter,
                     float wheel_separation, unsigned motor_cpr) {
  while (AppState::get().run) {
    // car_turn(AppState::get().console->sockets.esp_data, 90);
    // time_sleep(1);
    // car_turn(AppState::get().console->sockets.esp_data, -90);
    // time_sleep(1);
    // continue;

    while (!AppState::get().should_scan) {
      time_sleep(0.1);
    }
    lidar_scan(esp_data, wheel_diameter, wheel_separation, motor_cpr,
               AppState::get().car_position, AppState::get().car_angle);
    AppState::get().should_scan = false;

    float t = 0;
    while (AppState::get().path.size() < 1) {
      t += (float)time_sleep(0.5);
      if (t >= 1.0f) {
        closesocket(esp_data);
        esp_data = accept_connection_blocking(4001);
        if (AppState::get().should_scan) {
          lidar_scan(esp_data, wheel_diameter, wheel_separation, motor_cpr,
                     AppState::get().car_position, AppState::get().car_angle);
          AppState::get().should_scan = false;
        }

        if (AppState::get().should_restart_esp) {
          RestartEspInstruction r;
          send_pack(esp_data, r);
          AppState::get().should_restart_esp = false;
        }
        t = 0;
      }
    }

    car_follow_path(esp_data, AppState::get().path, wheel_diameter,
                    wheel_separation, motor_cpr);
    AppState::get().path.clear();
    AppState::get().should_scan = true;
  }
  // return NULL;
}

// void yolo_thread(const json &jsettings) {
//   std::mutex net_mut;
//   // std::string yoloclasses = "../object_detection_classes_yolov3.txt";
//   // std::string yolomodel = "C:/Code/opengl-es/darknet/yolov4.weights";
//   // std::string yoloconfig = "C:/Code/opengl-es/darknet/cfg/yolov4.cfg";

//   std::string yoloclasses = jsettings["yolo_classfile"];
//   std::string yolomodel = jsettings["yolo_model"];
//   std::string yoloconfig = jsettings["yolo_config"];

//   yolo::YoloInfo yo(cv::Size(256, 256));
//   yolo::TrackingInfo track(jsettings["cam_target"], yoloclasses);
//   cv::dnn::Net net = cv::dnn::Net();
//   yolo::create_dnn(yo, net, yolomodel, yoloconfig);

//   homo::HomoTransform trans;
//   cv::Size pattern_size = cv::Size(4, 3);
//   float square_size = 21;
//   std::vector<cv::Point2f> corners;
//   cv::Mat H;
//   cv::Point2i clickpt(0, 0);
//   cv::Point3d worldpt;

//   cv::FileStorage fs(cv::samples::findFile(jsettings["cam_intrinsics"]),
//                      cv::FileStorage::READ);
//   cv::Mat cameraMatrix, distCoeffs;
//   fs["camera_matrix"] >> cameraMatrix;
//   fs["distortion_coefficients"] >> distCoeffs;

//   cv::Mat frame, outframe, raw_data;
//   bool found = false;
//   while (run) {
//     if (pic_buffer.size() == 0) {
//       time_sleep(0.5);
//       continue;
//     }

//     frame_mut.lock();
//     cv::Mat rawData(1, pic_buffer.size(), CV_8SC1, (void
//     *)pic_buffer.data()); frame = cv::imdecode(rawData.clone(),
//     cv::IMREAD_UNCHANGED); frame_mut.unlock(); if (frame.empty())
//       continue;

//     // rotate 180 degrees
//     // cv::Point2f frame_cen(frame.cols * 0.5f, frame.rows * 0.5f);
//     // cv::Mat rot_mat = getRotationMatrix2D(frame_cen, 180, 1.0);
//     // cv::warpAffine(frame, frame, rot_mat, frame.size());

//     outframe = frame.clone();

//     std::vector<cv::Point2f> cornersTemp;
//     if (findChessboardCorners(frame, pattern_size, cornersTemp)) {
//       found = true;
//       corners = cornersTemp;
//       // corner points in the object frame
//       std::vector<cv::Point3f> objectPoints;
//       for (int i = 0; i < pattern_size.height; i++)
//         for (int j = 0; j < pattern_size.width; j++)
//           objectPoints.push_back(
//               cv::Point3f(float(j * square_size), float(i * square_size),
//               0));

//       std::vector<cv::Point2f> objectPointsPlanar;
//       for (size_t i = 0; i < objectPoints.size(); i++)
//         objectPointsPlanar.push_back(
//             cv::Point2f(objectPoints[i].x, objectPoints[i].y));

//       std::vector<cv::Point2f> imagePoints;
//       undistortPoints(corners, imagePoints, cameraMatrix, distCoeffs);

//       // estimate homography matrix
//       H = findHomography(objectPointsPlanar, imagePoints);

//       // estimate camera pose
//       homo::GetHomoTransform(H, trans);
//       trans.tvec.at<double>(1) += 40;
//       // trans.tvec.at<double>(0) -= 20;
//       std::vector<cv::Point3f> axes;
//       float ax_len = square_size * 2;
//       axes.push_back(objectPoints[0]);
//       axes.push_back(objectPoints[0] + cv::Point3f(ax_len, 0, 0));
//       axes.push_back(objectPoints[0] + cv::Point3f(0, ax_len, 0));
//       axes.push_back(objectPoints[0] + cv::Point3f(0, 0, ax_len));

//       std::vector<cv::Point2f> axesProjected;
//       std::vector<cv::Point2f> cornersProjected;
//       projectPoints(objectPoints, trans.rotation, trans.tvec, cameraMatrix,
//                     distCoeffs, cornersProjected);
//       projectPoints(axes, trans.rotation, trans.tvec, cameraMatrix,
//       distCoeffs,
//                     axesProjected);

//       cv::Point2f obj_cen(0, 0);
//       for (cv::Point2f &p : cornersProjected) {
//         obj_cen += p;
//       }
//       obj_cen.x /= objectPoints.size();
//       obj_cen.y /= objectPoints.size();

//       cv::Scalar axcolors[] = {cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0),
//                                cv::Scalar(0, 0, 255)};

//       for (int i = 1; i < axesProjected.size(); i++) {
//         line(outframe, axesProjected[0], axesProjected[i], axcolors[i - 1],
//         2);
//       }

//       for (cv::Point2f &c : corners) {
//         circle(outframe, c, 5, cv::Scalar(0, 0, 255));
//       }
//     }

//     yolo::preprocess(frame, yo, net);
//     std::vector<cv::Mat> outs;
//     net.forward(outs, yo.out_names);
//     yolo::postprocess(outframe, outs, net, yo, track);
//     std::vector<double> layersTimes;
//     double freq = cv::getTickFrequency() / 1000;
//     double t = net.getPerfProfile(layersTimes) / freq;
//     std::string label = cv::format("Inference time: %.2f ms", t);
//     cv::putText(outframe, label, cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX,
//                 0.4, cv::Scalar(0, 255, 0));

//     if (track.in_frame) {
//       if (found)
//         homo::inverseProjectPoint(track.base, worldpt, cameraMatrix, trans);
//       circle(outframe, track.base, 5, cv::Scalar(0, 0, 255), 2);
//       putText(outframe,
//               cv::format("(%.3f, %.3f, %.3f)", worldpt.x, worldpt.y,
//               worldpt.z), track.base,
//               // clickpt,
//               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
//     }

//     if (AppState::get().should_send_coords) {
//       int32_t x = worldpt.x, y = worldpt.y, z = worldpt.z;
//       send_int(pi_sock_arm, x);
//       send_int(pi_sock_arm, y);
//       send_int(pi_sock_arm, z);

//       AppState::get().arm_target = {-(float)worldpt.x, 30,
//       -(float)worldpt.z};

//       // cv::Mat tv_oframe = trans.rotation * trans.tvec;
//       // tv_oframe *= -1;
//       // x = tv_oframe.at<double>(0);
//       // y = tv_oframe.at<double>(1);
//       // z = tv_oframe.at<double>(2);
//       // send_int(cap.client_sock, x);
//       // send_int(cap.client_sock, y);
//       // send_int(cap.client_sock, z);
//       AppState::get().should_send_coords = false;
//     }

//     ocv_buf.clear();
//     std::vector<int> param(2);
//     param[0] = cv::IMWRITE_JPEG_QUALITY;
//     param[1] = 95; // default(95) 0-100
//     cv::imencode(".jpg", outframe, ocv_buf, param);
//     pic_needs_update = true;

//     // waitKey(1);
//   }

//   // return NULL;
// }

// void ik_sim_thread() {
//   float arm_len = 104;
//   int mode = 0;
//   vec3 pole = {0, 0, 1};
//   vec3 bonechain[] = {vec3{0, 0, 0}, vec3{0, 1, 0}, vec3{0, 1 + 7.5f / 18,
//   0},
//                       vec3{0, 1 + 15.0f / 18, 0}};
//   float start_bone_lengths[] = {vec3::dist(bonechain[0], bonechain[1]),
//                                 vec3::dist(bonechain[1], bonechain[2])};

//   ik::IkChain arm(4, bonechain, pole);

//   std::vector<xn::pio::SmoothServo> servos[] = {
//       std::vector<xn::pio::SmoothServo>(),
//       std::vector<xn::pio::SmoothServo>(),
//       std::vector<xn::pio::SmoothServo>(),
//       std::vector<xn::pio::SmoothServo>()};

//   servos[0].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(12, 500,
//   2500),
//                                            {0, -1, 0}, bonechain[0], 1500));
//   servos[0].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(13, 500,
//   2500),
//                                            {1, 0, 0}, bonechain[0], 1570));
//   servos[1].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(26, 500,
//   2500),
//                                            {1, 0, 0}, bonechain[1], 1420));
//   servos[2].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(16, 500,
//   2500),
//                                            {1, 0, 0}, bonechain[2], 1500));

//   ik::ServoChain phys_arm(arm, servos, &run);
// armptr = &phys_arm;
//   phys_arm.reset();

//   xn::pio::SmoothServo wrist(xn::pio::ServoAngular(6, 500, 2500));
//   wrist.tid = std::thread(xn::ik::move_servo_thread, std::ref(wrist));
//   start_bone_lengths[0] =
//       vec3::dist(phys_arm.positions[0], phys_arm.positions[1]);
//   start_bone_lengths[1] =
//       vec3::dist(phys_arm.positions[1], phys_arm.positions[2]);

//   float tot_err = 0;
//   float avg_err = 0;
//   int iters = 0;
//   while (run) {
//     vec3 tar = AppState::get().arm_target; // for debugging
//     // vec3 tar{1, 1, 0};
//     phys_arm.resolve(tar);
//     float ac = 0;
//     for (int i = 0; i < phys_arm.ideal_chain.bone_count - 1; i++) {
//       for (xn::pio::SmoothServo &s : phys_arm.servos[i]) {
//         if (s.axis.y > 0)
//           continue;
//         float fac = s.target_angle - (float)M_PI_2;
//         if (i != 3)
//           fac *= -1;
//         ac += fac;
//       }
//     }
//     wrist.target_angle = ac + (float)M_PI_2;

//     tot_err +=
//         vec3::dist(phys_arm.positions[2], phys_arm.ideal_chain.positions[2]);
//     avg_err = tot_err / (++iters);
//     time_sleep(1.0 / 60);
//   }

//   DUMP(avg_err);
//   wrist.tid.join();
// }

} // namespace xn