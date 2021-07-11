#include "app_threads_shared.hpp"

namespace xn {

static ik::ServoChain *armptr;
// static AppState app_state;

ik::ServoChain *getArm() { return armptr; }
// AppState *getAppState() { return &app_state; }

void car_sim_thread(GridGraph &navgraph, RobotController &robot) {
  CarInfo info{robot.rover.position, robot.rover.rotation, navgraph.boxSize,
               navgraph.offset,      navgraph.graph,       robot.rover.target};
  while (AppState::get().run) {
    while (navgraph.path.size() < 1 && AppState::get().run) {
      time_sleep(0.1);
    }
    if (!AppState::get().run)
      return;

    AppState::get().path_mut.lock();
    car_follow_path_sim(info, navgraph.path);
    AppState::get().path_mut.unlock();
    navgraph.path.clear();
    robot.rover.position = info.pos;
  }
}

void cam_io_thread(SOCKET &pi_cam, Jpeg &cam_pic,
                   std::vector<uint8_t> &cam_outframe) {
  AppState::get().received_pic = false;
  while (true) {
    int32_t pic_buflen = 0;
    read_int<int32_t>(pi_cam, pic_buflen, true, 5);
    if (pic_buflen == 0) {
      if (AppState::get().received_pic) {

        std::cerr << "cam io thread timeout\n";
        // AppState::get().received_pic = false;
        AppState::get().conn_accepted = false;
        AppState::get().pic_needs_update = true;
        AppState::get().yolo_completed = false;
        // cam_pic_buffer.clear();
        cam_pic.size = 0;
        closesocket(pi_cam);
        return;
      }
      continue;
    }
    AppState::get().frame_mut.lock();
    // AppState::get().cam_pic_buffer.resize(pic_buflen);
    cam_pic.size = pic_buflen;
    pic_buflen =
        read_buf(pi_cam, (char *)cam_pic.buffer, (uint32_t)cam_pic.size, 5);
    if (pic_buflen < 0) {
      std::cerr << "Camera Disconnected\n";
    }

    if (!AppState::get().yolo_completed) {
      // AppState::get().cam_pic_processed_buffer =
      // AppState::get().cam_pic_buffer;
      // cam_pic_processed.size = cam_pic.size;
      cam_outframe.resize(cam_pic.size);
      memcpy(cam_outframe.data(), (void *)cam_pic.buffer, cam_pic.size);
      // cv::Mat rawData(1, (int)cam_pic.size, CV_8SC1, (void *)cam_pic.buffer);
      // cam_outframe = cv::imdecode(rawData, cv::IMREAD_UNCHANGED);
      // cv::imshow("Display window", cam_outframe);
      // int k = cv::waitKey(0); // Wait for a keystroke in the window
      AppState::get().pic_needs_update = true;
    }
    AppState::get().received_pic = true;
    AppState::get().frame_mut.unlock();
  }
}

void cam_servo_ctl_thread(const json &settings, SOCKET &pi_cam,
                          RobotController &robot) {
  int x_prev = 0, y_prev = 0;
  robot.cam_servo_width.x = settings["cam_servo_width"][0];
  robot.cam_servo_width.y = settings["cam_servo_width"][1];
  while (AppState::get().run && AppState::get().conn_accepted) {
    if (x_prev != robot.cam_servo_width.x) {
      int32_t p = 23, v = robot.cam_servo_width.x;
      send_int(pi_cam, p);
      send_int(pi_cam, v);
    }

    if (y_prev != robot.cam_servo_width.y) {
      int32_t p = 24, v = robot.cam_servo_width.y;
      send_int(pi_cam, p);
      send_int(pi_cam, v);
    }

    x_prev = robot.cam_servo_width.x;
    y_prev = robot.cam_servo_width.y;
    time_sleep(1.0 / 60);
  }
}

void esp_log_thread(SOCKET &esp_log) {
  while (AppState::get().run) {
    char c;
    int n = read_buf(esp_log, &c, 1);
    if (n < 0)
      return;
    log_esp.AddLog("%c", c);
    // std::cout << c;
  }
}

void arm_wait_action_complete(double timeout) {
  const float MAX_DISTANCE = 0.01f;
  const double wait_interval = 0.1;
  auto t1 = pio::get_time();
  time_sleep(0.5);
  while (pio::time_diff_seconds(t1, pio::get_time()) < timeout) {
    bool finished = true;
    for (auto i = 0; i < armptr->ideal_chain.bone_count; i++) {
      for (pio::SmoothServo &s : armptr->servos[i]) {
        if (s.servo.getWidth() != s.targetw) {
          finished = false;
          break;
        }
      }
    }
    if (finished)
      break;
    time_sleep(wait_interval);
  }
}

void arm_update_thread(pio::SmoothServo &wrist) {
  while (AppState::get().run) {
    armptr->move_wrist(wrist);
  }
}

void arm_update(double wristlen, vec3 &arm_target) {
  armptr->grab_safe(wristlen, arm_target);
}

void ik_sim_thread(RobotController &robot) {
  AppState::get().arm_mut.lock();
  float arm_len = 104;
  int mode = 0;
  vec3 pole = {0, 0, 1};
  vec3 bonechain[] = {vec3{0, 0, 0}, vec3{0, 1, 0}, vec3{0, 1 + 7.5f / 18, 0},
                      vec3{0, 1 + 15.0f / 18, 0}};
  float start_bone_lengths[] = {vec3::dist(bonechain[0], bonechain[1]),
                                vec3::dist(bonechain[1], bonechain[2])};

  ik::IkChain arm(4, bonechain, pole);

  std::vector<xn::pio::SmoothServo> servos[4];
  servos[0].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(12, 500, 2500),
                                           {0, -1, 0}, bonechain[0], 1500));
  servos[0].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(13, 500, 2500),
                                           {1, 0, 0}, bonechain[0], 1570));
  servos[1].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(26, 500, 2500),
                                           {1, 0, 0}, bonechain[1], 1420));
  servos[2].push_back(xn::pio::SmoothServo(xn::pio::ServoAngular(16, 500, 2500),
                                           {1, 0, 0}, bonechain[2], 1500));

  ik::ServoChain phys_arm(arm, servos, &AppState::get().run);
  armptr = &phys_arm;
  AppState::get().arm_mut.unlock();
  phys_arm.reset();

  // xn::pio::SmoothServo wrist(xn::pio::ServoAngular(6, 500, 2500));
  // wrist.tid = std::thread(xn::ik::move_servo_thread, std::ref(wrist));

  robot.armInfo.wrist = new pio::SmoothServo(pio::ServoAngular(6, 500, 2500));
  start_bone_lengths[0] =
      vec3::dist(phys_arm.positions[0], phys_arm.positions[1]);
  start_bone_lengths[1] =
      vec3::dist(phys_arm.positions[1], phys_arm.positions[2]);

  std::thread update_thread(arm_update_thread, *robot.armInfo.wrist);

  float tot_err = 0;
  float avg_err = 0;
  int iters = 0;
  bool first = true;
  while (AppState::get().run) {
    // vec3 v_target = AppState::get().arm_target;
    // read_vec3(arm_ctl_sock, v_target);
    if (!AppState::get().grab_trigger) {
      time_sleep(0.1);
      continue;
    }
    AppState::get().grab_trigger = false;

    if (first) {
      // set_claw_target(claw_ctl, CLAW_OPEN);
      armptr->stiffy();
      arm_wait_action_complete();
      first = false;
    }

    // align arm to target
    // robot.armInfo.target = {-v_target.x + 90, -8, -v_target.y};
    DUMP(robot.armInfo.target);
    robot.armInfo.wristlen = 30 / robot.armInfo.base_len;
    // AppState::get().move_to_target = true;
    arm_update(robot.armInfo.wristlen, robot.armInfo.target);
    arm_update(robot.armInfo.wristlen, robot.armInfo.target);
    time_sleep(1.0);
    // arm_wait_action_complete();

    // move to target
    robot.armInfo.wrist->t_max = armptr->servos[0][1].t_max * 1.5f;
    robot.armInfo.wristlen = -50 / robot.armInfo.base_len;
    arm_update(robot.armInfo.wristlen, robot.armInfo.target);
    arm_update(robot.armInfo.wristlen, robot.armInfo.target);
    time_sleep(1.0);
    // arm_wait_action_complete();

    // grab
    // set_claw_target(claw_ctl, CLAW_CLOSE);

    // move to neutral position
    // AppState::get().move_to_target = false;
    robot.armInfo.wrist->t_max = armptr->servos[0][1].t_max / 2;
    arm_update(robot.armInfo.wristlen, robot.armInfo.target);
    // time_sleep(2.0);
    armptr->stiffy();
    // armptr->reset();
    // armptr->ideal_chain.reset();
    // arm_wait_action_complete();

    // release
    // set_claw_target(claw_ctl, CLAW_OPEN);
  }

  // DUMP(avg_err);
  // wrist.tid.join();

  update_thread.join();
}

void yolo_thread(const json &jsettings, SOCKET &pi_arm, vec3 &arm_target,
                 Jpeg &cam_pic, std::vector<uint8_t> &cam_outframe) {
  std::string yoloclasses = jsettings["yolo_classfile"];
  std::string yolomodel = jsettings["yolo_model"];
  std::string yoloconfig = jsettings["yolo_config"];

  yolo::YoloInfo yo(cv::Size(256, 256));
  yolo::TrackingInfo track(jsettings["cam_target"], yoloclasses);
  cv::dnn::Net net = cv::dnn::Net();
  yolo::create_dnn(yo, net, yolomodel, yoloconfig);

  homo::HomoTransform trans;
  cv::Size pattern_size = cv::Size(4, 3);
  float square_size = 21;
  std::vector<cv::Point2f> corners;
  cv::Mat H;
  cv::Point2i clickpt(0, 0);
  cv::Point3d worldpt;

  cv::FileStorage fs(cv::samples::findFile(jsettings["cam_intrinsics"]),
                     cv::FileStorage::READ);
  cv::Mat cameraMatrix, distCoeffs;
  fs["camera_matrix"] >> cameraMatrix;
  fs["distortion_coefficients"] >> distCoeffs;

  cv::Mat frame, outframe, raw_data;
  bool found = false;
  while (AppState::get().run && AppState::get().conn_accepted) {
    if (cam_pic.size == 0) {
      time_sleep(0.5);
      continue;
    }

    AppState::get().frame_mut.lock();
    cv::Mat rawData(1, (int)cam_pic.size, CV_8SC1, (void *)cam_pic.buffer);
    frame = cv::imdecode(rawData.clone(), cv::IMREAD_UNCHANGED);
    AppState::get().frame_mut.unlock();
    if (frame.empty())
      continue;

    // rotate 180 degrees
    // cv::Point2f frame_cen(frame.cols * 0.5f, frame.rows * 0.5f);
    // cv::Mat rot_mat = getRotationMatrix2D(frame_cen, 180, 1.0);
    // cv::warpAffine(frame, frame, rot_mat, frame.size());

    outframe = frame.clone();

    std::vector<cv::Point2f> cornersTemp;
    if (findChessboardCorners(frame, pattern_size, cornersTemp)) {
      found = true;
      corners = cornersTemp;
      // corner points in the object frame
      std::vector<cv::Point3f> objectPoints;
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

    if (AppState::get().should_send_coords) {
      int32_t x = (int32_t)worldpt.x, y = (int32_t)worldpt.y,
              z = (int32_t)worldpt.z;
      send_int(pi_arm, x);
      send_int(pi_arm, y);
      send_int(pi_arm, z);

      arm_target = {(float)worldpt.x, -5, (float)worldpt.y};
      arm_target = arm_target / 172.0f;
      // cv::Mat tv_oframe = trans.rotation * trans.tvec;
      // tv_oframe *= -1;
      // x = tv_oframe.at<double>(0);
      // y = tv_oframe.at<double>(1);
      // z = tv_oframe.at<double>(2);
      // send_int(cap.client_sock, x);
      // send_int(cap.client_sock, y);
      // send_int(cap.client_sock, z);
      AppState::get().should_send_coords = false;
    }
    if (AppState::get().update_sim_grab_target) {
      arm_target = {(float)worldpt.x, -5, (float)worldpt.y};
      arm_target = arm_target / 172.0f;
      AppState::get().update_sim_grab_target = false;
      AppState::get().grab_trigger = true;
    }
    if (!AppState::get().conn_accepted) {
      AppState::get().yolo_completed = false;
      return;
    }

    AppState::get().frame_mut.lock();
    AppState::get().yolo_completed = true;
    // AppState::get().ocv_buf.clear();
    cam_outframe.clear();
    std::vector<int> param(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = 95; // default(95) 0-100
    cv::imencode(".jpg", outframe, cam_outframe, param);
    // cam_outframe = outframe.clone();
    AppState::get().pic_needs_update = true;
    AppState::get().frame_mut.unlock();
    // waitKey(1);
  }

  // return NULL;
}

void arm_ctl_thread(vec3 &arm_target, RobotController &robot) {
  bool first = true;
  while (AppState::get().run) {
    // vec3 v_target = AppState::get().arm_target;
    // read_vec3(arm_ctl_sock, v_target);
    if (!AppState::get().grab_trigger) {
      time_sleep(0.1);
      return;
    }
    AppState::get().grab_trigger = false;

    if (first) {
      // set_claw_target(claw_ctl, CLAW_OPEN);
      armptr->stiffy();
      arm_wait_action_complete();
      first = false;
    }

    // align arm to target
    // AppState::get().arm_target = {-v_target.x + 90, -8, -v_target.y};
    DUMP(arm_target);
    robot.armInfo.wristlen = 30;
    AppState::get().move_to_target = true;
    arm_wait_action_complete();

    // move to target
    robot.armInfo.wrist->t_max = armptr->servos[0][1].t_max * 1.5f;
    robot.armInfo.wristlen = -50;
    arm_wait_action_complete();

    // grab
    // set_claw_target(claw_ctl, CLAW_CLOSE);

    // move to neutral position
    AppState::get().move_to_target = false;
    robot.armInfo.wrist->t_max = armptr->servos[0][1].t_max / 2;
    armptr->stiffy();
    arm_wait_action_complete();

    // release
    // set_claw_target(claw_ctl, CLAW_OPEN);
  }
}

// void robot_io_thread(SOCKET &esp_data, RobotWorldInfo &robot,
//                      GridGraph &navgraph, float wheel_diameter,
//                      float wheel_separation, unsigned motor_cpr) {
//   while (AppState::get().run) {
//     // car_turn(AppState::get().console->sockets.esp_data, 90);
//     // time_sleep(1);
//     // car_turn(AppState::get().console->sockets.esp_data, -90);
//     // time_sleep(1);
//     // continue;

//     while (!AppState::get().should_scan) {
//       time_sleep(0.1);
//     }
//     lidar_scan(esp_data, wheel_diameter, wheel_separation, motor_cpr,
//                robot.position, robot.rotation);
//     AppState::get().should_scan = false;

//     float t = 0;
//     while (navgraph.path.size() < 1) {
//       t += (float)time_sleep(0.5);
//       if (t >= 1.0f) {
//         closesocket(esp_data);
//         esp_data = accept_connection_blocking(5002 + 3);
//         if (AppState::get().should_scan) {
//           lidar_scan(esp_data, wheel_diameter, wheel_separation, motor_cpr,
//                      robot.position, robot.rotation);
//           AppState::get().should_scan = false;
//         }

//         if (AppState::get().should_restart_esp) {
//           RestartEspInstruction r;
//           send_pack(esp_data, r);
//           AppState::get().should_restart_esp = false;
//         }
//         t = 0;
//       }
//     }

//     car_follow_path(esp_data, navgraph.path, wheel_diameter,
//     wheel_separation,
//                     motor_cpr);
//     navgraph.path.clear();
//     AppState::get().should_scan = true;
//   }
//   // return NULL;
// }

} // namespace xn