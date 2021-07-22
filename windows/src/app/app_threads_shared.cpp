#include "app_threads_shared.hpp"

namespace xn {

void car_sim_thread(GridGraph &navgraph, RobotController &robot) {
  CarInfo info{robot.rover.position, robot.rover.rotation, navgraph.boxSize,
               navgraph.offset,      navgraph.graph,       robot.rover.target};
  while (AppState::get().run && AppState::get().sim_rover) {
    while (navgraph.path.size() < 1 && AppState::get().run) {
      time_sleep(0.1);
    }
    if (!AppState::get().run || !AppState::get().sim_rover)
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
    int64_t pic_buflen = 0;
    read_int<int64_t>(pi_cam, pic_buflen, true, 5);
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

// void arm_wait_action_complete(ArmInfo &arm, double timeout) {
//   const float MAX_DISTANCE = 0.01f;
//   const double wait_interval = 0.1;
//   auto t1 = pio::get_time();
//   time_sleep(0.5);
//   while (pio::time_diff_seconds(t1, pio::get_time()) < timeout) {
//     bool finished = true;
//     for (auto i = 0; i < arm.joints.ideal_chain.bone_count; i++) {
//       for (pio::SmoothServo &s : arm.joints.servos[i]) {
//         if (s.servo.getWidth() != s.targetw) {
//           finished = false;
//           break;
//         }
//       }
//     }
//     if (finished)
//       break;
//     time_sleep(wait_interval);
//   }
// }

// void arm_update_thread(ArmInfo &arm) {
//   while (AppState::get().run) {
//     arm.joints.move_wrist(*arm.wrist);
//   }
// }

// void arm_update(ArmInfo &arm) {
//   arm.joints.grab_safe(arm.wristlen, arm.target);
// }

void ik_sim_thread(RobotArm &arm) {
  float tot_err = 0;
  float avg_err = 0;
  int iters = 0;
  bool first = true;
  arm.createThreads();

  while (AppState::get().run) {
    if (!AppState::get().grab_trigger) {
      time_sleep(0.1);
      continue;
    }
    AppState::get().grab_trigger = false;

    arm.joints.straighten();
    arm.waitActionComplete();
    DUMP(arm.target);

    arm.updateJoints(30 / arm.base_len);
    arm.updateJoints(30 / arm.base_len);
    time_sleep(1.0);

    arm.updateJoints(-50 / arm.base_len);
    arm.updateJoints(-50 / arm.base_len);
    time_sleep(1.0);

    arm.joints.straighten();
  }
}

static void find_chessboard_thread(cv::Mat &frame,
                                   homo::ChessBoardSolver &solver) {
  while (AppState::get().run && AppState::get().conn_accepted) {
    if (frame.empty())
      continue;

    if (solver.getTransform(frame)) {
      AppState::get().found_chessboard = true;
    }
  }
}

void yolo_thread(const json &jsettings, SOCKET &pi_arm, vec3 &arm_target,
                 Jpeg &cam_pic, std::vector<uint8_t> &cam_outframe) {
  cv::Point2i clickpt(0, 0);
  cv::Point3d worldpt;
  homo::ChessBoardSolver homographySolver;

  std::string yoloclasses = jsettings["yolo_classfile"];
  std::string yolomodel = jsettings["yolo_model"];
  std::string yoloconfig = jsettings["yolo_config"];
  yolo::TrackingInfo track(jsettings["cam_target"], yoloclasses);

  yolo::YoloInfo yo(cv::Size(256, 256));
  cv::dnn::Net net = cv::dnn::Net();
  yolo::create_dnn(yo, net, yolomodel, yoloconfig);

  cv::Mat frame, outframe, raw_data;

  cv::FileStorage fs(cv::samples::findFile(jsettings["cam_intrinsics"]),
                     cv::FileStorage::READ);
  fs["camera_matrix"] >> homographySolver.cameraMatrix;
  fs["distortion_coefficients"] >> homographySolver.distCoeffs;

  std::thread findChessboardThread(find_chessboard_thread, std::ref(frame),
                                   std::ref(homographySolver));

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

    outframe = frame.clone();

    std::vector<cv::Point2f> cornersTemp;
    if (AppState::get().found_chessboard) {
      homographySolver.drawAxes(outframe);
      homographySolver.drawCorners(outframe);
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
      if (AppState::get().found_chessboard)
        homo::inverseProjectPoint(track.base, worldpt,
                                  homographySolver.cameraMatrix,
                                  homographySolver.T);
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
    cam_outframe.clear();
    std::vector<int> param(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = 95; // default(95) 0-100
    cv::imencode(".jpg", outframe, cam_outframe, param);
    AppState::get().pic_needs_update = true;
    AppState::get().frame_mut.unlock();
  }

  // return NULL;
}

// void arm_ctl_thread(RobotArm &arm) {
//   bool first = true;
//   while (AppState::get().run) {
//     // vec3 v_target = AppState::get().arm_target;
//     // read_vec3(arm_ctl_sock, v_target);
//     if (!AppState::get().grab_trigger) {
//       time_sleep(0.1);
//       return;
//     }
//     AppState::get().grab_trigger = false;

//     if (first) {
//       // set_claw_target(claw_ctl, CLAW_OPEN);
//       armInfo.joints.straighten();
//       arm_wait_action_complete(armInfo);
//       first = false;
//     }

//     // align arm to target
//     // AppState::get().arm_target = {-v_target.x + 90, -8, -v_target.y};
//     DUMP(armInfo.target);
//     armInfo.wristlen = 30;
//     AppState::get().move_to_target = true;
//     arm_wait_action_complete(armInfo);

//     // move to target
//     armInfo.wrist->t_max = armInfo.joints.servos[0][1].t_max * 1.5f;
//     armInfo.wristlen = -50;
//     arm_wait_action_complete(armInfo);

//     // grab
//     // set_claw_target(claw_ctl, CLAW_CLOSE);

//     // move to neutral position
//     AppState::get().move_to_target = false;
//     armInfo.wrist->t_max = armInfo.joints.servos[0][1].t_max / 2;
//     armInfo.joints.straighten();
//     arm_wait_action_complete(armInfo);

//     // release
//     // set_claw_target(claw_ctl, CLAW_OPEN);
//   }
// }

void rover_ctl_thread(Rover &rover, GridGraph &navgraph) {
  std::vector<std::vector<glm::vec3>> nav_verts;
  AppState::get().sim_rover = false;

  // rover.lidar_scan(nav_verts, navgraph);
  while (AppState::get().run) {

    // while (!AppState::get().should_scan) {
    //   time_sleep(0.1);
    // }
    // rover.lidar_scan(nav_verts, navgraph);
    // AppState::get().should_scan = false;

    float t = 0;
    while (navgraph.path.size() < 1) {
      t += (float)time_sleep(0.5);
      if (t >= 1.0f) {
        closesocket(rover.esp32);
        rover.esp32 = accept_connection_blocking(5002 + 3);
        if (AppState::get().should_scan) {
          rover.lidar_scan(nav_verts, navgraph);
          AppState::get().should_scan = false;
        }

        if (AppState::get().should_restart_esp) {
          RestartEspInstruction r;
          send_pack(rover.esp32, r);
          AppState::get().should_restart_esp = false;
        }
        t = 0;
      }
    }

    printf("following path");
    rover.follow_path(nav_verts, navgraph);
    navgraph.path.clear();
    AppState::get().should_scan = true;
  }
  // return NULL;
}

} // namespace xn