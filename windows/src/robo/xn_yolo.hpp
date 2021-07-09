#pragma once
#include <fstream>
#include <opencv2/opencv.hpp>

// using namespace cv;

namespace xn {
namespace yolo {

struct TrackingInfo {
  std::vector<std::string> classes;
  std::string label;
  int id;
  cv::Point2i center;
  cv::Point2i base;
  bool in_frame = false;

  TrackingInfo(std::string _label, std::string classfile);
};

struct YoloInfo {
  cv::Size inp_size;
  float scale;
  float conf_thres;
  float nms_thres;
  bool swap_rb;
  std::vector<std::string> out_names;

  int backend;
  /*
          0: automatically (by default)
          1: Halide language (http://halide-lang.org/)
          2: Intel's Deep Learning Inference Engine
     (https://software.intel.com/openvino-toolkit) 3: OpenCV implementation
  */

  int device;
  /*
          0: CPU target (by default)
          1: OpenCL
          2: OpenCL fp16 (half-float precision)
          3: VPU
  */

  YoloInfo(cv::Size _inp_size, float _conf_thres = 0.5, float _nms_thres = 0.4,
           float _scale = 0.00392f, bool _swap_rb = false, int _backend = 0,
           int _device = 0)
      : inp_size(_inp_size), scale(_scale), swap_rb(_swap_rb),
        conf_thres(_conf_thres), nms_thres(_nms_thres), backend(_backend),
        device(_device) {}
};

void create_dnn(YoloInfo &yo, cv::dnn::Net &out_net, std::string modelpath,
                std::string configpath);

void preprocess(const cv::Mat &frame, YoloInfo &yo, cv::dnn::Net &net);

void postprocess(cv::Mat &frame, const std::vector<cv::Mat> &outs,
                 cv::dnn::Net &net, YoloInfo &yo, TrackingInfo &track);

bool drawPred(int classId, float conf, int left, int top, int right, int bottom,
              cv::Mat &frame, TrackingInfo &track);

// void setConfThreshold(int pos, void*)
//{
//	confThreshold = pos * 0.01f;
//}

} // namespace yolo
} // namespace xn