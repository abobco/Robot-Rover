#define _CRT_SECURE_NO_WARNINGS

// #include "XN_net.hpp"
#include "../xn_yolo.hpp"
#include <fstream>
#include <iostream>

#include <mutex>
#include <queue>
#include <thread>

// using namespace std;
using namespace cv;

std::mutex net_mut;

// default command line args
std::string keys =
    "{ tracking_target t | bottle | camera servos will try to keep this object "
    "in the center of the frame }"
    "{ framework f | | Optional name of an origin framework of the model. "
    "Detect it automatically if it does not set. }"
    "{ classes     | ../../object_detection_classes_yolov3.txt | Optional path "
    "to "
    "a text file with names of classes to label detected objects. }"
    "{ config      | C:/Code/opengl-es/darknet/cfg/yolov4.cfg | }"
    "{ model       | C:/Code/opengl-es/darknet/yolov4.weights | }";

template <typename T> class QueueFPS : public std::queue<T> {
public:
  QueueFPS() : counter(0) {}

  void push(const T &entry) {
    std::lock_guard<std::mutex> lock(mutex);

    std::queue<T>::push(entry);
    counter += 1;
    if (counter == 1) {
      // Start counting from a second frame (warmup).
      tm.reset();
      tm.start();
    }
  }

  T get() {
    std::lock_guard<std::mutex> lock(mutex);
    T entry = this->front();
    this->pop();
    return entry;
  }

  float getFPS() {
    tm.stop();
    double fps = counter / tm.getTimeSec();
    tm.start();
    return static_cast<float>(fps);
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex);
    while (!this->empty())
      this->pop();
  }

  unsigned int counter;

private:
  TickMeter tm;
  std::mutex mutex;
};

int main(int argc, const char **argv) {
  CommandLineParser parser(argc, argv, keys);

  xn::yolo::YoloInfo yo(Size(256, 256));
  yo.backend = 2;
  xn::yolo::TrackingInfo track(parser.get<String>("tracking_target"),
                               parser.get<String>("classes"));
  dnn::Net net;
  xn::yolo::create_dnn(yo, net, "C:/Code/opengl-es/darknet/yolov4.weights",
                       "C:/Code/opengl-es/darknet/cfg/yolov4.cfg");

  // Create a window
  static const std::string kWinName =
      "Deep learning object detection in OpenCV";
  namedWindow(kWinName, WINDOW_NORMAL);

  // Open a video file or an image file or a camera stream.
  VideoCapture cap;
  cap.open(0);
  // xnnet::VideoStream cap(8080);

  size_t asyncNumReq = 0;
  bool process = true;

  // Frames capturing thread
  QueueFPS<Mat> framesQueue;
  std::thread framesThread([&]() {
    Mat frame;
    while (process) {
      cap >> frame;
      if (!frame.empty())
        framesQueue.push(frame.clone());
      else
        break;
    }
  });

  // Frames processing thread
  QueueFPS<Mat> processedFramesQueue;
  QueueFPS<std::vector<Mat>> predictionsQueue;
  std::thread processingThread([&]() {
    std::queue<AsyncArray> futureOutputs;
    Mat blob;
    while (process) {
      // Get a next frame
      Mat frame;
      {
        if (!framesQueue.empty()) {
          frame = framesQueue.get();
          if (asyncNumReq) {
            if (futureOutputs.size() == asyncNumReq)
              frame = Mat();
          } else
            framesQueue.clear(); // Skip the rest of frames
        }
      }

      // Process the frame

      if (!frame.empty()) {

        // imshow("raw frame", frame);
        // waitKey(1);
        xn::yolo::preprocess(frame, yo, net);

        processedFramesQueue.push(frame);

        if (asyncNumReq) {
          futureOutputs.push(net.forwardAsync());
        } else {
          std::vector<Mat> outs;

          net_mut.lock();
          net.forward(outs, yo.out_names);
          net_mut.unlock();
          predictionsQueue.push(outs);
        }
      }

      while (!futureOutputs.empty() &&
             futureOutputs.front().wait_for(std::chrono::seconds(0))) {
        AsyncArray async_out = futureOutputs.front();
        futureOutputs.pop();
        Mat out;
        async_out.get(out);
        predictionsQueue.push({out});
      }
    }
  });

  // Postprocessing and rendering loop
  while (waitKey(1) != 27) {
    if (predictionsQueue.empty())
      continue;

    std::vector<Mat> outs = predictionsQueue.get();
    printf("outs_size =%d\n", outs.size());
    Mat frame = processedFramesQueue.get();

    net_mut.lock();
    xn::yolo::postprocess(frame, outs, net, yo, track);
    net_mut.unlock();

    if (predictionsQueue.counter > 1) {
      std::string label = format("Camera: %.2f FPS", framesQueue.getFPS());
      putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5,
              Scalar(0, 0, 255));

      label = format("Network: %.2f FPS", predictionsQueue.getFPS());
      putText(frame, label, Point(0, 30), FONT_HERSHEY_SIMPLEX, 0.5,
              Scalar(0, 0, 255));

      label = format("Skipped frames: %d",
                     framesQueue.counter - predictionsQueue.counter);
      putText(frame, label, Point(0, 45), FONT_HERSHEY_SIMPLEX, 0.5,
              Scalar(0, 0, 255));
    }
    imshow(kWinName, frame);
  }

  process = false;
  framesThread.join();
  processingThread.join();
  return 0;
}