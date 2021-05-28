// Copyright 2019 The MediaPipe Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// An example of sending OpenCV webcam frames into a MediaPipe graph.
#include <cstdlib>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"

#include "include/libfreenect2/frame_listener_impl.h"
#include "include/libfreenect2/registration.h"
#include "include/libfreenect2/packet_pipeline.h"
#include "include/libfreenect2/logger.h"
#include "include/libfreenect2/config.h"
#include "include/libfreenect2/libfreenect2.hpp"

#include <time.h>


constexpr char kInputStream[] = "input_video";
constexpr char kOutputStream[] = "output_video";
constexpr char kWindowName[] = "MediaPipe";

int num_frames = 1;

ABSL_FLAG(std::string, calculator_graph_config_file, "",
          "Name of file containing text format CalculatorGraphConfig proto.");
ABSL_FLAG(std::string, input_video_path, "",
          "Full path of video to load. "
          "If not provided, attempt to use a webcam.");
ABSL_FLAG(std::string, output_video_path, "",
          "Full path of where to save result (.mp4 only). "
          "If not provided, show result in a window.");

absl::Status RunMPPGraph() {
  std::string calculator_graph_config_contents;
  MP_RETURN_IF_ERROR(
          mediapipe::file::GetContents(absl::GetFlag(FLAGS_calculator_graph_config_file),
      &calculator_graph_config_contents)
      );
  LOG(INFO) << "Get calculator graph config contents: "
            << calculator_graph_config_contents;

  mediapipe::CalculatorGraphConfig config =
      mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
          calculator_graph_config_contents);

  LOG(INFO) << "Initialize the calculator graph.";
  mediapipe::CalculatorGraph graph;
  MP_RETURN_IF_ERROR(graph.Initialize(config));

  LOG(INFO) << "Initialize the Kinect";
//  cv::VideoCapture capture;
//  const bool load_video = !absl::GetFlag(FLAGS_input_video_path).empty();
//  if (load_video) {
//    capture.open(absl::GetFlag(FLAGS_input_video_path));
//  } else {
//    capture.open(0);
//  }
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;

    bool enable_rgb = true;
    bool enable_depth = false;

    //pipeline = new libfreenect2::OpenGLPacketPipeline();
    std::string serial = "";

    if(freenect2.enumerateDevices() == 0)
    {
        LOG(ERROR) << "no device connected!\n";
    }

    if (serial == "")
    {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }

    pipeline = new libfreenect2::OpenGLPacketPipeline();

    dev = freenect2.openDevice(serial, pipeline);

    if(dev == 0)
    {
        LOG(ERROR) << "Failure Opening device !\n";
    }

    // else {
    //    LOG(INFO) << "Successfully Opened Kinect\n";
    //    LOG(INFO) << "device serial: " << dev->getSerialNumber() << "\n";
    //    LOG(INFO) << "device firmware: " << dev->getFirmwareVersion() << "\n";
    // }

    /// [listeners]
    int types = 0;
    if (enable_rgb)
        types |= libfreenect2::Frame::Color;
    if (enable_depth)
        types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    libfreenect2::SyncMultiFrameListener listener(types);
    libfreenect2::FrameMap frames;

  
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    // start the device
    if (!dev->startStreams(enable_rgb, enable_depth)) {
        LOG(ERROR) << "Cannot Receive Streams ";
    }
    // else{
    //   cv::waitKey(10);
    // }

    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4),  depth2rgb(1920, 1080 + 2, 4);
    cv::Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;

    LOG(INFO) << "Start running the calculator graph.";
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller,
                   graph.AddOutputStreamPoller(kOutputStream));
  MP_RETURN_IF_ERROR(graph.StartRun({}));

  LOG(INFO) << "Start grabbing and processing frames.";
  bool grab_frames = true;

  while (grab_frames) {
    // Capture Kinect Frames

    clock_t start = clock();
    

     if (!listener.waitForNewFrame(frames, 10*1000)) // 10 seconds
     {
         LOG(ERROR) << "timeout!" << "\n";

     }
  
      libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
      // libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
      // libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];


      cv::Mat{rgb->height, rgb->width, CV_8UC4, rgb->data}.copyTo(rgbmat);
//      cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
//      cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
//
//      cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
//      cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
//      cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);


      if (rgbmat.empty()) {

          LOG(INFO) << "Ignore empty frames from camera.";
          continue;
      }
      else{
        LOG(INFO) << "Obtained Kinect frame";

      }

    cv::Mat camera_frame;
    cv::cvtColor(rgbmat, camera_frame, cv::COLOR_BGR2RGB);

    
    // Wrap Mat into an ImageFrame.
    auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
        mediapipe::ImageFormat::SRGB, camera_frame.cols, camera_frame.rows,
        mediapipe::ImageFrame::kDefaultAlignmentBoundary);
    cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
    camera_frame.copyTo(input_frame_mat);

    // Send image packet into the graph.
    size_t frame_timestamp_us =
        (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
    MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(
        kInputStream, mediapipe::Adopt(input_frame.release())
                          .At(mediapipe::Timestamp(frame_timestamp_us))));
    

    // Get the graph result packet, or stop if that fails.
    mediapipe::Packet packet;
    if (!poller.Next(&packet)) break;
    auto& output_frame = packet.Get<mediapipe::ImageFrame>();

    // Convert back to opencv for display or saving.
    cv::Mat output_frame_mat = mediapipe::formats::MatView(&output_frame);
    cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGB2BGR);

    clock_t end = clock();

    double seconds =  (double(end) - double(start)) / double(CLOCKS_PER_SEC);

    LOG(INFO) << "Seconds " << seconds;
    

    double fpsLive = double(num_frames) / double(seconds);
    
 
    putText(output_frame_mat, "FPS: " + std::to_string(fpsLive), {50,50}, cv::FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255),2);


      cv::imshow(kWindowName, output_frame_mat);
      // cv::waitKey(5);
      // Press any key to exit.

      // std::this_thread::sleep_for(std::chrono::milliseconds(10));

      const int pressed_key = cv::waitKey(1);
      if (pressed_key >= 0 && pressed_key != 255) grab_frames = false;

      listener.release(frames);

  }

  LOG(INFO) << "Shutting down.";
  MP_RETURN_IF_ERROR(graph.CloseInputStream(kInputStream));
  return graph.WaitUntilDone();
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  absl::ParseCommandLine(argc, argv);
  absl::Status run_status = RunMPPGraph();
  if (!run_status.ok()) {
    LOG(ERROR) << "Failed to run the graph: " << run_status.message();
    return EXIT_FAILURE;
  } else {
    LOG(INFO) << "Success!";
  }
  return EXIT_SUCCESS;
}
