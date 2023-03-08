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
// This example requires a linux computer and a GPU with EGL support drivers.
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
#include "mediapipe/gpu/gl_calculator_helper.h"
#include "mediapipe/gpu/gpu_buffer.h"
#include "mediapipe/gpu/gpu_shared_data_internal.h"


#include "include/libfreenect2/frame_listener_impl.h"
#include "include/libfreenect2/registration.h"
#include "include/libfreenect2/packet_pipeline.h"
#include "include/libfreenect2/logger.h"
#include "include/libfreenect2/config.h"
#include "include/libfreenect2/libfreenect2.hpp"


// for memory mapping
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include "iomanip"

constexpr char kInputStream[] = "input_video";
constexpr char kInputDepthStream[] = "depth_stream";
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

//const std::string currentDateTime() {
//    time_t     now = time(0);
//    struct tm  tstruct;
//    char       buf[80];
//    tstruct = *localtime(&now);
//    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
//    // for more information about date/time format
//    strftime(buf, sizeof(buf), "%Y%m%d_%X", &tstruct);
//
//    return buf;
//}


absl::Status RunMPPGraph() {


  std::string calculator_graph_config_contents;
  MP_RETURN_IF_ERROR(mediapipe::file::GetContents(
      absl::GetFlag(FLAGS_calculator_graph_config_file),
      &calculator_graph_config_contents));
  LOG(INFO) << "Get calculator graph config contents: "
            << calculator_graph_config_contents;
  mediapipe::CalculatorGraphConfig config =
      mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
          calculator_graph_config_contents);

  LOG(INFO) << "Initialize the calculator graph.";
  mediapipe::CalculatorGraph graph;
  MP_RETURN_IF_ERROR(graph.Initialize(config));

  LOG(INFO) << "Initialize the GPU.";
  ASSIGN_OR_RETURN(auto gpu_resources, mediapipe::GpuResources::Create());
  MP_RETURN_IF_ERROR(graph.SetGpuResources(std::move(gpu_resources)));
  mediapipe::GlCalculatorHelper gpu_helper;
  gpu_helper.InitializeForTest(graph.GetGpuResources().get());

  LOG(INFO) << "Initialize the Kinect";

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;

  bool enable_rgb = true;
  bool enable_depth = true;

  std::string serial = "";

  if(freenect2.enumerateDevices() == 0)
    {
        LOG(ERROR) << "no device connected!\n";
    }

    if (serial == "")
    {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }

    pipeline =  new libfreenect2::OpenGLPacketPipeline();

    dev = freenect2.openDevice(serial, pipeline);

    if(dev == 0)
    {
        LOG(ERROR) << "Failure Opening device !\n";
    }

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

    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4),  depth2rgb(1920, 1080 + 2, 4);
    cv::Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;

  // LOG(INFO) << "Initialize the camera or load the video.";
  // cv::VideoCapture capture;
  // const bool load_video = !absl::GetFlag(FLAGS_input_video_path).empty();
  // if (load_video) {
  //   capture.open(absl::GetFlag(FLAGS_input_video_path));
  // } else {
  //   capture.open(0);
  // }
  // RET_CHECK(capture.isOpened());

//  cv::VideoCapture capture;
   cv::VideoWriter writer;
//   const bool save_video = !absl::GetFlag(FLAGS_output_video_path).empty();
   const bool save_video = true;
//   if (!save_video) {
//     cv::namedWindow(kWindowName, /*flags=WINDOW_AUTOSIZE*/ 1);
// #if (CV_MAJOR_VERSION >= 3) && (CV_MINOR_VERSION >= 2)
//     capture.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
//     capture.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
//     capture.set(cv::CAP_PROP_FPS, 30);
// #endif
//   }

  LOG(INFO) << "Start running the calculator graph.";
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller,
                   graph.AddOutputStreamPoller(kOutputStream));
  MP_RETURN_IF_ERROR(graph.StartRun({}));

  LOG(INFO) << "Start grabbing and processing frames.";
  bool grab_frames = true;

  // setting up shared memory

    const char *kinect_status_object_name = "/kinectStatus";
    char * mapped_kinect_status_memory = nullptr;
    int fd_kinectS =  shm_open(kinect_status_object_name, O_RDWR , S_IRUSR | S_IWUSR);
    ftruncate(fd_kinectS,  sizeof(char) );

    LOG(INFO) << "File Descriptor: " << fd_kinectS ;

    if (fd_kinectS == -1) {
        LOG(INFO) << "SHM OPEN FAILED.. exiting" ;
        LOG(INFO) << "File Descriptor: " << fd_kinectS ;
    }
    RET_CHECK(fd_kinectS > 0 );

    mapped_kinect_status_memory = (char *) mmap(NULL, sizeof(char), PROT_READ | PROT_WRITE, MAP_SHARED, fd_kinectS, 0);
    LOG(INFO) <<" mapped_kinect_status_memory: "<< int(*mapped_kinect_status_memory);
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

  while (grab_frames &&  *mapped_kinect_status_memory) { //latter checking if kinect can be still on
//  while (grab_frames ) {

      LOG(INFO) <<" mapped_kinect_status_memory: "<< int(*mapped_kinect_status_memory);

//    clock_t start = clock(); // uncomment if you want to display fps

    if (!listener.waitForNewFrame(frames, 10*1000)) // 10 seconds
     {
         LOG(ERROR) << "timeout!" << "\n";

     }

     libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
      // libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
       libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

     cv::Mat{rgb->height, rgb->width, CV_8UC4, rgb->data}.copyTo(rgbmat);
//      cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
      cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
//
/// [registration]
      registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
      /// [registration]
//      cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
//      cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
      cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);

      rgbd2 = rgbd2 / 4096.0f;

    if (rgbmat.empty()) {

          LOG(INFO) << "Ignore empty frames from camera.";
          continue;
      }

    // Capture opencv camera or video frame.
    // cv::Mat camera_frame_raw;
    // capture >> camera_frame_raw;
    // if (camera_frame_raw.empty()) {
    //   if (!load_video) {
    //     LOG(INFO) << "Ignore empty frames from camera.";
    //     continue;
    //   }
    //   LOG(INFO) << "Empty frame, end of video reached.";
    //   break;
    // }
    cv::Mat camera_frame;
    cv::cvtColor(rgbmat, camera_frame, cv::COLOR_BGR2RGBA);
    // if (!load_video) {
    //   cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*/ 1);
    // }

    // Wrap Mat into an ImageFrame.
    auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
        mediapipe::ImageFormat::SRGBA, camera_frame.cols, camera_frame.rows,
        mediapipe::ImageFrame::kGlDefaultAlignmentBoundary);

    cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
    camera_frame.copyTo(input_frame_mat);

//      cv::Mat camera_depth_frame;
//      cv::cvtColor(rgbd2, camera_depth_frame, cv::COLOR_BGR2RGBA);

    // Prepare and add graph input packet.
    size_t frame_timestamp_us =
        (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
    MP_RETURN_IF_ERROR(
        gpu_helper.RunInGlContext([&input_frame, &frame_timestamp_us, &graph,
                                   &gpu_helper]() -> absl::Status {
          // Convert ImageFrame to GpuBuffer.
          auto texture = gpu_helper.CreateSourceTexture(*input_frame.get());
          auto gpu_frame = texture.GetFrame<mediapipe::GpuBuffer>();
          glFlush();
          texture.Release();
          // Send GPU image packet into the graph.
          MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(
              kInputStream, mediapipe::Adopt(gpu_frame.release())
                                .At(mediapipe::Timestamp(frame_timestamp_us))));
          return absl::OkStatus();
        }));

      // Wrap Depth Mat into an ImageFrame.
      auto input_depth_frame = absl::make_unique<mediapipe::ImageFrame>(
              mediapipe::ImageFormat::VEC32F1, rgbd2.cols, rgbd2.rows,
              mediapipe::ImageFrame::kGlDefaultAlignmentBoundary);
      cv::Mat input_depth_frame_mat = mediapipe::formats::MatView(input_depth_frame.get());
      rgbd2.copyTo(input_depth_frame_mat);

      // Send  depth ImageFrame  packet into the graph.
      MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(
              kInputDepthStream, mediapipe::Adopt(input_depth_frame.release())
                      .At(mediapipe::Timestamp(frame_timestamp_us))));

    // Get the graph result packet, or stop if that fails.
    mediapipe::Packet packet;
    if (!poller.Next(&packet)) break;
    std::unique_ptr<mediapipe::ImageFrame> output_frame;


    // Convert GpuBuffer to ImageFrame.
    MP_RETURN_IF_ERROR(gpu_helper.RunInGlContext(
        [&packet, &output_frame, &gpu_helper]() -> absl::Status {
          auto& gpu_frame = packet.Get<mediapipe::GpuBuffer>();
          auto texture = gpu_helper.CreateSourceTexture(gpu_frame);
          output_frame = absl::make_unique<mediapipe::ImageFrame>(
              mediapipe::ImageFormatForGpuBufferFormat(gpu_frame.format()),
              gpu_frame.width(), gpu_frame.height(),
              mediapipe::ImageFrame::kGlDefaultAlignmentBoundary);
          gpu_helper.BindFramebuffer(texture);
          const auto info = mediapipe::GlTextureInfoForGpuBufferFormat(
              gpu_frame.format(), 0, gpu_helper.GetGlVersion());
          glReadPixels(0, 0, texture.width(), texture.height(), info.gl_format,
                       info.gl_type, output_frame->MutablePixelData());
          glFlush();
          texture.Release();
          return absl::OkStatus();
        }));

    // Convert back to opencv for display or saving.
    cv::Mat output_frame_mat = mediapipe::formats::MatView(output_frame.get());
    if (output_frame_mat.channels() == 4)
      cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGBA2BGR);
    else
      cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGB2BGR);

    // uncomment next 4 lines if you want to display FPS
//    clock_t end = clock();
//    double seconds =  (double(end) - double(start)) / double(CLOCKS_PER_SEC);
//    double fpsLive = double(num_frames) / double(seconds);
//    putText(output_frame_mat, "FPS: " + std::to_string(fpsLive), {50,50}, cv::FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255),2);

//     uncomment if you want to preview output frames
// it a   cv::imshow(kWindowName, output_frame_mat);
//    cv::waitKey(5);

     if (save_video) {
       if (!writer.isOpened()) {
         LOG(INFO) << "Prepare video writer.";
           std::ostringstream dateTimeStream;
           dateTimeStream << std::put_time(&tm, "%Y%m%d_%H%M%S");
         auto dateTimeString = dateTimeStream.str();
        auto filename = std::string("InteractionModel_")+dateTimeString+".mp4";
         writer.open(std::string(filename),
                     mediapipe::fourcc('a', 'v', 'c', '1'),  // .mp4
                     20, output_frame_mat.size());
         RET_CHECK(writer.isOpened());
       }
       writer.write(output_frame_mat);
     }
//     else {

      // Press any key to exit.
//      const int pressed_key = cv::waitKey(1);
//      if (pressed_key >= 0 && pressed_key != 255) grab_frames = false;
    // }
      listener.release(frames);
  }

  LOG(INFO) << "Shutting down.";
   if (writer.isOpened()) writer.release();
  MP_RETURN_IF_ERROR(graph.CloseInputStream(kInputStream));
  MP_RETURN_IF_ERROR(graph.CloseInputStream(kInputDepthStream));
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
