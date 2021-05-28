
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_format.pb.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/framework/tool/status_util.h"

#include "include/libfreenect2/frame_listener_impl.h"
#include "include/libfreenect2/registration.h"
#include "include/libfreenect2/packet_pipeline.h"
#include "include/libfreenect2/logger.h"
#include "include/libfreenect2/config.h"
#include "include/libfreenect2/libfreenect2.hpp"

namespace mediapipe {

    namespace {
// cv::VideoCapture set data type to unsigned char by default. Therefore, the
// image format is only related to the number of channles the cv::Mat has.
        ImageFormat::Format GetImageFormat(int num_channels) {
            ImageFormat::Format format;
            switch (num_channels) {
                case 1:
                    format = ImageFormat::GRAY8;
                    break;
                case 3:
                    format = ImageFormat::SRGB;
                    break;
                case 4:
                    format = ImageFormat::SRGBA;
                    break;
                default:
                    format = ImageFormat::UNKNOWN;
                    break;
            }
            return format;
        }
    }

    class Kinectv2 : public CalculatorBase {
    public:

        static absl::Status GetContract(CalculatorContract *cc) {
            cc->Outputs().Tag("VIDEO").Set<ImageFrame>();
            LOG(ERROR) << "Getting Contract Successfull\n";

            return absl::OkStatus();
        }

        absl::Status Open(CalculatorContext *cc) override {

            if (freenect2.enumerateDevices() == 0) {
                LOG(ERROR) << "no device connected!\n";
                return absl::InternalError("no device connected!");
            }

            // open the device by its serial number
            dev = freenect2.openDevice(serial);

            if (dev == 0) {
                LOG(ERROR) << "Failure Opening device !\n";
                return absl::InternalError("Failure Opening device !\n");
            } else {
                LOG(INFO) << "Successfully Opened Kinect\n";
                LOG(INFO) << "device serial: " << dev->getSerialNumber() << "\n";
                LOG(INFO) << "device firmware: " << dev->getFirmwareVersion() << "\n";
            }
            if (enable_rgb)
                types |= libfreenect2::Frame::Color;
            if (enable_depth)
                types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;

            libfreenect2::SyncMultiFrameListener listener(types);

            dev->setColorFrameListener(&listener);
            dev->setIrAndDepthFrameListener(&listener);

            // start the device
            if (!dev->startStreams(enable_rgb, enable_depth)) {
                LOG(ERROR) << "Cannot Receive Streams ";
                return absl::InternalError("Cannot Receive Streams");
            }

            registration = new libfreenect2::Registration(dev->getIrCameraParams(),dev->getColorCameraParams());


            return absl::OkStatus();

        }

        absl::Status Process(CalculatorContext *cc) override {




            if (!listener.waitForNewFrame(frames, 10 * 1000)) // 10 sconds
            {
                LOG(INFO) << "Timeout! \n";
                return absl::InternalError("Frame Receiving Timeout");
            }

            libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
            libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
            libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

            registration->apply(rgb, depth, &undistorted, &registered);

            cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
//      cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
//      cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
//
//      cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
//      cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
//      cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);

            if (rgbmat.empty()) {
                LOG(INFO) << "Ignore empty frames from camera.";
                return absl::OkStatus();;
            }
            cv::Mat camera_frame;
            cv::cvtColor(rgbmat, camera_frame, cv::COLOR_BGR2RGBA);

            format_ = GetImageFormat(camera_frame.channels());
            auto image_frame = absl::make_unique<ImageFrame>(format_, 1920, 1080,
                    /*alignment_boundary=*/1);

            size_t frame_timestamp_us =
                    (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
            Timestamp timestamp(frame_timestamp_us);

            // If the timestamp of the current frame is not greater than the one of the
            // previous frame, the new frame will be discarded.
            if (prev_timestamp_ < timestamp) {
                cc->Outputs().Tag("VIDEO").Add(image_frame.release(), timestamp);
                prev_timestamp_ = timestamp;
            }

            listener.release(frames);

            return absl::OkStatus();

        }

        absl::Status Close(CalculatorContext *cc) override {

            dev->stop();
            dev->close();

            return absl::OkStatus();

        }

    private:
        bool enable_rgb = true;
        bool enable_depth = false;

        int types = 0;

        // attaching Framelisteners to the device to receive images frames.

        libfreenect2::Registration *registration = nullptr;

        libfreenect2::SyncMultiFrameListener listener{0};


        ImageFormat::Format format_;


        std::string serial = freenect2.getDefaultDeviceSerialNumber();

        libfreenect2::Freenect2 freenect2;
        libfreenect2::Freenect2Device *dev = 0;
        libfreenect2::PacketPipeline *pipeline = 0;
        libfreenect2::FrameMap frames;

        libfreenect2::Frame undistorted{512, 424, 4}, registered{512, 424, 4},  depth2rgb{1920, 1080 + 2, 4};

        cv::Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;

        Timestamp prev_timestamp_ = Timestamp::Unset();
    };

    REGISTER_CALCULATOR(Kinectv2);

}