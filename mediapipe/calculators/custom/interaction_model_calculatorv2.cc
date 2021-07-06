//
// Created by robolab on 1.7.2021.
//

#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/formats/rect.pb.h"
#include "mediapipe/framework/formats/classification.pb.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"

// for memory mapping
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>


namespace  mediapipe {

    namespace {

        constexpr char landmarksTag[] = "LANDMARKS";
        constexpr char handednessTag[] = "HANDEDNESS";
        constexpr char imageTag[] = "IMAGE";
        const char *cursor_coords_mapped_object_name = "/cursor";
        int * mapped_cursor_memory = nullptr;
    }

    class InteractionModelCalculator : public CalculatorBase {

    public:



        static ::mediapipe::Status GetContract(CalculatorContract *cc);

        ::mediapipe::Status Open(CalculatorContext *cc) override;

        ::mediapipe::Status Process(CalculatorContext *cc) override;

        ::mediapipe::Status Close(CalculatorContext *cc) override;



    private:


        float calculate_euclidean_distance(float Ax, float Ay, float Bx, float By){

            float X, Y, dist2, dist;

            X = Ax - Bx;
            Y = Ay - By;

            dist2 = std::pow(X,2) + std::pow(Y,2);
            dist = std::sqrt(dist2);

            return dist;
        }
    };

    REGISTER_CALCULATOR(InteractionModelCalculator);

    ::mediapipe::Status InteractionModelCalculator::GetContract(CalculatorContract *cc){

        int fd_cursor =  shm_open(cursor_coords_mapped_object_name, O_RDWR | O_CREAT , S_IRUSR | S_IWUSR);
        ftruncate(fd_cursor,  sizeof(int) * 2);

        if (fd_cursor == -1) {
            LOG(INFO) << "SHM OPEN FAILED.. exiting" ;
            LOG(INFO) << "File Descriptor: " << fd_cursor ;
        }
        RET_CHECK(fd_cursor > 0 );

        mapped_cursor_memory = (int *) mmap(NULL, sizeof(int) * 2, PROT_READ | PROT_WRITE, MAP_SHARED, fd_cursor, 0);

        RET_CHECK(cc->Inputs().HasTag(landmarksTag));
        cc->Inputs().Tag(landmarksTag).Set< std::vector<mediapipe::NormalizedLandmarkList, std::allocator<mediapipe::NormalizedLandmarkList>>>();

        RET_CHECK(cc->Inputs().HasTag(handednessTag));
        cc->Inputs().Tag(handednessTag).Set<std::vector<mediapipe::ClassificationList>>();

        RET_CHECK(cc->Inputs().HasTag(imageTag));
        cc->Inputs().Tag(imageTag).Set<mediapipe::ImageFrame>();

        return ::mediapipe::OkStatus();
    }

    ::mediapipe::Status InteractionModelCalculator::Open(CalculatorContext *cc){
        // Sets this packet timestamp offset for Packets going to all outputs.
        cc->SetOffset(TimestampDiff(0));
        return ::mediapipe::OkStatus();
    }

    ::mediapipe::Status InteractionModelCalculator::Process(CalculatorContext *cc){

        LOG(INFO) << " ";



//        const auto rect = &(cc->Inputs().Tag(normRectTag).Get<NormalizedRect>());
//        float height = rect->height();
//        float width = rect->width();

        if (cc->Inputs().Tag(landmarksTag).IsEmpty()) {
            return absl::OkStatus();
        }

//        if (width < 0.01 || height < 0.01)
//        {
//            LOG(INFO) << "No Hand Detected";
//            return ::mediapipe::OkStatus();
//        }

//        const auto &landmarks = cc->Inputs().Tag(landmarksTag).Get<mediapipe::NormalizedLandmarkList>();
//        const auto &rects = cc->Inputs().Tag(normRectTag).Get<mediapipe::NormalizedRect>();

//        LOG(INFO) <<" Rect ID" << rects.rect_id();
//        LOG(INFO) <<" Rect X center" << rects.x_center();
//        LOG(INFO) <<" Rect Y center" << rects.y_center();
//        LOG(INFO) <<" Rect Height" << rects.height();
//        LOG(INFO) <<" Rect Width" << rects.width();
//        LOG(INFO) <<" Rect Rotation" << rects.rect_id();

//        RET_CHECK_GT(landmarks.landmark_size(), 0) << "Empty list of landmarks";

        std::vector<std::string> labels;
        std::vector<float> scores;

        if (!cc->Inputs().Tag(handednessTag).IsEmpty()) {
            const auto& classifications =
                    cc->Inputs().Tag(handednessTag).Get<std::vector<mediapipe::ClassificationList>>();

            LOG(INFO) << "Number of Hands Detected: " << classifications.size();

//            for (auto& cl: classifications) {
                for (int j = 0; j < classifications.size(); ++j){
                    LOG(INFO) << "Hand  "<< j + 1;
                   auto &cl = classifications[j];
//                for (int i = 0; i < cl.classification_size(); ++i) {
//
//                labels[i] = cl.classification(i).label();
//                scores[i] = cl.classification(i).score();
//
//                LOG(INFO) << "Label: " << labels[i] << " Score " << scores[i];
//            }
                for (int i = 0; i < cl.classification_size(); ++i) {
//                    LOG(INFO) << "Hand i: "<< i;
                    LOG(INFO) << "LABEL: "<< cl.classification(i).label();
                    LOG(INFO) << "SCORE: "<< cl.classification(i).score();
                }
            }
//            labels.resize(classifications.classification_size());
//            scores.resize(classifications.classification_size());
//
//            for (int i = 0; i < classifications.classification_size(); ++i) {
//                labels[i] = classifications.classification(i).label();
//                scores[i] = classifications.classification(i).score();
//
//                LOG(INFO) << "Label: " << labels[i] << " Score " << scores[i];
//            }

        }

        float normalized_x = 0.0f;
        float normalized_y = 0.0f;
        int pixel_x = 1920;
        int pixel_y = 1080;


        if (!cc->Inputs().Tag(landmarksTag).IsEmpty()) {
            const auto& collection =
                    cc->Inputs().Tag(landmarksTag).Get<std::vector<mediapipe::NormalizedLandmarkList, std::allocator<mediapipe::NormalizedLandmarkList>>>();

//            LOG(INFO) << collection.size() << " hands detected"; // another place to know number of hands
            // this works, size of collection is 2 if there are 2 hands
            for (auto& c :collection) {

                normalized_x = c.landmark(9).x();
                normalized_y = c.landmark(9).y();

                pixel_x = normalized_x * pixel_x;
                pixel_y = normalized_y * pixel_y;

                LOG(INFO) << "(norm) Landmark 9 X: " << normalized_x;
                LOG(INFO) << "(norm) Landmark 9 Y: " << normalized_y;

                LOG(INFO) << "Landmark 9 X Px: " << pixel_x;
                LOG(INFO) << "Landmark 9 Y Px: " << pixel_y;
            }
//            LOG(INFO) << collection;
//            for (const auto& item : collection) {
//                LOG(INFO) << item;
//                cc->Outputs().Tag("ITEM").AddPacket(
//                        MakePacket<ItemT>(item).At(loop_internal_timestamp_));
//                ForwardClonePackets(cc, loop_internal_timestamp_);
//                ++loop_internal_timestamp_;
//            }
        }

        // mapping pixel coordinates to shared memory

        * mapped_cursor_memory = pixel_x;
        * (mapped_cursor_memory + 1) = pixel_y;



        if (!cc->Inputs().Tag(imageTag).IsEmpty()) {
            const ImageFrame& image_frame =
                    cc->Inputs().Tag(imageTag).Value().Get<ImageFrame>();
            ImageFormat::Format format = image_frame.Format();
            cv::Mat original_mat = formats::MatView(&image_frame);

//            LOG(INFO) << "FORMAT: " << format;
//            LOG(INFO) << "Byte Depth: " <<  image_frame.ByteDepth();
//            LOG(INFO) << "Height: " << image_frame.Height();
//            LOG(INFO) << "Width: " <<  image_frame.Width() ;
//
            float landmark_9_depth_pixel = original_mat.at<float>(pixel_y, pixel_x);

            LOG(INFO) << "landmark_9_depth_pixel " << landmark_9_depth_pixel*4096.0f /10.0 << "cm";

            cv::imshow("Depth2Rgb", original_mat);
            cv::waitKey(1);

        }


//        int THUMB   = 0;
//        int INDEX   = 0;
//        int MIDDLE  = 0;
//        int RING    = 0;
//        int PINKY   = 0;
//
////        LOG(INFO) << "LANDMARK 9 Coordinates: " << landmarks.landmark(9).x();
//
//        float thumb_finger_reference_point = landmarks.landmark(2).x();
//        if (landmarks.landmark(3).x() < thumb_finger_reference_point && landmarks.landmark(4).x() < thumb_finger_reference_point)
//        {
//            THUMB = 1 << 1;
//        }
//
//        float index_finger_reference_point =  landmarks.landmark(6).y();
//        if (landmarks.landmark(7).y() < index_finger_reference_point && landmarks.landmark(8).y() < index_finger_reference_point)
//        {
//            INDEX = 1 << 2;
//        }
//
//        float middle_finger_reference_point =  landmarks.landmark(10).y();
//        if (landmarks.landmark(11).y() < middle_finger_reference_point && landmarks.landmark(12).y() < middle_finger_reference_point)
//        {
//            MIDDLE = 1 << 3;
//        }
//
//        float ring_finger_reference_point =  landmarks.landmark(14).y();
//        if (landmarks.landmark(15).y() < ring_finger_reference_point && landmarks.landmark(16).y() < ring_finger_reference_point)
//        {
//            RING = 1 << 4;
//        }
//
//        float pinky_finger_reference_point =  landmarks.landmark(18).y();
//        if (landmarks.landmark(19).y() < pinky_finger_reference_point && landmarks.landmark(20).y() < pinky_finger_reference_point)
//        {
//            PINKY = 1 << 5;
//        }
//
//        // Checking current gesture
//        int gesture = THUMB | INDEX | MIDDLE | RING | PINKY;

//        switch (gesture) {
//
//            case 32:
//                LOG(INFO) << " High FIVE";
//                break;
//
//            case 2:
//                LOG(INFO) << "Pointing";
//                break;
//
//            case 0:
//                LOG(INFO) << "Fist";
//                break;
//
//            default:
//                LOG(INFO) << "Unsupported Gesture";
//
//        }


        return ::mediapipe::OkStatus();

    }

    ::mediapipe::Status InteractionModelCalculator::Close (CalculatorContext *cc) {
        shm_unlink(cursor_coords_mapped_object_name);
        return ::mediapipe::OkStatus();
    }

}