//
// Created by robolab on 1.7.2021.
//

#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/formats/rect.pb.h"
#include "mediapipe/framework/formats/classification.pb.h"


namespace  mediapipe {

    namespace {

        constexpr char landmarksTag[] = "LANDMARKS";
        constexpr char handednessTag[] = "HANDEDNESS";
    }

    class InteractionModelCalculator : public CalculatorBase {

    public:
        static ::mediapipe::Status GetContract(CalculatorContract *cc);

        ::mediapipe::Status Open(CalculatorContext *cc) override;

        ::mediapipe::Status Process(CalculatorContext *cc) override;

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

        RET_CHECK(cc->Inputs().HasTag(landmarksTag));
        cc->Inputs().Tag(landmarksTag).Set< std::vector<mediapipe::NormalizedLandmarkList, std::allocator<mediapipe::NormalizedLandmarkList>>>();

        RET_CHECK(cc->Inputs().HasTag(handednessTag));
        cc->Inputs().Tag(handednessTag).Set<std::vector<mediapipe::ClassificationList>>();



        return ::mediapipe::OkStatus();
    }

    ::mediapipe::Status InteractionModelCalculator::Open(CalculatorContext *cc){
        // Sets this packet timestamp offset for Packets going to all outputs.
        cc->SetOffset(TimestampDiff(0));
        return ::mediapipe::OkStatus();
    }

    ::mediapipe::Status InteractionModelCalculator::Process(CalculatorContext *cc){

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

        if (!cc->Inputs().Tag(landmarksTag).IsEmpty()) {
            const auto& collection =
                    cc->Inputs().Tag(landmarksTag).Get<std::vector<mediapipe::NormalizedLandmarkList, std::allocator<mediapipe::NormalizedLandmarkList>>>();

            // this works, size of collection is 2 if there are 2 hands
            for (auto& c :collection)
                LOG(INFO) << c.landmark(9).x();

//            LOG(INFO) << collection;
//            for (const auto& item : collection) {
//                LOG(INFO) << item;
//                cc->Outputs().Tag("ITEM").AddPacket(
//                        MakePacket<ItemT>(item).At(loop_internal_timestamp_));
//                ForwardClonePackets(cc, loop_internal_timestamp_);
//                ++loop_internal_timestamp_;
//            }
        }

        std::vector<std::string> labels;
        std::vector<float> scores;

        if (!cc->Inputs().Tag(handednessTag).IsEmpty()) {
            const auto& classifications =
            cc->Inputs().Tag(handednessTag).Get<std::vector<mediapipe::ClassificationList>>();

            LOG(INFO) << "Classification size: " << classifications.size();

            for (auto& cl: classifications) {
//                for (int i = 0; i < cl.classification_size(); ++i) {
//
//                labels[i] = cl.classification(i).label();
//                scores[i] = cl.classification(i).score();
//
//                LOG(INFO) << "Label: " << labels[i] << " Score " << scores[i];
//            }
                for (int i = 0; i < cl.classification_size(); ++i) {
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

}