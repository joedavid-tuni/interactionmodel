cc_library(
    name = "Kinectv2",
    srcs = ["Kinectv2.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework:packet",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework/port:opencv_highgui",
        "//mediapipe/framework/port:opencv_imgproc",
        "//mediapipe/framework/port:opencv_video",
        "@com_google_absl//absl/flags:flag",
        "@libfreenect2//:libfreenect2"
    ],
    alwayslink = 1,
)

#
# cc_library(
#    name = "interaction_model_calculator",
#    srcs = ["interaction_model_calculator.cc"],
#    visibility = ["//visibility:public"],
#    deps = [
#        "//mediapipe/framework:calculator_framework",
#        "//mediapipe/framework/formats:landmark_cc_proto",
#        "//mediapipe/framework/port:status",
#        "//mediapipe/framework/formats:rect_cc_proto",
#        "//mediapipe/framework/port:ret_check",
#    ],
#    alwayslink = 1,
# )

cc_library(
    name = "interaction_model_calculator",
    srcs = ["interaction_model_calculatorv2.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/formats:landmark_cc_proto",
        "//mediapipe/framework/port:status",
        "//mediapipe/framework/formats:rect_cc_proto",
        "//mediapipe/framework/port:ret_check",
"//mediapipe/framework/formats:classification_cc_proto",
"//mediapipe/framework/formats:image_frame_opencv",
"//mediapipe/framework/port:opencv_highgui",
    ],
    alwayslink = 1,
)
