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
