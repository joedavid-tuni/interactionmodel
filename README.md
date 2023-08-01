# Interaction Model
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) ![Bazel](https://img.shields.io/badge/built_using-bazel-green)

This git repository contains supplementary material to the Doctoral Dissertation of Joe David, "_A Design Science Research Approach to Architecting and Developing Information Systems for Collaborative Manufacturing: A Case for Human-Robot Collaboration_". 

> **Note**: For other (dissertation) related git repositories, see the meta git repository [here](https://permanent.link/to/jd-doctoral-dissertation/meta-repository).

> **Note**: A large part of this repository is a clone of [Google's Mediapipe](https://github.com/google/mediapipe) and is only extended to suit the dissertation context.

## Pre-requisites

 + [Libfreenect2](https://github.com/OpenKinect/libfreenect2) open-source library for the Kinectv2 Sensor.
 + [Bazel](https://bazel.build/) build tool.
 + [Kinect v2](https://learn.microsoft.com/en-us/windows/apps/design/devices/kinect-for-windows) Sensor
 + a pair of hands would be handy


## Getting Started


+ Include a [libfreenect2.BUILD](/libfreenect2.BUILD) in the root directory. 

+ Configure libfreenect2 in the  [WORKSPACE](./WORKSPACE) in the root directory as an external dependency. Edit the below part to point to the directory where the `libfreenect2` files are.
```
new_local_repository(
    name = "libfreenect2",
    build_file = "libfreenect2.BUILD",
    path = "/home/joe/Desktop/freenect2",
)
```

+ The MediaPipe graph used for the implementation can be found in [mediapipe/graphs/hand_tracking](/mediapipe/graphs/hand_tracking/hand_tracking_desktop_live_gpu.pbtxt).


+ The `HandLandMarkTrackingGpu` node detects the operator’s hand. The `HandRenderer` node outputs the live feed overlayed with the hand detection (as shown in Fig. below); it serves no other function

<!-- ![Handreder Node Output](/assets/handrendereroutput.png) -->

<img src="/assets/handrendereroutput.png" width="400" height="389" style="text-align: center">

The foregoing nodes are both part of the existing implementation of MediaPipe hands and reused directly. 

+ Custom calculators for interfacing the Kinect can be found in [/mediapipe/calculators/custom/](/mediapipe/calculators/custom/).

+ The entry point to running the code is [/mediapipe/examples/desktop/demo_run_graph_main_gpu.cc](/mediapipe/examples/desktop/demo_run_graph_main_gpu.cc)



## Helpful resources



+ [Understanding Mediapipe](https://developers.google.com/mediapipe/framework) for understanding mediapipe terminologies such as calculators, graphs, etc.
+ [Bazel Tutorial](https://bazel.build/start/cpp) for building the project.

## Citation

Under the included [LICENSE](./LICENSE), if you use or extend the application developed as part of the **dissertation only**, especially in an academic context, please cite. You can click "Cite this repository" on the right sidebar to copy both `APA` and `BibTeX` formatted citation.