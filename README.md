# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/KITTI/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

## Write Up :

# 1. FP.1 Match 3D Objects
In this part, we try to match 3D objects in different frames using Features extractions and matching.
Completed in camFusion_student.cpp.

# 2. FP.2 Compute Lidar-based TTC
In this part we compute time-to-collision for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.

We do this by computing mean X distance between ego car and preceding vehicles in the current and previous frame, then we use the math equations shown below. Completed in camFusion_student.cpp.

# 3. FP.3 Associate Keypoint Correspondences with Bounding Boxes
In this part we Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. Completed in camFusion_student.cpp.

# 4. FP.4 Compute Camera-based TTC
In this part we Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.

in camFusuin_Student.cpp

# 5. FP.5 Performance Evaluation 1
TTC from Lidar is not always correct because of some outliers and some unstable points from preceding vehicle's front mirrors, those need to be filtered out.

Example of errors in Lidar TTC shown below, where first TTC was ~13 s, and it decreased to ~10.9 s then decreased to ~8.09 s. The error between Lidar TTC and Camera TTC decreases from 1.84s to 0.04s and then increases to 1.22s

<img src="images/KITTI/img2.png" width="779" height="414" />
<img src="images/KITTI/fig1.png" width="779" height="414" />
<img src="images/KITTI/fig3.png" width="779" height="414" />

# 6. FP.6 Performance Evaluation 2
From the previous project(https://github.com/gautam-sharma1/SensorFusion/tree/master/SFND_2D_Feature_Tracking) top 3 detector/descriptor has been seletected in terms of their performance on accuracy and speed.

Like Lidar, TTC from camera is not always correct as when get a robust clusterKptMatchesWithROI can get a stable TTC from Camera. If the result get unstable, It's probably the worse keypints matches.

It can be seen from the last image that Camera TTC is 9.31 whereas Lidar TTC 8.09. It can be observed that even a error of 1.31 seconds is enough to cause an accident.

Top 3 combinations are FAST-BRIEF, FAST-ORB, ORB-BRIEF.
