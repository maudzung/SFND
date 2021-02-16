# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1

```
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 4.1.0
cd .. #get out of opencv folder
```

```
https://github.com/opencv/opencv_contrib/
cd opencv_contrib
git checkout 4.1.0
cd .. # get out of opencv_contrib folder
```

```
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D OPENCV_ENABLE_NONFREE=ON -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -D WITH_GTK=ON ..
make
sudo make install
```

* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Execute `./run.sh`



| Detector |Descriptor | binHogDescriptorType | matcherType | selectorType | # Keypoints |# Matches |Time (ms) |# Matches/ms) |
|:---:|:----:|:----:|:----:|:----:|:----:|:-----:|:-----:|:-----:|
| SHITOMASI |BRISK |DES_BINARY |MAT_BF |SEL_KNN |13423 |2255 |178.648 |12.6226 |
| SHITOMASI |BRIEF |DES_BINARY |MAT_BF |SEL_KNN |13423 |3234 |134.174 |24.103 |
| SHITOMASI |ORB |DES_BINARY |MAT_BF |SEL_KNN |13423 |2856 |130.761 |21.8413 |
| SHITOMASI |FREAK |DES_BINARY |MAT_BF |SEL_KNN |13423 |2299 |418.962 |5.48737 |
| SHITOMASI |AKAZE |DES_BINARY |MAT_BF |SEL_KNN |- |- |- |- |
| HARRIS |BRISK |DES_BINARY |MAT_BF |SEL_KNN |728 |219 |126.886 |1.72595 |
| HARRIS |BRIEF |DES_BINARY |MAT_BF |SEL_KNN |728 |257 |117.958 |2.17873 |
| HARRIS |ORB |DES_BINARY |MAT_BF |SEL_KNN |728 |252 |123.612 |2.03864 |
| HARRIS |FREAK |DES_BINARY |MAT_BF |SEL_KNN |728 |209 |395.407 |0.52857 |
| HARRIS |AKAZE |DES_BINARY |MAT_BF |SEL_KNN |- |- |- |- |
| FAST |BRISK |DES_BINARY |MAT_BF |SEL_KNN |17874 |3170 |90.134 |35.1699 |
| FAST |BRIEF |DES_BINARY |MAT_BF |SEL_KNN |17874 |4904 |35.0202 |140.034 |
| FAST |ORB |DES_BINARY |MAT_BF |SEL_KNN |17874 |4254 |26.8548 |158.407 |
| FAST |FREAK |DES_BINARY |MAT_BF |SEL_KNN |17874 |3164 |409.592 |7.72476 |
| FAST |AKAZE |DES_BINARY |MAT_BF |SEL_KNN |- |- |- |- |
| BRISK |BRISK |DES_BINARY |MAT_BF |SEL_KNN |27116 |5073 |2722.89 |1.8631 |
| BRISK |BRIEF |DES_BINARY |MAT_BF |SEL_KNN |27116 |7474 |2744.03 |2.72374 |
| BRISK |ORB |DES_BINARY |MAT_BF |SEL_KNN |27116 |5095 |2609.96 |1.95213 |
| BRISK |FREAK |DES_BINARY |MAT_BF |SEL_KNN |27116 |5008 |3036.69 |1.64916 |
| BRISK |AKAZE |DES_BINARY |MAT_BF |SEL_KNN |- |- |- |- |
| ORB |BRISK |DES_BINARY |MAT_BF |SEL_KNN |5000 |1378 |182.005 |7.57124 |
| ORB |BRIEF |DES_BINARY |MAT_BF |SEL_KNN |5000 |1403 |145.882 |9.61737 |
| ORB |ORB |DES_BINARY |MAT_BF |SEL_KNN |5000 |1466 |155.343 |9.43721 |
| ORB |FREAK |DES_BINARY |MAT_BF |SEL_KNN |5000 |627 |439.979 |1.42507 |
| ORB |AKAZE |DES_BINARY |MAT_BF |SEL_KNN |- |- |- |- |
| AKAZE |BRISK |DES_BINARY |MAT_BF |SEL_KNN |13429 |3240 |511.91 |6.32924 |
| AKAZE |BRIEF |DES_BINARY |MAT_BF |SEL_KNN |13429 |4041 |449.472 |8.99055 |
| AKAZE |ORB |DES_BINARY |MAT_BF |SEL_KNN |13429 |3340 |441.645 |7.56263 |
| AKAZE |FREAK |DES_BINARY |MAT_BF |SEL_KNN |13429 |3228 |740.799 |4.35745 |
| AKAZE |AKAZE |DES_BINARY |MAT_BF |SEL_KNN |13429 |3463 |899.114 |3.85157 |
| SIFT |BRISK |DES_BINARY |MAT_BF |SEL_KNN |13860 |2458 |695.147 |3.53594 |
| SIFT |BRIEF |DES_BINARY |MAT_BF |SEL_KNN |13860 |3243 |627.329 |5.16954 |
| SIFT |ORB |DES_BINARY |MAT_BF |SEL_KNN |- |- |- |- |
| SIFT |FREAK |DES_BINARY |MAT_BF |SEL_KNN |13860 |2428 |913.539 |2.6578 |
| SIFT |AKAZE |DES_BINARY |MAT_BF |SEL_KNN |- |- |- |- |



TOP3 detector / descriptor combinations:

| Detector |Descriptor | binHogDescriptorType | matcherType | selectorType | # Keypoints |# Matches |Time (ms) |# Matches/ms) |
|:---:|:----:|:----:|:----:|:----:|:----:|:-----:|:-----:|:-----:|
| FAST |ORB |DES_BINARY |MAT_BF |SEL_KNN |17874 |4254 |26.8548 |158.407 |
| FAST |BRIEF |DES_BINARY |MAT_BF |SEL_KNN |17874 |4904 |35.0202 |140.034 |
| FAST |BRISK |DES_BINARY |MAT_BF |SEL_KNN |17874 |3170 |90.134 |35.1699 |
