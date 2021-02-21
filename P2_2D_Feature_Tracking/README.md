# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The objective of the project is to build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. 

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

## How to run

1. Clone this repo.
2. Execute `./run.sh`


## Rubric of Camera Based 2D Feature Tracking project

The project consists of four main steps:

1. Data Buffer

    Loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. This can be achieved by pushing in new elements on one end and removing elements on the other end. The code of this part is from line _#75_ to line _#78_ in the file `src/MidTermProject_Camera_Student.cpp`.

2. Keypoints

    - Keypoint Detection <br>
      The implementations of detectors such as HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT are from the line _#141_ to line _#262_ in the file `src/matching2D_Student.cpp`
    - Keypoint Removal <br>
      Only keeping keypoints on the preceding vehicle by removing all keypoints outside of a pre-defined rectangle. The code for this part is from line _#113_ to line _#124_ in the file `src/MidTermProject_Camera_Student.cpp`

3. Descriptors

    - Keypoint Descriptors <br>
    Implementing descriptors such as BRIEF, ORB, FREAK, AKAZE and SIFT in `descKeypoints()` function in the file `src/matching2D_Student.cpp` (From line _#54_ to line _#95_).

    - Descriptor Matching
    This part was implemented in the function `matchDescriptors()` in the file `src/matching2D_Student.cpp`. Not only `FLANN` but also `k-nearest neighbor selection` could be easily selective in the shell script `run.sh`.

    - Descriptor Distance Ratio
    The descriptor distance ratio could be adjust in the file `src/matching2D_Student.cpp`, line _#41_.

4. Performance <br>

  The below table compares different combinations in terms of number of keypoints, number of matches and runtime. 


| Detector |Descriptor | binHogDescriptorType | matcherType | selectorType | # Keypoints |# Matches |Time (ms) |# Matches/ms) |
|:---:|:----:|:----:|:----:|:----:|:----:|:-----:|:-----:|:-----:|
| SHITOMASI |BRISK |DES_BINARY |MAT_BF |SEL_KNN |13423 |767 |136.008 |5.63939 |
| SHITOMASI |BRIEF |DES_BINARY |MAT_BF |SEL_KNN |13423 |944 |125.688 |7.51068 |
| SHITOMASI |ORB |DES_BINARY |MAT_BF |SEL_KNN |13423 |908 |121.562 |7.46942 |
| SHITOMASI |FREAK |DES_BINARY |MAT_BF |SEL_KNN |13423 |768 |415.079 |1.85025 |
| SHITOMASI |AKAZE |DES_BINARY |MAT_BF |SEL_KNN |- |- |- |- |
| HARRIS |BRISK |DES_BINARY |MAT_BF |SEL_KNN |728 |129 |115.473 |1.11715 |
| HARRIS |BRIEF |DES_BINARY |MAT_BF |SEL_KNN |728 |146 |106.616 |1.3694 |
| HARRIS |ORB |DES_BINARY |MAT_BF |SEL_KNN |728 |146 |115.647 |1.26246 |
| HARRIS |FREAK |DES_BINARY |MAT_BF |SEL_KNN |728 |128 |421.721 |0.303518 |
| HARRIS |AKAZE |DES_BINARY |MAT_BF |SEL_KNN |- |- |- |- |
| FAST |BRISK |DES_BINARY |MAT_BF |SEL_KNN |17874 |899 |23.712 |37.9133 |
| FAST |BRIEF |DES_BINARY |MAT_BF |SEL_KNN |17874 |1099 |16.6352 |66.0648 |
| FAST |ORB |DES_BINARY |MAT_BF |SEL_KNN |17874 |1071 |17.3953 |61.5685 |
| FAST |FREAK |DES_BINARY |MAT_BF |SEL_KNN |17874 |878 |333.318 |2.63412 |
| FAST |AKAZE |DES_BINARY |MAT_BF |SEL_KNN |- |- |- |- |
| BRISK |BRISK |DES_BINARY |MAT_BF |SEL_KNN |27116 |1570 |2560.76 |0.613099 |
| BRISK |BRIEF |DES_BINARY |MAT_BF |SEL_KNN |27116 |1704 |2730.16 |0.62414 |
| BRISK |ORB |DES_BINARY |MAT_BF |SEL_KNN |27116 |1514 |2697.41 |0.561278 |
| BRISK |FREAK |DES_BINARY |MAT_BF |SEL_KNN |27116 |1524 |3048.11 |0.499982 |
| BRISK |AKAZE |DES_BINARY |MAT_BF |SEL_KNN |- |- |- |- |
| ORB |BRISK |DES_BINARY |MAT_BF |SEL_KNN |5000 |751 |180.17 |4.1683 |
| ORB |BRIEF |DES_BINARY |MAT_BF |SEL_KNN |5000 |545 |149.168 |3.6536 |
| ORB |ORB |DES_BINARY |MAT_BF |SEL_KNN |5000 |763 |173.527 |4.397 |
| ORB |FREAK |DES_BINARY |MAT_BF |SEL_KNN |5000 |420 |467.239 |0.898897 |
| ORB |AKAZE |DES_BINARY |MAT_BF |SEL_KNN |- |- |- |- |
| AKAZE |BRISK |DES_BINARY |MAT_BF |SEL_KNN |13429 |1215 |500.301 |2.42854 |
| AKAZE |BRIEF |DES_BINARY |MAT_BF |SEL_KNN |13429 |1266 |477.313 |2.65234 |
| AKAZE |ORB |DES_BINARY |MAT_BF |SEL_KNN |13429 |1182 |551.95 |2.1415 |
| AKAZE |FREAK |DES_BINARY |MAT_BF |SEL_KNN |13429 |1187 |786.24 |1.50972 |
| AKAZE |AKAZE |DES_BINARY |MAT_BF |SEL_KNN |13429 |1259 |771.968 |1.6309 |
| SIFT |BRISK |DES_BINARY |MAT_BF |SEL_KNN |13860 |594 |608.835 |0.975634 |
| SIFT |BRIEF |DES_BINARY |MAT_BF |SEL_KNN |13860 |704 |624.359 |1.12756 |
| SIFT |ORB |DES_BINARY |MAT_BF |SEL_KNN |- |- |- |- |
| SIFT |FREAK |DES_BINARY |MAT_BF |SEL_KNN |13860 |595 |910.661 |0.653372 |
| SIFT |AKAZE |DES_BINARY |MAT_BF |SEL_KNN |- |- |- |- |



**TOP3 detector / descriptor combinations** <br>


  | Detector |Descriptor | binHogDescriptorType | matcherType | selectorType | # Keypoints |# Matches |Time (ms) |# Matches/ms) |
  |:---:|:----:|:----:|:----:|:----:|:----:|:-----:|:-----:|:-----:|
  | FAST |BRIEF |DES_BINARY |MAT_BF |SEL_KNN |17874 |1099 |16.6352 |66.0648 |
  | FAST |ORB |DES_BINARY |MAT_BF |SEL_KNN |17874 |1071 |17.3953 |61.5685 |
  | FAST |BRISK |DES_BINARY |MAT_BF |SEL_KNN |17874 |899 |23.712 |37.9133 |


Some result examples with the top-3 detector / descriptor combinations:

- FAST + BRIEF

<img src="images/FAST_BRIEF.png"/>

- FAST + ORB

<img src="images/FAST_ORB.png"/>

- FAST + BRISK

<img src="images/FAST_BRISK.png"/>

