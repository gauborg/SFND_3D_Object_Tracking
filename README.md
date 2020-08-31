# SFND 3D Object Tracking

This project assumes you have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

For some theory on collision detection for autonomous vehicles, check out this [documentation.](./collision_detection.md)

<img src="images/course_code_structure.png" width="779" height="414" />

In this project, we will implement the missing parts in the schematic. To do this, you will complete four major tasks:

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
  
### Special Instructions for Plotting

I have used Matplotlib library to create matlab like plots in this code. If you want to use the library,
you have to add following lines to CMakeLists.txt file.

find_package(PythonLibs 2.7)
link_directories(${PYTHON_LIBRARY_DIRS})
add_definitions(${PYTHON_DEFINITIONS})
include_directories(${PYTHON_INCLUDE_DIRS})

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

### We can also use the following commands -
1. `./clean.sh` to clean the project.
2. `./build.sh` to build the project.
3. `./run.sh` to run the project.

We implement the following functions in our code tracking 3D objects from given data -

## Match 3D Objects
Implement the method `matchBoundingBoxes`, which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). Matches must be the ones with the highest number of keypoint correspondences.

## Compute Lidar Based TTC
Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.

## Associate Keypoint Correspondences with Bounding Boxes
Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition must be added to a vector in the respective bounding box.

## Compute Camera-based TTC
Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.

## Performance Evaluation 1
Here we find examples where the TTC estimate of the Lidar sensor does not seem plausible and offer an explanation as to why this may happen.

### Observations for implausible values

1. We observed a few cases where the LIDAR TTC estimate is erroneous or does not make practical sense. We ran the code for TTC estimate for 60 frames with stepwidth of 2. Here is the table showing the values we have for SHITOMASI + BRISK detector descriptor combination run with MAT_BF and SEL_KNN matching.

#### SHITOMASI + BRISK

| **Frame** | **TTC Lidar (seconds)** | **TTC Camera (seconds)** |
| :---: | :-------: | :-------: |
|1|  12.79	 | 13.25 |
|2|  15.74  | 19.57 |
|3|	 13.18	 | 17.9  |
|4|	 13.89	 | 21.89 |
|5|	 12.34	 | 11.91 |
|6|	 11.91	 | 10.97 |
|7|	 9.55	  | 10.93 |
|8|	 8.75	  | 13.34 |
|9|	 9.63	  | 9.23  |
|10|	7.83	  | 9.43  |
|11|	7.35	  | 7.86  |
|12|	8.17	  | 8.56  |
|13|	7.21	  | 8.21  |
|14|	9.33	  | 9.50  |
|15|	16.65  | 9.35  |
|16|	11.87  | 9.92  |
|17|	9.95	  | 9.96  |
|18|	**-83.13** |	8.8|
|19|	18.00	 | 7.98  |
|20|	13.05	 | 7.33  |
|21|	9.52	  | 6.71  |
|22|	10.32	 | 12.65 |
|23|	5.18	  | 8.18  |
|24|	12.48	 | 9.75  |
|25|	**-4.25**	 | 13.54 |
|26|	13.02	 |**-10.76** |
|27|	**-18.14**	| 23.32 |
|28|	 23.74	| 10.64 |
|29|	**-98.99**	|**-inf**|
|30|	**-378.56**|-8.65  |
|31|	10.32	 |**-inf**   |
|32|	**473.97**	|-6.37  |
|33|	**-313.89**|**-inf** |
|34|	**-30622.7**|	-8.83|
|35|	53.40	|**-inf**    |

We start noticing that around frame 25 (image frame = 25*2 = 50), the values of Lidar TTC estimate seem very erroneous. It took me some time to understand why this was happening, but on closer look at the images, it looks like both the preceding vehicle and our ego vehicle are stopped. Hence, d0 and d1 become the same. Hence, the TTC goes to infinity. We also notice a erroneous value around frame 18, which may be because of some noise.

2. I ran the tests again, this time with MAT_FLANN matcher and only on the first 30 frames with
stepwidth of 1. We see that the LIDAR TTC estimate is not consistent and seems to increase somewhat between few frames which suggests that there is still so me noise in the measurements. This noise maybe due to some erroneous measurements, the reasons for which could be any of the following -

- uneven road surface with bumps/potholes
- improper calibration or mounting of LIDAR sensors
- maybe the LIDAR and camera are not perfectly synced together
- some reflections of LIDAR from unwanted objects like road/dust particles

There can some ways in which the noise in the TTC estimate can be reduced and a higher accuracy can
be achieved -
- We can fuse the LIDAR data with radar sensor data which directly gives us range estimates
- Additional data such as the orientation of the vehicle as it passes over a bump/pothole can be included in the calculations. (The inertial measurement unit IMU sensor can be used for this purpose).
- We can fuse LIDAR data with camera images and kalman Filters for higher accuracy

