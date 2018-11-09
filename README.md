# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
 
   
### Simulator.
The Term3 Simulator which contains the Path Planning Project is available for download at [this location](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).


### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization, sensor fusion data and a sparse map list of waypoints around the highway are provided. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
## [Rubric](https://review.udacity.com/#!/rubrics/1971/view) points
### Compilation
Code compiles without errors with cmake and make.

```
make clean; make
[ 50%] Building CXX object CMakeFiles/path_planning.dir/src/main.cpp.o
[100%] Linking CXX executable path_planning
[100%] Built target path_planning
```
### Valid Trajectories

#### The car is able to drive at least 4.32 miles without incident

* Following is the screenshot of a test run:


![Test run](./Result.png)


#### The car drives according to the speed limit
* The car speed could get up to a limit of 49.5 Mph set at line 435 of [main.cpp](./src/main.cpp)

```
    const double MAX_REF_SPEED       = Mph2mps(49.5); //m/s
```
* The minimum reference speed was set to 5 Mph at line 436 of [main.cpp](./src/main.cpp)
```
    const double MIN_REF_SPEED       = Mph2mps(5.0);  //m/s
```
* The unit of reference speed used in the code is in meter-per-second (mps). The conversion between miles-per-hour (Mph) to meter-per-second (mps) is implemented as following in [main.cpp](./src/main.cpp)

```
    double Mph2mps(double x)
    {
        return (x * 1.60934 / 3.6);
    }

    double mps2Mph(double x)
    {
        return (x * 3.6 / 1.60934);
    }
```

#### Max Acceleration and Jerk are not Exceeded
* For the car to not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3, the maximum acceleration/deceleration used in the code was set to 5 m/s^2 at line 434 of [main.cpp](./src/main.cpp)

```
    const double MAX_ACCELERATION    = 5.0;           //m/s2
    
```

#### Car does not have collisions.
* The car did not collide with other cars on the road in test runs. The safety distances were set at lines 437 and 438 of [main.cpp](./src/main.cpp)

```
    const double SAFE_FRONT_DISTANCE = 30.0;
    const double SAFE_REAR_DISTANCE  = 20.0;
```

#### The car stays in its lane, except for the time between changing lanes

* The car did not spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stayed inside one of the 3 lanes on the right hand side of the road.


#### The car is able to change lanes
* The car was able to smoothly change lanes when it could, such as when behind a slower moving car and an adjacent lane is clear of other traffic.


### Reflection

The implementation in [main.cpp](./src/main.cpp) has three parts.

* Part 1 (from line 494 to 540 of [main.cpp](./src/main.cpp)) processes the car's data and sensor fusion data. The sensor fusion data input are groupped into lanes based on their Frenet d coordinates. A helper function find_lane was implemented at line 206, that works out the lane number between 0 and 2 from the input value d. The code also works out the lanes of the car and of the end of the previous path.
* Part 2 (from line 542 to 669 of [main.cpp](./src/main.cpp)) makes prediction and decision on slowing down or lane changing using data from the ego car, the previous path and sensor fusion data. The main idea was taken and adapted from the [Project's Walkthrough](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/23add5c6-7004-47ad-b169-49a5d7b1c1cb/concepts/3bdfeb8c-8dd6-49a7-9d08-beff6703792d). It first predicts relative location of the car and other cars on the same lane. A function distance_check was implemented at line 270, which takes as inputs coordinates of the ego car and other cars on the current lane of the previous path's end. The function outputs closest distance in front and behind the ego car in Frenet's s-coordinate. The sign of distance is positive if the prediction is ahead of the ego car and negative if the prediction is behind the ego car. The closest predicted distance ahead is compared against a preset safe distance (set to 30.0 metres at line 437) to make decision in the next step. If the predicted distance ahead is negative, which means some car will get within the previous path, the ego car will abandon the remaining of the previous path and generate new path ahead with negative acceleration to quickly slow down (lines 558 to 569 in [main.cpp](./src/main.cpp)). If the predicted distance ahead is further than safe distance, the ego car will keep moving on the current lane and accelerating to the maximum speed. If the predicted distance ahead is less than safe distance, the code will check if it can change lane. A function lanes_to_check_for_change (implemented at line 244) works out possible lanes for change based on the car's current lane. The function distance_check is called to predict closest distance in front and behind of the ego car on possible lanes for change. A lane is considered safe for lane changing if both predicted distance ahead and behind of the ego car are further than preset safe values. The maximum cost of lane changing for a lane is set to 1.0 if the lane is either not available or not considered as safe for lane changing. If a lane is safe, its cost is calculated as ratio of sum of minimum safe distances over sum of actual distances as in following example:

```
    //Calculate cost to change to the left lane
    if( (d_front_left > SAFE_FRONT_DISTANCE) && (d_back_left < -SAFE_REAR_DISTANCE) )
    {
        cost_change_left = (SAFE_FRONT_DISTANCE + SAFE_REAR_DISTANCE) / (d_front_left - d_back_left);
    }
    
```
* If there are two lanes considered as safe for lane changing, the one with the less cost will be selected.
* Finally if there is no safe lane for lane changing, the code will take decision to slow down the car's speed.
* The code uses a preset maximum acceleration for speeding up or slowing down of the car (line 434)
* The function distance_check searches for cars within a maximum range of 100 metres around the ego car. The s-coordinate of the car, the end of the previous path and other cars are unwrapped for correct distance calculation when passing the wrapping point of maximum value for s (6945.554 metres).


* Part3 (from line 672 to 804) is for Trajectory Generation. The basic algorithm was taken and adapted from the [Project's Walkthrough](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/23add5c6-7004-47ad-b169-49a5d7b1c1cb/concepts/3bdfeb8c-8dd6-49a7-9d08-beff6703792d). The idea is to keep a good overlap between the previous path and newly generated path for smooth driving. The code uses a simple open-source [Cubic Spline Interpolation](https://kluge.in-chemnitz.de/opensource/spline/) for generation of new trajectory positions. The calculation of position spacing is modified to use [Uniform Acceleration Motion Equations](https://en.wikipedia.org/wiki/Equations_of_motion). The modification allows for smoother speeding up and slowing down. The reference speed is capped at a maximum and a minimum values (lines 435, 436). If the reference speed is capped, the acceleration is set to zero. The maximum number of trajectory positions is set to 40 (line 439 in [main.cpp](./src/main.cpp)).







