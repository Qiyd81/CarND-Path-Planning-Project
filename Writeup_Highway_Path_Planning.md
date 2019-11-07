# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


### 1. Files submitted

Project includes the following files:
* main.cpp contains all the relevant packages, helpers.h, spline.h and so on.
* helpers.h: functions "hasdata","distance", "Nextwaypoint","getFrenet", "getXY" are included.
* spline.h: a function to construct a smooth spline rajectory.
* Writeup_Highway_Path_planningt.md summarizes the results


### 2. Project code instructions (main.cpp)

The main.cpp file code instructions:
* include relevant packages and files in L1-L10
* Read/load map data to acquire highway waypoint data in L21-L52
* Init ego car in lane 1, and speed 0 in L55-L58
* Read in the main cars status data in L67-L85
* Read in the previous path data and sensor fusion data in L87-L103
* Check surrounding cars status in L107-159
       -check which lane each surrounding car is in L114-L130
       -predict surrounding cars' new location in s in L133-L139
       -check if there is some car ahead, left, or right of ego car within safe distance, if so, flag the flag parameter to be true, L143-L159
* Behavior planning according to the road environment, max_speed as 48.5, max_acc as 0.30 mph to keep safe and avoid jerk limit, in L163-L187
       -if has car ahead and no car in the left lane, change left; else no car in the right, change right; else lower down the speed
       -else if no car ahead, speed up, or keep the max speed   
* Create trajectory at L219-L323
       -create path point list, start with 2 points tangent to previous path or car's current location, L219-L257
       -create 3 more points, 30, 60, 90 meters away from car's location, L260-L270
       -shift car reference angle to 0 degrees, L272-L279
       -create spline using the 5 points, L282-L285
       -split the spline according to the speed, L287-L323

### 3. Next Steps and Future Updates
* This project assumes ego is the fastest car in the lane, it only checks whether some car ahead, but if some car behind is accelerating into it, it can't deal with.
* The safe distance is set as constant value, but it is better to make it a function according to speed.
* The accelaration rate is set constant value, but it is better to make it a function according to speed.
* The spline is created with constant length value, but it is better to make it a function according to speed 

