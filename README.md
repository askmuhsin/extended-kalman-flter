# Extended-Kalman-Flter
Implementation of EKF with sensor fusion to track the position of a car in real time, using data streams from RADAR and LIDAR.

---

# Build Instruction
1. Clone this repo and cd into it.
2. `mkdir build && cd build`
3. `cmake ..` 
4. `make`
5. Run : `./ExtendedKF`

Once the program starts running, the following message can be seen : **Listening to port 4567**.
Note: This project requires the Udacity open source simulator : [Udacity term 2 sim](https://github.com/udacity/self-driving-car-sim/releases/tag/v1.45).

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
  
  ---
  
  ## Result
  The make file runs without any errors.
  The RMSE for dataset 1 is as follows :
  
  Input |   MSE  
  ----- | -------
   px   | 0.0974 
   py   | 0.0855 
   vx   | 0.4517 
   vy   | 0.4404 
  
  The RMSE for dataset 2 is as follows :
  
   Input |   MSE  
   ----- | -------
    px   | 0.0726 
    py   | 0.0965 
    vx   | 0.4219 
    vy   | 0.4937 
    
  ![Result visualization] (https://github.com/askmuhsin/extended-kalman-flter/blob/master/images/visualization1.gif)

  
