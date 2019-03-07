# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

[//]: # (Image References)

[image1]: ./images/pid_equation.png
[image2]: ./images/pid_kp_only.png
[image3]: ./images/pid_final.png

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


## Rubrics

### The PID procedure follows what was taught in the lessons.
Yes, see the implementation in `PID.cpp`. Cross track error (CTE) is used to calculate steering angle of the car
with the equation:

![Screenshot][image1]


### Describe the effect each of the P, I, D components had in your implementation.
- P: It pulls the car when it is off track back. It is the main contribution to steering.
     However, P component is too sensitive, and always causes car oscillation and out of control.
- D: A compensation part of P component to reduce oscillation. It's magnitude is proportional to the change of CTE
- I: Contribute to the control to reduce the "drift" part which caused by imperfect control.
     It reduce the global CTE.


### Describe how the final hyperparameters were chosen.
I measure performance of hyperparameters by:
  * Complete lap (>6000 steps)
  * Accumulative CTE^2 if the first 6000 steps. The lower the better.
However, Accumulative CTE^2 for same hyperparameters set is not deterministic in simulator, and it has a pretty wide variance.
This makes fine tuning very hard since gradient descent (Twiddle) does not produce optimal result. I think this could
be caused by the unstable protocol between PID process with the simulator, which introduce random packet delays.

So I eventually judge by my eyes.

1. Manual tune Kp
   Let Kd = Ki = 0.0, and tune Kp in range [0, 1.0] with step 0.05.
   No candidate Kp can finish the lap because of severe oscillation, but I narrow down the range to [0.1, 0.2]
   by comparing number steps theat a car can stay on road.
   
   * [Video of Kp = 0.1, Kd = Ki = 0.0](https://youtu.be/43toG052o4c)
     ![Screenshot][image2]
   
   * Comparison
   
    | Kp    | # Step on road |
    |-------|--------|
    | 0.1   | 711    |
    | 0.15  | 538    |
    | 0.2   | 531    |
    
     

2. Manual tune Kd for Kp in [0.1, 0.15, 0.2]
   The goal of is step if find roughly okay Kd for each Kp. I manually try kd in range [2.0, 4.0] with step 1.0
   and validate whether it can finish the lap. If both can finish the lap, I compare the sum of CTE^2 
   in the first 6000 steps.
   
   * Kp = 0.1
   
    | Kd    | Accum CTE^2 |
    |-------|--------|
    | 2.0   | 4527.3    |
    | 3.0   | 4047.03    |
    | 4.0   | 3893.89    |

   * Kp = 0.15
   
    | Kd    | Accum CTE^2 |
    |-------|--------|
    | 2.0   | 2697.58    |
    | 3.0   | 2175.19    |
    | 4.0   | 1995.35    |
    
   * Kp = 0.2
   
    | Kd    | Accum CTE^2 |
    |-------|--------|
    | 2.0   | 2377.24    |
    | 3.0   | 1517.19    |
    | 4.0   | 1375.7    |
    
3. Manual tune Ki for candidate (Kp, Kd) pairs:

   * Kp = 0.15, Kd = 4.0
   
    | Ki    | Accum CTE^2 |
    |-------|--------|
    | 3e-4  | 1354.27 |
    | 5e-4  | 1278.32    |
    | 7e-4  | 1229.19   |
   
   * Kp = 0.2, Kd = 4.0
   
    | Ki    | Accum CTE^2 |
    |-------|--------|
    | 3e-4  | 1023.69 |
    | 5e-4  | 1014.25    |
    | 7e-4  | 1016.39   |

4. Choose Kp = 0.2, Kd = 4.0, Ki = 5e-4 for lowest accumulative CTE^2, justify by eyes that car stay on the lap.
   However, Kp = 0.2 causes sharp turns and the car still oscillate, compared to Kp=0.1, which is much mild (but large CTE error).
   

 * [Video of Kp = 0.2, Kd = 4.0, Ki = 5e-4](https://youtu.be/vkTzThCBTto)
   
   ![Screenshot][image3]

