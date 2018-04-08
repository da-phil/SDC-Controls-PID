# Implementation of simple PID control 

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

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Implementation

The implementation of the PID controller was straight forward, the function `void PID::UpdateError(double cte)` takes the current control error for calcuating the internal error gains and the function `double PID::TotalError()` returns the resulting control error value.

One instance of the PID class controls the steering, where the cross-tack-error (CTE) is being used as error value and the other PID instance controls the speed of the car by using the a target speed range between 10 and 40 mph, depending on the steering output, increasing speed at lower steering angles and decreasing for high steering angles, just like this: `target_speed = 30. * (1. - abs(steer_value)) + 10.`

For tuning the PID gains I started out with `0.1, 0.0005, 1.0` (P,I,D) and refined the parameters manually to `0.07, 0.002, 1.7`.
Good guidelines on how to tune PID parameters are:
- https://en.wikipedia.org/wiki/Controller_(control_theory)
- https://www.embedded.com/design/prototyping-and-development/4211211/PID-without-a-PhD

After manually tuning the PID parameters I implemented twiddle ([used in class](https://www.youtube.com/watch?v=2uQ2BSzDvXs)) in order to optimize the PID parameters in respect to the RMSE over a lap on the racing course.

Even after tuning the PID parameters carefully the car drives safe but very shacky, presumably a real passenger would throw up after only one lap already! I'm already curious on how model predictive control improves this situation ;)

