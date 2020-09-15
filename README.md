# PID Controller for Self Driving Car

---

### Goals
The goals of this project were to build a PID controller and tune the PID hyperparameters by applying the "twiddle" algorithm.

[//]: # (Image References)

[image1]: ./pid1.png "twiddle left"
[image2]: ./pid2.png "twiddle right"
[image3]: ./pid3.png "steady state"

### Overall Approach

#### Steering PID Controller

The steering PID was initialized with rough values determined by running the simulation a couple of times. The "p" term targeted response to the overall offset, the "d" term targeted the damping in the overshoot, and the "i" term targeted the overall drift. The algorithm allowed the simulation to run for about 100 iterations and then started the twiddle process, tuning each of the parameters until the change between iterations was below a certain threshold. 

Images of the initial "twiddling" are shown below.

![alt text][image1]
![alt text][image2]

#### Speed PID Controller

A second PID controller was used to keep the speed at the desired target speed. The parameters for this controller --considered less critical-- were set through observation of several simulation runs and were not optimized via twiddle.

#### Speed Reduction in Turns

In addition to the speed controller, a separate routine was implemented to slow the vehicle when the steering angle became high. In this implementation, if the steering angle hit a high value, the throttle would be set at 25% of the PID output. Otherwise, the PID output would be scaled base on the steering angle. This approach ensured the car slowed down if the steering became challenging (around curves) or just erratic.

#### Retraining

In an attempt to continuously improve the performance, I had the PID steering controller re-twiddle itself periodically (every 3000 iterations). Surprisingly, this had little effect and the PID parameters kept converging to roughly the same values.

#### Steady State

The vehicle was able to make multiple loops around the simulated track, albeit with some oscillations in certain regions of the road.

![alt text][image3]

## Reflection

1. Initial conditions matter:  I naively thought that initial conditions were not terribly important--I believed the optimization routine would quickly overcome poor initial guesses. However, since twiddle is a hill-climbing algorithm it is susceptible to local minima. This appears to be the reason the retraining did not have any effect. Perhaps a greater perturbation to the parameters during retraining could help escape those local minima.

2. Twiddle was difficult to implement "live": My implementation of twiddle occurred as the car was driving. Though it seemed to work, this seems inherently dangerous, especially for a real vehicle. It seems another approach should be possible.

3. Speed is coupled to steering: This may seem obvious, but speed adjustments were critical to ensuring the controller kept the car on the road.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

### Simulator.
This project uses the (Unity-based) Udacity self-driving car simulator that can be downloaded from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/download/v1.45/term2_sim_mac.zip).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```



