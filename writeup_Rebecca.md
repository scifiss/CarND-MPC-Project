**MPC Project**

The goals / steps of this project are the following:

* design the MPC model with correct initial states, limitations, constraints

* tune the hyperparameter so that the car stays in the track safely

## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points

### 1. statement of the problem
We need to navigate the vehicle based on the waypoints from GPS, through steering and throttle. The points on the track are given for every short length by the simulator. We will fit the points with a polynomial and treat it as the reference for the vehicle to follow. The vihicle has the following controls: steering angle (-25~25 degrees) and throttle (-1~1). The vehicle can't have sharp turns. The vehicle also has a latency from the commands to the vehicle's actual action. The controller gives predicted command serials that minimize the difference between the vehicle's predicted path and the polynomial fitted waypoints and feed to vehicle. The commands are updated for every specified time span.

### 2. kinematic model and dynamic model
The car is a mimic of a bicycle. The two front wheels act like one wheel, and the two back wheels act like one wheel. The elements of dynamic model like sliding friction are ignored.
```
    x = x + v*cos(ψ)* dt
    y = y + v sin(psi) dt
    v=v+a*dt
    ψ=ψ+(v/L_f)*δ*dt
```
### 3. initial states 
There are 6 state components: the position (x,y) of the car in the local coordinants, the yaw of the car, the velocity, the difference between predicted line and actual line, and the difference between predicted yaw and actual yaw.
The received x,y values are at global coordinates. They need to be converted to the vehicle's coordinates.

### 4. constraints
The next time state and the current time state are correlated for each state components. 

### 5. cost function (Line 68-88 of MPC.cpp)
The cost function (FG)eval) is designed so as to minimize the accumulated errors of position, yaw, and velocity (r_cte, r_epsi, and r_v).
r_cte = 500~2000, and 2000 is chosen.
r_epsi = 500~2000, and 500 is chosen.
The reason to choose them very high is they are the primary estimation of how good is the vehicle predicted to run. 

r_delta controls the cost weight of the steering angle. r_a controls that of the throttle. A high r_a makes negative throttles approaching -1, and therefore low speed much lower than the reference when the errors of cte and epsi are high. 
The cost also aims to minimize the use of actuators, and the gap between successive actuator outputs.
I tried 5,10,25, 50, 100 for each of them, and choose r_delta = 50 and r_a = 50.  A lower r_a can get the car run faster, but not so stable.

r_prev_delta = 280, and r_prev_a=100; The higher of the two weights, the smoother is the car running.

### 6. solver of the optimization
The solver is called IPOPT.

#### 7. duration of the trajectory T
This is done by choosing N and dt, so that N*dt is one to several seconds. 
N = 15, and dt = 0.12
The duration T = 1.8s, almost 2s. A larger N or higher dt makes the car bouncing back and forth on the road.

### Thinking
1. To tune the parameters like the weights, I should first calculate the dynamic ranges of each considered errors, and divide them by the ranges before going to the step of selecting relative weights.
2. For the same set of parameters, the car doesn't behave the same every time. I am not sure it is the uncertainty of the simulator, or my algorithm, especially the optimization problem.

