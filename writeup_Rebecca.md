**MPC Project**

The goals / steps of this project are the following:

* design the MPC model with correct initial states, limitations, constraints

* tune the hyperparameter so that the car stays in the track safely

## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points

### 1. Statement of the problem
We need to navigate the vehicle based on the waypoints from GPS, through steering and throttle. The points on the track are given for every short length by the simulator. The points are fitted with a polynomial of order 3 and treated as the reference for the vehicle to follow. 
The car is a mimic of a bicycle. The two front wheels act like one wheel, and the two back wheels act like one wheel. The elements of dynamic model like slip angle, slip ratio (sliding friction), etc. are ignored.

### 2. Reference curve
The polynomial fitting of the small time span of the waypoints <x,y> is:<br>
f(x) = k<sub>0</sub> +k<sub>1</sub>x + k<sub>2</sub>x<sup>2</sup> + k<sub>3</sub>x<sup>3</sup> <br>
The tangential of any point on the curve is <br>
f'(x)=k<sub>1</sub> + 2k<sub>2</sub>x + 3k<sub>3</sub>x<sup>2</sup> <br>
The desired angle (yaw) at any point is atan(f'(x))

### 3. States: *x*,*y*,*ψ*,*v*,*cte*,*eψ*
In the kinematic model, There are 6 state components: <br>
*x* and *y*: the position values (x,y) of the car in the vehicle's coordinate<br>
*v*        : the velocity value<br> 
*ψ*        : the angle (yaw) of the car's heading from x axis, denoted as *psi* in the code<br>
*cte*= y-f(x)      : cross track error,the difference between predicted line and actual line, where f(x) is the reference line<br>
*eψ*= ψ-ψ_des  : orientation error, the difference between predicted yaw and actual yaw, where ψ_des is the tangential angle of the polynomial f evaluated at x <br>

#### Initial states 
For each start of the small time span with polynomial fitted reference and control command serials to be predicted, initial states are reset as follows in according to the car's coordinates:<br>
x0 = 0<br>
y0 = 0<br>
v0 = v<br>
psi0 = 0<br>
cte0 = polyeval(coeffs,x0)-y0<br>
epsi0 = psi0 - atan(polyFirstderiv(coeffs,x0)) = -atan(coeffs[1])

#### State update
The states are updated as below.
```
    x' = x + v*cos(ψ)* dt
    y' = y + v*sin(ψ)*dt    
    ψ' = ψ +(v/L_f)*δ*dt    
    cte'​ = cte ​+v​*sin(eψ​)*dt
    eψ' = eψ +(v/L_f)*δ*dt	
    v' = v + a*dt
```
where *L_f* is the distance from the front wheel axis to the gravity center of the vehicle<br>
*δ* is the steering angle actuator <br>
*a* is the throttle actuator <br>
### 3. Actuators (control inputs): *δ*,*a*
The car has 3 actuator contributors: the steering wheel, the throttle pedal and the brake pedal. In this project, the steering wheel accepts input of steering angle *δ*, while the throttle and brake pedal combine to one actuator *a* for simplicity, with positive signifies acceleration, and negative signifies braking.
 
*a* is taken between -1 and 1, and the steering angle between -25 and 25 degrees to avoid violent turns, which is not safe. 

The vehicle also has a latency from the commands to the vehicle's actual action. The controller gives predicted command serials that minimize the difference between the vehicle's predicted path and the polynomial fitted waypoints and feed to vehicle. The commands are updated for every specified time span.




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

