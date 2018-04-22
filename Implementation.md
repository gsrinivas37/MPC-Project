# MPC Project Implementation Details

The below section describes details of the MPC Project implementation covering the rubric points.

---

## MODEL

The state contains five variables

      px   --> Vehicle x-coordinate
      py   --> Vehicle y-coordinate
      v    --> Vehicle velocity
      psi  --> Vechicle direction
      cte  --> Cross-track-error. The Distance between the car and trajectory
      epsi --> Orientation error. Desired orientation subtracted from the current orientation

Actuators has two values
  
      delta  --> Steering angle.  Delta is positive we rotate counter-clockwise, or turn left
      acceleration --> Throttle. Either positive or negative.

Update equations:

Below are update equations for state variable from time step t-1 to t.

      x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
      y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
      psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
      v[t] = v[t-1] + a[t-1] * dt
      cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
      epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt


## Main.cpp

1. First we get the waypoints, vehicle co-ordinates, velocity, direction, steering angle and throttle from the simulator.
2. All the co-ordinates we get from the simulator are in global co-ordinate systems.
3. I transformed the waypoints from global coordinate systems to vehicle coordinate systems using below transformations.
    
        double shift_x = ptsx[i]-px;
        double shift_y = ptsy[i]-py;
        ptsx[i] = (shift_x*cos(0-psi)-shift_y*sin(0-psi));
        ptsy[i] = (shift_x*sin(0-psi)+shift_y*cos(0-psi));
     
4. Using these transformed waypoints, we fit them into a 3rd degree polynomial.
5. Once we have the polynomial, then we calculate cross-track-error and error epsilon using below equations. Since we are using transformed points from vehicle co-ordinate systems, the vehicle is at 0,0 with direction of 0 angle.
        
        double cte  = polyeval(coeffs,0);
        double epsi = -atan(coeffs[1]);

6. I then run the update equations on state variables for 100ms time duration which takes care of actuators latency. Since we are staring at origin for vehicle position, the update equations look as below.

        double dt = 0.1;
        double px_post = v*dt;
        double py_post = 0;
        double psi_post = -v*steer_value*dt/Lf;
        double v_post = v + throttle_value*dt;
        double cte_post = cte+v*sin(epsi)*dt;
        double epsi_post = epsi + psi_post;

7. I then pass these updated state values and polynomial co-efficients to Model Predictive Control class to get the actuator values and the loop continues.


## MPC.cpp

1. The main objective of the MPC.cpp is to get the next set of actuator values by optimizing the cost function.
2. The cost function has mainly three components.
        
        a. cost based on the reference state  --> Minimise the difference between actual car state, orientation and trajectory state, orientation.
        b. Minimize the use of actuators --> So that we don't get too abrupt steering and throttle which makes vehicle move smooth.
        c. Minimize the value gap between sequential actuations --> Ensures smooth steering and acceleration/deceleration.
        
3. All the above cost components have different weights which needs to be tuned to drive the car smoothly on the track.
4. There are two main components in MPC.cpp. (MPC::Solve and FG_eval class)

## MPC::Solve

1. state is the initial state [x,y,psi,v,cte,epsi], coeffs are the coefficients of the fitting polynomial. The bulk of this method is setting up the vehicle model constraints (constraints) and variables (vars) for Ipopt.
2. Ipopt expects all the constraints and variables as vectors so we need to put all of them into a single vector. The size of it depends on N and dt we chosen.

## FG_eval class

1. The FG_eval class has only one method void operator()(ADvector& fg, const ADvector& vars).
2. vars is the vector of variables from MPC::Solve and fg is the vector of constraints.
3. An FG_eval object is created in MPC::Solve and then used by Ipopt to find the lowest cost trajectory


## Timestep Length and Elapsed Duration (N & dt)

1. N & dt determines prediction horizon is the duration over which future predictions are made.
2. Ideally we should predict for a few seconds, at most. Beyond that, the environment will change enough that it won't make sense to predict any further into the future.
3. High value of N and low value dt will increase the computation cost.
4. Based on above considerations, I have tried few combinations of N, dt and found that 10 and 0.1 are optimal for the solution.
5. Different values I tried are 
      
        20 and 0.05 -->  Doesn’t work.. Fluctuates a lot and goes off road.
        10 and 0.05 -->  Fluctuates a lot.
        10 and 0.1  -->  Works smooth.
        15 and 0.1  -->  Also works fine.


## Tuning weight parameter for cost functions.

1. I started with equal weights for all cost components and then I noticed that the steering angles are too abrupt and the vehicles eventually went off the road.
2. I modified the code to take weight parameter from command-line to tune the parameters easily.
3. After playing with different combination of parameters, I found that giving high weightage to cte, orientation and change of steering angle will give best result.
4. It makes sense because we are penalizing heavily for car's deviation from trajectory state and sudden changes in steering values.
5. It is OK to sacrifice the speed to keep the car on track and have smooth changes in actuators.
6. The final weight parameters I chose are

        mpc.w_cte = 1000;
        mpc.w_epsi = 1000;
        mpc.w_v = 1;
        mpc.w_delta = 1;
        mpc.w_acc = 1;
        mpc.w_d_delta = 500;
        mpc.w_d_a = 1;
     
     
## Final video

Below is the link to video with my solution

https://www.youtube.com/watch?v=vrSTQQcgylA
