# ASE389_proj
This is the class project for ASE389 Decision and Control of Human-Centered Robots.

1. Trajectory for Walking Robotics (TOWR) (https://github.com/ethz-adrl/towr) library is modified to include some constraints on the robot for navigating indoor environments.
2. TOWR uses trajectory optimizer IPOPT for solving the non-linear programming problem and runs using ROS.
3. Two kinds of problems are solved in this project:

    a. Sharp left/right turns for a quadruped (HyQ):
        In this problem, the quadruped has to reach it's target (green star) from start (red circle) while navigating in the constrained region, and avoiding the           restricted region.
                 
         <img src="https://github.com/shubhamsingh91/ASE_389proj/blob/master/catkin_ws/images/proba.png" width="450"/>


    b. Obstacle avoidance using a quadruped (HyQ):
       The quadruped is forced to move through single/double obstacles placed one after the other to reach the target(green star) as shown below. The constarint is         placed only on the COM (Center of Mass, depicted by black box at the center of HyQ).
          
        <img src="https://github.com/shubhamsingh91/ASE_389proj/blob/master/catkin_ws/images/probb.png" width="450">


4. Solution for optimal trajectory for prob 3a using the new constraints:    

![Alt Text](https://github.com/shubhamsingh91/ASE_389proj/blob/master/catkin_ws/images/turns.gif)


5. Solution for optimal trajectory for prob 3b using the new constraints:    



