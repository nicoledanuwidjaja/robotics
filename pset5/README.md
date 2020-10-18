# Project 5 - Maze Pt. 1

Nicole Danuwidjaja

Write a control program that solves simple mazes with a specific simulated robot with limited sensors in Gazebo. The maze consists of 90 degree angles and 12 horizontal hallways. The robot only provides the `range` and `pos_t` readings as sensors. The `set_vel` method controls the motion of the robot.

Strategy: Use the four cardinal directions (north, south, east, west) to create right-angle turns for the robot. Utilized wall following to indicate to sensors as to when the robot is or isn't currently in range of an obstacle. If the robot was wall following and finds an opening, it learns to turn to advance to the next row, according to the direction it is currently pointed towards. This program supports various robot turns and uses Gazebo's API library to properly time actions.

Attempted to use Depth-First Search and other pathsolving algorithms.


YouTube Video: None. The robot did not make it to the goal.


