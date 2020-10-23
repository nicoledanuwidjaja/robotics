# Project 6 - Maze Pt. 2

Nicole Danuwidjaja

Write a control program that enables a simulated robot to wander a maze and generate a map.

Strategy: The robot will attempt to wander the entire maze using the wall following strategy. The total size of the map is precalculated and discretized so that it is represented by a Cell class. The robot has three sensors for forward, left, and right. When the robot encounters an obstacle, it will create a thread to mark the cell as occupied (red); if no obstacle is encountered, the robot will mark the cell as free (yellow).

Used Bayesian statistics and occupancy grid mapping to calculate and generate the maze on GTK+ Widget.

YouTube Video: https://youtu.be/uqbISrT-EkA 
