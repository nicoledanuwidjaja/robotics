# Project 8 - Computer Vision

Strategy: Use OpenCV to detect and process lines in camera output. The Canny edge detection algorithm was used to process images:

1. Use Gaussian blur to smooth image and reduce noise levels.
2. Convert image from RGB color space --> HSV color image --> grayscale --> binary representation
3. Find and filter intensity gradients to apply thresholds.
4. Determine potential edges and track by suppressing weaker edges using the Hough line detection voting algorithm.
5. Use slope filter to find single line of best fit that represents the closest walls.

The robot will traverse the maze to visualize the map in hopes of finding the goal.

YouTube Video: None :(

Nicole Danuwidjaja
