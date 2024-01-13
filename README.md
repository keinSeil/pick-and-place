# Pick-and-Place Project

## Overview
This project develops a computer vision system that locates objects for a 3D-printed robot. It integrates the kinematics and controls of a three degree-of-freedom robot, image segmentation techniques to separate objects of interest from the background, and an Intel Realsense 435 "stereo" camera to locate objects in 3D space.

Check out the demo here: https://youtu.be/oYR2Y1Otlyo

## Equipment
- Robot kinematics and controls are derived within the MATLAB computation environment.
- Implementation within the Arduino IDE.
- Image processing and segmentation are implemented in MATLAB on a Windows desktop computer.

## How It Works
1. An image is taken of the robot's work area.
2. The user defines a region of interest (first time setup only).
3. The image is segmented to separate the chess pieces from the rest of the image.
4. The user sets the origin interactively, needed for coordinate transformation between the camera and robot (first time setup only).
5. The user verifies the accuracy by clicking on different parts of the robot work area grid and comparing the output coordinates to the grid's coordinates (first time setup only).
6. The path planning algorithm moves the robot to pick-and-place the chess piece onto a pre-determined location.

## Current Issues
The system currently struggles with missing depth information due to the orientation of the camera relative to the robot's work area. The camera's top-down view does not capture depth information effectively from the middle of taller objects or objects farther from the camera. This results in the centroid (marked in red) representing a point on the work area passing through the chess piece, rather than the object's actual location.

## Note
During the second pickup sequence, the issue with depth perception causes the king to be thrown around unexpectedly.

## Code Structure
- Main code: `main.m` and `00_PaP_Final.ino`.
- Other files include functions used in the main script and additional scripts for calibration or training images.

## Recommendations
For a deeper understanding, it's recommended to examine the `main.m` and `00_PaP_Final.ino` scripts, which contain the bulk of the program.
