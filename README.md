# PUMA560 MATLAB Simulation

A MATLAB-based simulation of a PUMA 560 robotic arm performing pick and place operations using the Robotics Toolbox.

## Overview
This project demonstrates a 3D simulation of the PUMA 560 robotic manipulator executing a pick and place task. The simulation includes:
- Forward and inverse kinematics
- Trajectory planning
- Smooth motion control
- Visual representation of the robot's workspace
- Object manipulation visualization

## Prerequisites
- MATLAB (Tested on R2023a)
- [Robotics Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/) by Peter Corke
- MATLAB's Computer Vision Toolbox

## Installation
1. Clone this repository:
- git clone https://github.com/yourusername/puma560-matlab-simulation.git

2. Install the Robotics Toolbox:

- Download the toolbox from here
- Add the toolbox to your MATLAB path

3. Update the toolbox path in the main script:

- addpath(genpath('path/to/your/rvctools'))

## Usage

- Open MATLAB
- Navigate to the project directory
- Run the main simulation script puma_simulation.m

## Features

- Robot Model: Accurate PUMA 560 DH parameters implementation
- Motion Planning: Smooth trajectory generation using joint space interpolation
- Visualization:
-- Real-time 3D animation
-- Object status indicators
-- Position markers
-- Motion state display

## Task Sequence:

1. Home position to approach point
2. Object pickup
3. Transfer motion
4. Object placement
5. Return to home position

## Demo
- Watch the simulation demo on YouTube
- https://youtu.be/HT4EO40KXPs
