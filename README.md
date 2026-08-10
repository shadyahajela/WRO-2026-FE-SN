Engineering materials
====

This repository contains engineering materials of a self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2022.

## Content

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

## Introduction

We are the Blue Lobsters, Niva, Jeevesh and Shadya. We are united by a passion for STEM, Electronics, coding and jalapeño poppers. We wanted to bring our jouney to many others with the same passions and inspire teenagers around the world. To us its more than just coding or building, its the experiences and connections along the way.

## Robot 

Our robot is called Jadoo which means magic in Hindi. The name comes from a popular movie character who is also an alien and can do Jadoo. The drive base of our robot kind of looked like an alien head, so we named it after our favorite alien character.

**Dimensions**

  Width: 12 cm                     
  Length: 23 cm                    
  Height: 17 cm (Without wire)      
          20 cm (With Camera wire)  
          
The Width helps to get around tight areas and the height gives a low base with a more balanced center of gravity all while being easy to maneuver due to the length. 

### Robot images
_*Add at end*

# 1. Mobility & Mechanical Design

To build a self-driving car we need to build a car.

Objectives:

As per the rules, the car must have a steering mechanism and the rear axle driven by a motor.
Our team also wanted the car to be stable and fast (and colorful if we can help it).

### 1.1 Chassis
Steering: We use the Ackerman steering to turn accurately. The Ackerman steering is a design where the inside front wheel turns sharper than the outside front wheel when a car goes around a corner.

<img src="v-photos/ackerman steering.png" alt="Ackerman Steering">

*This was an early prototype of the Ackerman Steering model before we added it to the first iteration of our Lego car chassis.*

Drive train: A drive train is all the parts that transfer power from the motor to the rear wheels. These are the parts used
  - Lego rubber wheels: The Lego wheels were the most available and efficient than printing wheels.
  - WLTOYS 144001 differential: A differential is a drive that is controlled by one motor but can run both wheels independently. The particular differential is made of metal ensuring fool proof connections. 

### 1.2 Drive/Steering choice

### 1.3 Motor selection/ speed reasoning

### 1.4 Mounting  of components

# 2. Power & Sensor Architecture

For the car to know what its doing, we need input and power.

Objective: 
  - Incorporate camera sensor
  - Create power system architecture for robot

### 2.1 Power System Architecture


### 2.2 Current Draw Reasoning 

### 2.3 Sensor Selection and Justification

### 2.4 Sensor Placement and Calibration

### 2.5 Wiring Diagram

# 3. Software Architecture

### 3.1 Code Implementation 

### 3.2 Modularity 

### 3.3 Code Comments 

### 3.4 State Machine 

### 3.5 PID control

### 3.6 Algorithm

### 3.7 Lane Following

### 3.8 Obstacle

### 3.9 Parking

### 3.10 Microcontroller code

### 3.11 Testing / Tuning Process

### 3.12 Performance Measurement 

### 3.13 Flow charts

# 4. Subsystem Thinking and Engineering Decisions

### 4.1 Subsystem integration with constraints and tradeoffs

### 4.2 Iteration Cycles With Reasoning

### 4.3 Engineering Reasoning With Risk Analysis






