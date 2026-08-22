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


To build a self-driving car we need to build a car.

Objectives:

As per the rules, the car must have a steering mechanism and the rear axle driven by a motor.
Our team also wanted the car to be stable and fast (and colorful if we can help it).


# 1. Mobility Management

## 1.1 Drive System
**Drive Motor:** JGA25-371 DC Motor with _____ encoder. MUST CALCULATE TORQUE (TO-DO)
  <table>
  <tr>
    <td align="center" width="300" >
      <img width="768" height="768" alt="image" src="https://github.com/user-attachments/assets/6da4969b-11e3-4240-9260-fae5cd6c5b2f" />
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Voltage: 12V </li>
        <li>No-load Speed: 126??????RPM </li>
        <li>Stall Torque: 4.2kg/cm</li>
        <li>Current: 0.046A</li>
        <li>Gear Ratio: 21.3:1</li>
      </ul>
    </td>
  </tr>
  </table>
  
  **Selection Reasoning:**
  + It is compact and lightweight, allowing us to fit it into our robot easily.
  + Built-in encoder, allowing for precise movement and less wiring hassle.
  + High torque for pushing the robot with vigor and avoiding stalling.

  The JGA25-371 motor was selected because it combines high torque with integrated encoder feedback, providing both the power and precision required for the WRO   Future Engineers competition. The motor drives the rear wheels through a metal WLTOYS 144001 differential, which allows the robot to maintain smooth and balanced turns while reducing the difference in rotational speed between the left and right wheels. By distributing torque between both LEGO SPIKE Prime wheels, the differential minimizes wheel slip and improves stability during cornering. The integrated Hall encoder provides real-time feedback for closed-loop control, allowing the robot to accurately measure wheel rotation, regulate speed, and maintain consistent movement throughout each run. Unlike smaller motors, the JGA25-371 provides sufficient torque to support the drivetrain while maintaining reliable performance under varying loads. The combination of encoder feedback, the metal differential, and the SPIKE Prime wheels creates a drivetrain that balances power, stability, and precision, making it well-suited for the competition environment.

  **Differential:**
  We used a metal WLTOYS 144001 differential in our robot, which allows the left and right rear wheels to rotate at different speeds while turning, reducing tire drag and improving turning efficiency. This setup provides smoother and more controlled cornering by distributing torque between both rear wheels and compensating for the difference in the distance each wheel travels during a turn. The differential improves maneuverability and stability, particularly during the obstacle challenge and parallel parking, where precise and repeatable movements are essential.
  
  <table>
  <tr>
    <td align="center" width="300" >
      <img width="1000" height="1000" alt="image" src="https://github.com/user-attachments/assets/5421c3d1-071a-458a-9708-831aaf08baae" />
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Differential Type: Planetary bevel-gear differential</li>
        <li>Material: Metal</li>
        <li>Application: 1:14 RC vehicles</li>
        <li>Drive Configuration: Rear-wheel drive</li>
        <li>Compatibility: 5 mm drive cups (custom coupled to LEGO axle)</li>
      </ul>
    </td>
  </tr>
  </table>

  **Mounting:**
  gseiohgsr

  **Considerations:**
  fwesng
  
## 1.2 Steering System

**Steering:** Initially Parallel Steering, then changed to Ackermann Steering Geometry, prototyped with LEGO technic parts and later made with 3D printed parts.
  insert images of prototypes

**Steering Motor:** MG90S Micro Servo for precise steering and weight reduction.
  <table>
  <tr>
    <td align="center" width="300" >
      <img width="1500" height="1500" alt="image" src="https://github.com/user-attachments/assets/7a6009ed-4475-47c5-9c7a-cf6d4c9111b9" />
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Voltage: 4.8V-6V </li>
        <li>Operating Speed: 0.1 s/60° at 4.8 V, 0.08 s/60° at 6 V</li>
        <li>Stall Torque: 1.8 kg·cm at 4.8 V, 2.2 kg·cm at 6 V</li>
        <li>Current: 0.010A</li>
        <li>Rotation Range: Approximately 180° (90° in each direction)</li>
      </ul>
    </td>
  </tr>
  </table>

**Selection Reasoning:**
* The compact size and PWM interface make the MG90S easy to integrate and control using the Arduino Nano.
* It provides sufficient torque to steer the front wheels accurately and responsively.
* Its fast response and metal gear construction provide reliable and stable steering during turns and lane changes.
* The MG90S is widely used in hobby robotics, making replacement parts, mounting hardware, and documentation readily available.

We considered several steering systems, but following our design principle of precision, we decided to implement Ackermann steering geometry to better replicate the controlled turning behavior of real-world vehicles.

Unlike simpler steering systems, Ackermann geometry allows each front wheel to turn at a different angle during a corner. This reduces tire slippage and improves steering accuracy, resulting in smoother and more controlled turns.

The fundamental principle of Ackermann geometry involves positioning the steering linkage so that the lines extending from the front wheels intersect at a common point along the rear axle of the robot.

<img src="v-photos/ackerman steering.png" alt="Ackerman Steering">

*This was an early prototype of the Ackerman Steering model before we added it to the first iteration of our Lego car chassis.*

Although this steering geometry is more complex to implement, we believe its advantages are especially important for obstacle navigation and parking, where precise control and a small turning radius are essential. It allows the robot to maneuver smoothly and maintain accurate alignment in tighter spaces.

Our implementation uses a custom 3D-printed Ackermann steering mechanism, with the following key design considerations:
* We used Onshape to experiment with different pivot points, linkage positions, and steering angles throughout the design process.
* Since achieving perfect Ackermann geometry at the robot's small scale is difficult, we aimed to closely approximate the ideal geometry through multiple design iterations.
* The MG90S servo horn and steering angles were adjusted directly in Onshape before 3D printing and physical testing.

IMAGE OF ACKEERMANN

**Calibration and Implementation:**
To achieve accurate and consistent steering, we used a combination of CAD adjustments and physical testing:
* Different servo positions, linkage lengths, and steering angles were tested in Onshape to determine the most suitable configuration.
* The physical steering assembly was then tested by turning the wheels fully in both directions and checking for smooth, consistent movement.
* Based on the test results, we modified the CAD model and repeated the process until the steering geometry provided the desired range of motion and wheel alignment.
* The final Onshape design was then used to produce the 3D-printed steering assembly.

**Mounting:**
abfaegneog
image

**Considerations:**
While the MG90S is suitable for our current steering system, we considered upgrading to the MG996R because it offers significantly higher torque and a more durable metal gear system. The increased torque would allow the servo to handle greater resistance from the steering mechanism and front wheels, providing more consistent steering and reducing the possibility of the servo struggling or losing its position during sharper turns. Its stronger construction would also make it more reliable under repeated use and during rapid steering adjustments or uneven surfaces. Although the MG996R is larger and requires more power than the MG90S, these disadvantages could be managed by modifying the chassis and power system. Overall, the MG996R would provide a stronger and more robust steering system, making it a potential upgrade if additional steering torque and durability are needed.

## 1.3 Chassis Design

isometric image of chassis

| Dimension | Value (mm) | Reason |
| ----------- | ----------- | -------------------------------------------- |
| Width | 120 | Component fit (Arduino, voltage converter and battery in line horizontally) |
| Length | 230 | Tight turns and component fit |
| Height | 200 | (Including camera wire) |

**Design Overview:**
wnog

**Iteration 1: LEGO**

IMAGE

Aspects acheived:
* A functioning robot capable of performing the game to a certain degree of efficiency.

Drawbacks:
* Tall and bulky design paired with a narrow body produced a high center of gravity and minimal support, making the robot unstable on quick turns and slower on straights.
* LEGO Technic design limited freedom of customizability, forcing any 3D printed parts to align to a LEGO frame.
* Limited range of movement for LEGO Ackermann steering system, constraining turns to 40 degrees to either side.
* Terrible tolerances (byproduct of using LEGO) result in constant plastic grinding in rear drive system.
* Inefficient component layout leaving much wasted space and area for drag

**Iteration 2: 3D Printed Parts and LEGO fusion:**

IMAGE

Aspects achieved/retained:
* Customizability of component mounting, such as custom in-built mounts for the IMU, DC Motor, etc.
* Expanded range of movement for newly 3D printed Ackermann steering system, allowing turns up to 55 degrees to either side.
* Improved tolerances and a switch from LEGO to metal differential remove the grinding in the read drive system.
* Optimized component layout for maximum space usage and minimal drag-creating parts.
* 

Drawbacks:
* LEGO camera mount restrains pinpoint mount changes to find optimal camera angle.
* LEGO servo mount is unnecessarily complex and is not fixed precisely, allowing for small movements when the servo moves.
* Slightly imprecise tolerances for rear differential drive system, resulting in jerky movement and inconsistent torque and speed.

**Iteration 3: Fully 3D Printed Chassis:**

IMAGE

Aspects achieved/retained:
* 3D printed camera mount allows for custom mounting; iteration testing resulted in us finding the optimal camera angle and height.
* 3D printed servo mount is a simple, single piece mount that effectively zeros servo shaking.
* Perfected tolerances in the rear differential drive system allow the motor to turn the rear axle efficiently with little energy lost and constant torque and speed.
* Customizability of component mounting, such as custom in-built mounts for the IMU, DC Motor, etc.
* Expanded range of movement for Ackermann steering system, allowing turns up to 55 degrees to either side.
* Optimized component layout for maximum space usage and minimal drag-creating parts.

Accepted Trade-Offs:
* Overall shape of the robot is not streamlined and produces drag, but minimal enough that it barely affects performance.
* 

**Layout:**
guewgbrig

Our robot chassis was completely custom-designed in FreeCAD and 3D printed using esun PLA+, which we found is easy to print with, offering a smoother texture and less warping compared to ABS, while also being lightweight and durable. Alongside the main chassis, the drivetrain and steering modules are mounted on our 3D-printed detachable plates that were fine-tuned during testing to achieve the correct alignment with other components. Other components, such as motor clamps and sensor brackets, are designed as independent printable components. The chassis was also designed with modularity in mind for replacements and upgrades, with reduced overhangs for printing ease. (PARAPHRASE)

# 2. Power Systems and Architecture

For the car to know what its doing, we need input and power.

Objective: 
  - Incorporate camera sensor
  - Create power system architecture for robot


## 2.1 Power Source

**Battery:** 3 Cell Lithium Battery

<table>
  <tr>
    <td align="center" width="300" >
      <img width="600" height="600" alt="image" src="https://github.com/user-attachments/assets/2157e72e-b7c3-4a06-a0a2-fe5b41f2cea0" />
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Nominal Voltage: 11.1V</li>
        <li>Fully Charged Voltage: 12.6V</li>
        <li>Capacity: 3000 mAh</li>
        <li>Discharge Rate: 30 C</li>
        <li>Maximum Continuous Current: 90 A</li>
        <li>Battery Type: Lithium-Polymer</li>
      </ul>
    </td>
  </tr>
  </table>

  yapyapyapyapyapyapyap

  **Motor Driver:** L298N
  
  <table>
  <tr>
    <td align="center" width="300" >
      <img width="750" height="500" alt="image" src="https://github.com/user-attachments/assets/d846cb1f-c902-4d4f-a61a-748b336f3ed7" />
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Driver Type: Dual H-Bridge</li>
        <li>Motor supply voltage: 5–35 V</li>
        <li>Maximum current: 2 A per channel</li>
        <li>PWM speed control</li>
        <li>Bidirectional motor control</li>
        <li>Overtemperature protection</li>
      </ul>
    </td>
  </tr>
  </table>

  ## 2.2 Sensor and Camera
  
  **SainSmart Wide-Angle Camera:** 

  <table>
  <tr>
    <td align="center" width="300" >
      <img width="1000" height="1000" alt="image" src="https://github.com/user-attachments/assets/5a9d887b-8b2f-4ce6-a781-90fff08bd01f" />
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Resolution: 5MP (2592 × 1944)</li>
        <li>Image Sensor: OV5647</li>
        <li>Field of View: 160°</li>
        <li>Interface: CSI</li>
        <li>Video: 1080p @ 30 FPS, 720p @ 60 FPS, 640 × 480 @ 60/90 FPS</li>
        <li>Dimensions: 25 × 24 × 9 mm</li>
      </ul>
    </td>
  </tr>
  </table>

  **Selection Reasoning:** 

  * 160° wide field of view from the wide-angle lens captures a larger area, improving the robot’s ability to detect and track its surroundings.
  * 5MP resolution provides clear and detailed images for vision-based tasks.
  * 1080p video at 30 FPS provides smooth real-time footage for image processing and object detection.
  * Compact and lightweight design allows the camera to be easily mounted on the robot without adding significant weight.

  This setup provides a 160° wide-angle view, improving the robot’s environmental awareness during both the Open Challenge and Obstacle Challenge. The camera captures course elements such as walls, pillars, colored markers, parking spaces, and lane lines, providing visual information for navigation and decision-making.

  **The camera is mainly used for the following tasks:**

  * Detect and determine wall positions.
  * Identify pillar colors and types.
  * Recognize parking zones.
  * Track path lines and boundaries.


   **BNO055 Inertial Measurement Unit (IMU):** 

  <table>
  <tr>
    <td align="center" width="300" >
      <img width="640" height="640" alt="image" src="https://github.com/user-attachments/assets/e744f7c4-f232-47a1-849b-ada9e6dda31e" />
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Sensor Type: 9-axis IMU (3-axis accelerometer, 3-axis gyroscope, 3-axis magnetometer)</li>
        <li>Processor: ARM Cortex-M0+ with integrated Bosch sensor-fusion firmware</li>
        <li>Interfaces: I²C, UART</li>
        <li>Operating Voltage: 3.3V</li>
        <li>Key Features: Absolute orientation, linear acceleration, gravity vector, magnetic heading, temperature sensing, and motion detection</li>
      </ul>
    </td>
  </tr>
  </table>

  **Selection Reasoning:** 

    The BNO055 is a compact 9-axis IMU that combines a 3-axis accelerometer, 3-axis gyroscope, and 3-axis magnetometer with an onboard processor for sensor fusion. It provides orientation data such as heading, Euler angles, linear acceleration, and gravity direction, reducing the need for complex external calculations.
    
    We mounted the BNO055 near the center of the chassis to provide stable and consistent measurements. It communicates with the Arduino Nano through the I²C interface, continuously providing real-time orientation and angular velocity data. This helps the robot maintain an accurate heading and make smoother steering adjustments, which are especially important during obstacle navigation.
  
  
  ## 2.3 Processing Units

  **Single Board Computer: Raspberry Pi 5**

  <table>
  <tr>
    <td align="center" width="300" >
      <img width="1200" height="1200" alt="image" src="https://github.com/user-attachments/assets/b2f8ca6d-7777-4d41-8b53-0eb7e7725d96" />
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Processor: Broadcom BCM2712</li>
        <li>CPU: Quad-core Arm Cortex-A76 (2.4 GHz)</li>
        <li>Power: 5 V / 5 A USB-C</li>
        <li>RAM: 8 GB LPDDR4X</li>
        <li>USB Ports: 2 × USB 3.0, 2 × USB 2.0</li>
        <li>Video Output: Dual micro-HDMI (60fps)</li>
      </ul>
    </td>
  </tr>
  </table>

  * The Raspberry Pi 5 acts as the robot’s primary processing unit, handling high-level decision-making, image processing, and the integration of sensor data.
  * Equipped with a quad-core Arm Cortex-A76 CPU and 8GB of RAM, it is capable of processing real-time camera data for applications such as mapping, obstacle detection, and path planning.
  * Its compact design also allows it to be easily mounted onto the chassis without adding significant weight.

  **Microcontroller:** Arduino Nano

  <table>
  <tr>
    <td align="center" width="300" >
      <img width="388" height="320" alt="image" src="https://github.com/user-attachments/assets/e3d85b52-7df4-4e75-ab28-ce22eec3256f" />
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>ATmega328P 8-bit microcontroller</li>
        <li>32 KB flash memory</li>
        <li>2 KB SRAM</li>
        <li>16 MHz clock speed</li>
        <li>22 multi-function I/O pins</li>
        <li>UART, I²C, and SPI communication</li>
      </ul>
    </td>
  </tr>
  </table>

  * The Arduino Nano acts as the robot’s low-level controller, managing tasks such as motor control, steering, and sensor input.
  * It communicates with the Raspberry Pi 5 through a serial connection, separating high-level processing from precise hardware control.
  * Its fast and reliable response allows for accurate control of the robot’s motors and steering system, resulting in smoother and more consistent movement.

  **Expansion Board:** Arduino Nano Shield

  <table>
  <tr>
    <td align="center" width="300" >
      <img width="894" height="861" alt="image" src="https://github.com/user-attachments/assets/a4056b59-17b4-4e73-a7d9-48c3cd8f3f62" />
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Interface: Arduino Nano-compatible pin headers</li>
        <li>Supported Devices: Arduino Nano and compatible modules/sensors</li>
        <li>I/O Access: Digital, analog, PWM, UART, I²C, and SPI pins</li>
        <li>Power Supply: 5V and 3.3V power outputs</li>
      </ul>
    </td>
  </tr>
  </table>

  * The Arduino Nano Shield expands the robot’s capabilities by providing a convenient platform for connecting and controlling external components such as motors, sensors, and other peripherals.
  * It simplifies wiring by providing accessible connection points for the Arduino Nano’s input and output pins, while also supporting power distribution to connected components.
  * This makes the system easier to assemble, maintain, and modify, while providing reliable connections between the Arduino Nano and the robot’s hardware.

  ## 2.4 Circuit Diagram

  **Wiring Diagram:**

  IMAGE OF DIAGRAM

  ## 2.5 Power Consumption

  | Component | Supply (V) | Typical Current (A) | Peak Current (A) | Typical Power (W) |
|------------|------------|---------------------|------------------|-------------------|
| Raspberry Pi 5 | 5 | 0.80 | 5.00 | 4.00 |
| Arduino Nano | 5 | 0.019 | 0.030 | 0.095 |
| 5MP SainSmart Wide-Angle Lens Camera | 5 | 0.15 | 0.25 | 0.75 |
| BNO055 IMU | 3.3 | 0.012 | 0.015 | 0.04 |
| Micro Servo MG90S (steering) | 5 | 0.20 | 0.80 | 1.00 |
| JGA25-371 DC Motor | 12 | 0.50 | 2.00 | 6.00 |
| L298N Motor Driver | 12 | 0.02 | 0.04 | 0.24 |
| Arduino Nano Shield | 5 | 0.019 | 0.030 | 0.095 |
| Step-Down Converter | — | — | — | η ≈ 88% |
  


+ **Torque Calculations:**
  [insert torque calculation image]

+ **3D Printed Structure:** Base layer design with a plate that holds the raspberry pi and a mounting plate that holds the step down voltage converter and Arduino Nano.
  [insert cross-sections and isometric and face images of robot]

+ **Custom Mounts:** Mounts and holders for servo, camera, raspberry pi, and a joint mount for step down voltage converter and Arduino Nano. Custom Motor-Differential, Servo-Axle, and Differential-Axle coupling.
  [pictures and link]

+ **Wheels:** Spike Prime wheels (56mm diameter, 14mm thick) for compactness and grip.
  [link to explanation]

+ **Iterations:** As you can see in the following image, this was an iterative project with many versions of each part.
  [insert image of all iterations]


###  Power System Architecture

<img src="v-photos/Screenshot 2026-08-10 193454.png" alt="Ackerman Steering">

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






