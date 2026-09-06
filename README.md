Team Sub-Atomica
====

This repository contains the engineering documentation and our journey of building an autonomous vehicle to compete at the 2026 WRO Future Engineers competition.

<p align="center">
<img width="562" height="507" alt="image" src="https://github.com/user-attachments/assets/5c8130d9-c9df-4284-a268-e8d9e77e684b" />


## Introduction

We are Team Sub-Atomica, Niva, Jeevesh and Shadya. We are united by a passion for STEM, Electronics, coding and jalapeño poppers. We wanted to bring our jouney to many others with the same passions and inspire teenagers around the world. To us its more than just coding or building, its the experiences and connections along the way.

<img width="330" height="248" alt="IMG_6842" src="https://github.com/user-attachments/assets/0a190bb5-6e52-40ad-af4f-699491a4c7ea" />
<img width="330" height="248" alt="159C16BD-3B52-4F8A-85BE-AFA5FE405926 (1)" src="https://github.com/user-attachments/assets/3ceb7079-0236-4770-a4e8-5067800ad02a" />
<img width="330" height="248" alt="IMG_6102 (1)" src="https://github.com/user-attachments/assets/dd9d688d-cd1b-44b3-93f7-74d6a8bdba7c" />

## Robot 

Our robot is called Jadoo which means magic in Hindi. The name comes from a popular movie character who is also an alien and can do Jadoo. The drive base of our robot kind of looked like an alien head, so we named it after our favorite alien character.

### Robot images
<img width="300" height="408" alt="image" src="https://github.com/user-attachments/assets/af7561bf-10e8-4b97-ac37-556d9599feff" />
<img width="300" height="408" alt="image" src="https://github.com/user-attachments/assets/259c8fb3-f92e-446a-88b5-39c5ec398d54" />
<img width="300" height="408" alt="image" src="https://github.com/user-attachments/assets/0b135114-54b1-469a-9566-dff8d1d074bd" />
<img width="300" height="408" alt="image" src="https://github.com/user-attachments/assets/321d071c-ed87-43dc-b66c-276058c2d453" />
<img width="300" height="408" alt="image" src="https://github.com/user-attachments/assets/ce060441-80ed-43f7-803a-4352d1043b11" />
<img width="300" height="408" alt="image" src="https://github.com/user-attachments/assets/6c13cf31-8b4d-4c99-8bdf-1608f29fbbc6" />

**Dimensions**

| **Specification** | **Measurement** |
|---|---|
| **Width** | 12 cm |
| **Length** | 23 cm |
| **Height** | 17 cm (Without wire) |
| | 20 cm (With Camera wire) |
| **Weight** | About 800 grams |
          
The Width helps to get around tight areas and the height gives a low base with a more balanced center of gravity all while being easy to maneuver due to the length. 

## Open challenge videos
[![Watch the video](https://github.com/user-attachments/assets/bb547e53-6020-4515-87ad-47aec5ea7645)](https://www.youtube.com/watch?v=8OkpqqfYOhU )

## Obstacle challenge video
[![Watch the video](https://github.com/user-attachments/assets/e8f30f3e-51ba-4644-936d-f1f42620b62b)](https://www.youtube.com/watch?v=r0wWpeo_AAc)


## Content

* `t-photos` contains photos of the team.
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

# 1. Mobility Management

To build a self-driving car we need to build a car.

Objectives:

As per the rules, the car must have a steering mechanism and the rear axle driven by a motor.
Our team also wanted the car to be stable and fast (and colorful if we can help it).

## 1.1 Drive System
**Drive Motor:** JGA25-371 DC Motor with Hall-encoder. 
  <table>
  <tr>
    <td align="center" width="300" >
      <img width="768" height="768" alt="image" src="https://github.com/user-attachments/assets/6da4969b-11e3-4240-9260-fae5cd6c5b2f" />
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Voltage: 12V </li>
        <li>No-load Speed: 620RPM </li>
        <li>Stall Torque: 4.2kg/cm</li>
        <li>Current: 0.046A</li>
        <li>Gear Ratio: 21.3:1</li>
      </ul>
    </td>
  </tr>
  </table>

 **Selection Reasoning** 

+ Built-in encoder, allowing for precise movement and less wiring hassle.
+ High torque for pushing the robot with vigor and avoiding stalling.

The JGA25-371 motor was selected because it combines high torque with integrated encoder feedback, providing both the power and precision required for the WRO Future Engineers competition. The 620 RPM gives enough torque to have a fast robot and have enough control for parking. We calculated torque required to pull our robot and to start it. Additionally the hall encoder provides real-time feedback for closed-loop control, allowing the robot to accurately measure wheel rotation, regulate speed, and maintain consistent movement especially during parking. 

<img width="700" height="516" alt="image" src="https://github.com/user-attachments/assets/4a14070e-126d-4522-b77e-8ea55c50e8be" />

*Image pinpoints exact location of torque calculations.(for full image look for Torque and speed reasoning)*
<br/>

 **Considerations:**
  An alternative would be a NEMA 17 stepper motor, which can provide precise speed and position control. However, the NEMA 17 is larger and heavier, and would require a more complex driver setup, making it less suitable for our robot. The JGA25-371 was chosen instead because it provides sufficient speed and torque while being more compact and easier to integrate.

### Differential

We used a metal WLTOYS 144001 differential in our robot, allowing the left and right rear wheels to rotate at different speeds during turns, reducing tire scrub and improving turning efficiency. This provides smoother cornering by distributing torque between both wheels and compensating for the different distances they travel. The differential improves maneuverability and stability during the Obstacle Challenge and parallel parking, where precise and repeatable movement is essential, while its metal construction provides durability under drivetrain loads.
  
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

  * Installed using an inset groove system in the chassis screwed to a detachable 3D-printed motor differential housing clamp that is placed above the differential gear compartment. This will allow for future changes to accommodate different motors and gears if the need arise.

    <img width="325.5" height="434.5" alt="image" src="https://github.com/user-attachments/assets/ee7787bb-2744-43ed-9582-68ccc280698d" />

  * Wires connected to Arduino Nano and Motor Driver.
  * Spike Prime wheels fitted onto the LEGO motor axle.


  
## 1.2 Steering System

**Steering:** Initially Parallel Steering, then changed to Ackermann Steering Geometry, prototyped with LEGO technic parts and later made with 3D printed parts. Ackermann was especially important for obstacle navigation and parking, where precise control and a small turning radius are essential. We also considered parallel steering because it was simpler to implement, but chose Ackermann as a calculated risk for its improved maneuverability. Our implementation uses a custom 3D-printed Ackermann steering mechanism, we used Onshape to experiment with different pivot points, linkage positions, and steering angles throughout the design process.

Through 2 iterations, we shortened both rods and increased the length of the side rods to achieve a smaller turning radius while keeping the mechanism       compact. In our first iteration we made our tabs 2mm longer and the neck of the robot, this gave us a 15 degree increase in our turning radius. We also experimented with shortening the rods to make our wheels streamlined and finalized with 9.53cm for the long shaft and 8.5cm for the short one. A major constraint was the chassis neck, which limited the steering angle because the wheels could collide with it. We therefore cut part of the chassis around the steering area to provide sufficient clearance.

**Calibration and Implementation:**
To achieve accurate and consistent steering, we used a combination of CAD adjustments and physical testing:

* Different servo positions, linkage lengths, and steering angles were tested in Onshape to determine the most suitable configuration.
* The physical steering assembly was then tested by turning the wheels fully in both directions and checking for smooth, consistent movement.
* Based on the test results, we modified the CAD model and repeated the process until the steering geometry provided the desired range of motion and wheel           alignment.
* The final Onshape design was then used to produce the 3D-printed steering assembly.

<img width="275" height="489" alt="IMG_4013" src="https://github.com/user-attachments/assets/f28c9ddb-460f-42f4-8379-5101067f59bd" />
<img width="650" height="424" alt="image" src="https://github.com/user-attachments/assets/9caed275-5877-459d-ba2e-db10171e0d1b" />
<img src="v-photos/ackerman steering.png" alt="Ackerman Steering">

*This was an early prototype of the Ackerman Steering model before we added it to the first iteration of our Lego car chassis.*


### Steering Motor

We chose the MG90S Micro Servo for precise steering and weight reduction. The compact size and PWM interface make the MG90S easy to integrate and control using the Arduino Nano. It provides sufficient torque to steer the front wheels accurately and responsively. Its fast response and metal gear construction provide reliable and stable steering during turns and lane changes. The MG90S is widely used in hobby robotics, making replacement parts, mounting hardware, and documentation readily available. Additionally while testing it with our code for control we were able to make it work 90% of the time, upon investigating we found a fault with the power system in the testing module which we solved by adjusting our wires. 

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



**Mounting:**

* Screwed into a platform plate in front of the chassis, connected to the steering mechanism.

  <img width="579.5" height="434.5" alt="image" src="https://github.com/user-attachments/assets/e2cee4a4-7c77-436e-8802-3e7016e79ba2" />

**Considerations:**
An alternative would be the MG996R servo, which provides higher torque and more durable metal gears. However, it is larger and requires more power than the MG90S, which would require modifications to our chassis and power system. The MG90S was chosen instead because it is more compact and sufficient for our steering system.

## 1.3 Chassis Design


| Dimension | Value (mm) | Reason |
| ----------- | ----------- | -------------------------------------------- |
| Width | 120 | Component fit (Arduino, voltage converter and battery in line horizontally) |
| Length | 230 | Tight turns and component fit |
| Height | 200 | (Including camera wire) |

**Design Overview:**

**Iteration 1: LEGO**

<img width="216" height="288" alt="image" src="https://github.com/user-attachments/assets/7ef8f9a2-31b6-4438-8b2a-51e36d70d14d" />
<img width="216" height="288" alt="image" src="https://github.com/user-attachments/assets/498d1418-7f5b-4504-8630-1bbe256acc12" />
<img width="216" height="288" alt="image" src="https://github.com/user-attachments/assets/fde05a0b-671a-4879-acd4-6e87f84e8fb9" />
<img width="216" height="288" alt="image" src="https://github.com/user-attachments/assets/219b3825-5091-4f02-a971-c35816ce2748" />


Aspects acheived:
* A functioning robot capable of performing the game to a certain degree of efficiency.
* Experiment with a new camera angle.                                     

Drawbacks:
* Tall and bulky design paired with a narrow body produced a high center of gravity and minimal support, making the robot unstable on quick turns and slower on straights.
* The low camera angle caused loss of depth perception, we we decided to completely reengineer it. 
* LEGO Technic design limited freedom of customizability, forcing any 3D printed parts to align to a LEGO frame.
* Limited range of movement for LEGO Ackermann steering system, constraining turns to 40 degrees to either side.
* Terrible tolerances (byproduct of using LEGO) result in constant plastic grinding in rear drive system.
* Inefficient component layout leaving much wasted space and area for drag

**Iteration 2: 3D Printed Parts and LEGO fusion:**

<img width="244.5" height="424.5" alt="image" src="https://github.com/user-attachments/assets/24dba292-0dd8-4a38-afa0-b5159013af0e" />

Aspects achieved/retained:
* Customizability of component mounting, such as custom in-built mounts for the IMU, DC Motor, etc.
* Expanded range of movement for newly 3D printed Ackermann steering system, allowing turns up to 55 degrees to either side.
* Improved tolerances and a switch from LEGO to metal differential remove the grinding in the read drive system.
* Optimized component layout for maximum space usage and minimal drag-creating parts.
  

Drawbacks:
* LEGO camera mount restrains pinpoint mount changes to find optimal camera angle.
* LEGO servo mount is unnecessarily complex and is not fixed precisely, allowing for small movements when the servo moves.
* Slightly imprecise tolerances for rear differential drive system, resulting in jerky movement and inconsistent torque and speed.

**Iteration 3: Fully 3D Printed Chassis:**

<img width="325.5" height="434.5" alt="image" src="https://github.com/user-attachments/assets/dfaddd8d-61b9-4028-ac8f-ad6805f09e88" />

Aspects achieved/retained:
* 3D printed camera mount allows for custom mounting; iteration testing resulted in us finding the optimal camera angle and height.
* 3D printed servo mount is a simple, single piece mount that effectively zeros servo shaking.
* Perfected tolerances in the rear differential drive system allow the motor to turn the rear axle efficiently with little energy lost and constant torque and speed.
* Customizability of component mounting, such as custom in-built mounts for the IMU, DC Motor, etc.
* Expanded range of movement for Ackermann steering system, allowing turns up to 55 degrees to either side.
* Optimized component layout for maximum space usage and minimal drag-creating parts.

Accepted Trade-Offs:
* Overall shape of the robot is not streamlined and produces drag, but minimal enough that it barely affects performance.
* Minimal space for wire management, wires don't affect functions of the robot so its accepted.


Our chassis was designed on Onshape and printed using Carbon fiber, as it is light and resistant to snapping, which helps in thin and high tension areas like the neck of our robot. Other than the chassis itself the other 3D printed components use PLA and are designed on Onshape as well.

## 1.4 Torque and Speed reasoning

<img width="870" height="645" alt="image" src="https://github.com/user-attachments/assets/f74db39f-b30e-49f5-a9ce-666f96bb5f14" />

Our torque and speed reasoning lists the given parameters, we are able to calculate the max speed, the robots acceleration as well as the gives the minimum torque required to move our robot, allowing us to control speed. We are also able to calculate the cruising torque and stall torque which we used for selecting our motor. Based on our robot's total mass of 0.8 kg and wheel diameter of 0.056 m, we calculated a required motor torque of approximately 0.0117 N·m during acceleration, which is well within our motor's stall torque of 0.0833 N·m, giving a safety margin of roughly 7x (required torque is only ~14% of available stall torque).


# 2. Power Systems and Hardware Architecture

For the car to know what its doing, we need input and power.

Objective: 
  - Select the right hardware / electronic components for the entire sensor input processing and actuator control in real-time 
  - Supply adequate power to all these components

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

The power system is essential to the robot’s ability to operate reliably throughout the competition. Our robot is powered by a 3-cell (3S) Lithium Polymer (LiPo) battery. The battery provides a nominal voltage of 11.1 V and a fully charged voltage of 12.6 V. Its high discharge capability allows it to supply sufficient current to the motors and other components. The battery’s high energy density provides a good balance between runtime, weight, and available power, making it well suited for a mobile competition robot. A rocker switch is connected directly to the power source independent of the code to switch on and off the robot. 

The JGA25-371 motors operate from the 12V supply provided by the battery, while the L298N motor driver regulates the power delivered to the motors based on commands from the Arduino Nano. The higher voltage of the 3S battery allows the motors to operate without requiring a step-up converter, simplifying the power system and reducing unnecessary conversion losses. A step-down converter is used to reduce the battery voltage to the appropriate levels required by the Raspberry Pi 5, Arduino Nano, camera, IMU, and other electronics. This separates the high-current motor supply from the lower-voltage electronics while allowing the entire robot to be powered from a single battery.

The Raspberry Pi 5 serves as the vehicle’s main processing unit and requires a stable 5V supply. The step-down converter provides the required regulated voltage, ensuring that the Raspberry Pi and other electronics receive consistent power during operation. This setup allows the robot to run its sensors, process camera data, control the motors, and make navigation decisions without relying on separate batteries for each subsystem.

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

We chose the L298N motor driver because it provides reliable control of our JGA25-371 DC motors while being compatible with the Arduino Nano. It supports the 12V supply from our 3-cell LiPo battery and allows the Arduino to control motor direction and speed through its PWM inputs. This provides a simple and reliable connection between the robot’s power system and drivetrain, making it well suited for our differential drive system. The driver has a voltage drop in its internal transistors which causes us to not achieve its full potential, but we were willing to make a trade-off as we were content with what we were able to use it for.  


OLD MOTOR DRIVER IMAGE: DRV8871

<img width="140" height="92.7" alt="image" src="https://github.com/user-attachments/assets/1ebcd3af-6231-4427-b338-a13d668f537a" />

We used DRV8871 in earlier iterations of the robot because of its compactness and its single motor control. However, it kept burning out due to motor feedback coming back to the driver. Motor feedback occurs when a motor is forced to move, and was probably caused by the constant pulling of the robot we did when testing. After further research, we found out that this feedback was a common problem with the DRV8871, so we decided to move to the L298N motor driver. Although it is much larger and bulkier, we had considerable space for the motor driver so it wasn't an issue. The L298N also has a heatsink, so overheating is not a problem.

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

- 160° wide field of view captures more of the course, reducing blind spots and improving environmental awareness.
- 5MP resolution provides enough detail to reliably distinguish course features and colored markers.
- 1080p at 30 FPS provides responsive visual data for real-time image processing and object detection.
- Compact and lightweight design reduces its impact on vehicle weight and balance while allowing flexible mounting.

This setup provides a 160° wide-angle view that increases the robot's visual coverage during both the Open Challenge and Obstacle Challenge. The camera supplies visual information about walls, pillars, colored markers, parking spaces, and lane lines, which is combined with other sensor data to support navigation and decision-making. The main tradeoff is balancing wider coverage with image processing requirements, so the selected resolution and frame rate provide sufficient detail and responsiveness without unnecessarily increasing processing demands.

The camera is mainly used for the following tasks:

- Detect and determine wall positions.
- Identify pillar colors and types.
- Recognize parking zones.
- Track path lines and boundaries.

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

Considering we wanted to have a relative compass to reduce additional code in the Arduino, we chose the BNO055. The BNO055 is a compact 9-axis IMU that combines a3-axis accelerometer, 3-axis gyroscope, and 3-axis magnetometer with an onboard processor for sensor fusion. It provides orientation data such as heading,       Euler angles, linear acceleration, and gravity direction, reducing the need for complex external calculations.
    
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

*Subsystem Integration:*
The Raspberry Pi 5 acts as the robot’s primary processing unit, handling high-level decision-making, image processing, and sensor-data integration. Its processing capability allows it to run real-time vision, mapping, obstacle detection, and path-planning tasks while communicating with the Arduino Nano for low-level control.

*Hardware and Software Architecture:*
The Raspberry Pi 5’s quad-core Arm Cortex-A76 CPU and 8GB of RAM provide sufficient processing performance for the computational demands of the robot. Its compact form factor also allows it to be mounted directly onto the chassis without significantly increasing the vehicle’s size or weight. It communicates with the Arduino Nano through a serial connection, creating a modular architecture where high-level processing and low-level control are separated.

*Iterations and Trade-Offs:*
We initially tested a Raspberry Pi 4 to reduce power consumption, but testing showed that its lower processing performance caused slower image processing and reduced responsiveness during real-time vision and navigation tasks. We therefore changed to the Raspberry Pi 5, accepting higher power consumption in exchange for significantly improved processing performance and more reliable real-time operation.

*Risk Analysis and Reliability:*
Using the Raspberry Pi 5 reduces the risk of processing bottlenecks affecting navigation and vision performance. Separating high-level processing from the Arduino Nano’s low-level control also prevents computationally intensive tasks from directly interfering with motor and steering control, improving overall system reliability.
  

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

+ The Arduino Nano was selected as the robot’s low-level controller to handle time-sensitive tasks such as motor control, steering, and encoder/sensor input. This architecture separates high-level decision-making from low-level control, allowing the Raspberry Pi 5 to focus on vision processing, navigation, and path planning while the Arduino maintains consistent motor and steering responses.

+ The Arduino Nano communicates with the Raspberry Pi 5 through a serial connection, creating a modular hardware and software architecture. This reduced the risk of timing conflicts caused by running motor-control tasks alongside computationally intensive processes on the Raspberry Pi. It also allows each subsystem to be tested independently, making troubleshooting and future modifications easier. It also displays steering values on the 1.12" OLED screen, we use it for understanding what the robot is thinking and troubleshoot. 

+ Our initial design used a microbit, but research showed that the Arduino Nano offered greater flexibility for external hardware and simpler bidirectional communication with the Raspberry Pi 5. We therefore changed to the Arduino Nano, trading the microbit’s built-in buttons and LEDs for improved subsystem integration and expandability.

+ The control system was then iteratively tested and adjusted to improve motor response, steering accuracy, and communication reliability. Using a dedicated microcontroller introduced additional hardware and communication complexity, but provided more predictable real-time control and reduced the dependence of critical drivetrain functions on the Raspberry Pi’s processing workload.

+ This subsystem architecture improves reliability by isolating critical low-level control from higher-level software. If the vision or navigation software requires significant processing, the Arduino can continue executing motor and steering commands without relying on continuous high-level computation. This separation therefore supports more consistent movement and provides a more robust architecture for the challenges defined by WRO Future Engineers.


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

  <img width="1224" height="1763" alt="Wiring Diagram (2)" src="https://github.com/user-attachments/assets/4c6feabf-4b5b-4540-9d90-26b686d79076" />

  The Wiring diagram shows the connections to the different components that are located in the robot.
  
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

## 2.6 Power System Architecture

<img width="1167" height="607" alt="image" src="https://github.com/user-attachments/assets/c2637fd0-5c40-4e5f-b6e0-8fb43fe7991f" />

This diagram shows how the 12V battery supplies power to the DC motor through the motor driver, and the step down buck converter which them supplies power to the rest of the components. 

## 2.7 List of Components

| Major components                                                 | Reference Cost in CAD<br>\- As of Sep 1, 2026<br>\- Taxes not included | Purchase Link for Future Reference                                                            |
| ---------------------------------------------------------------- | ---------------------------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| Raspberry Pi 5 4GB                                               | $153.95                                                                | [pishop.ca](https://www.pishop.ca/product/raspberry-pi-5-4gb)                                 |
| Arduino Nano                                                     | $27.19                                                                 | [Amazon.ca](https://www.amazon.ca/dp/B0F6Y7GS4Q)                                              |
| Arduino Nano Expansion Board                                     | $13.99                                                                 | [Amazon.ca](https://www.amazon.ca/dp/B08198MSJ2)                                              |
| 1.3" OLED display module SH1106 128x64                           | $5.33                                                                  | [aliexpress.com](https://www.aliexpress.com/item/1005006827988792.html)                       |
| Sainsmart Wideangle 5MP Camera 160 degree FoV (OV5647 sensor)    | $12.91                                                                 | [Amazon.ca](https://www.amazon.ca/SainSmart-Fish-Eye-Camera-Raspberry-Arduino/dp/B00N1YJKFS)  |
| Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 | $38.38                                                                 | [Aliexpress.com](https://www.aliexpress.com/item/1005010734176030.html)                       |
| 12V Battery                                                      | $27.99                                                                 | [Amazon.ca](https://www.amazon.ca/12V-2800mAh-Rechargeable-Battery-Replacement/dp/B0FJ27ZT28) |
| 12V to 5V Step-down converter (XL4015 with Display)              | $4.80                                                                  | [aliexpress.com](https://www.aliexpress.com/item/1005008401247033.html)                       |
| JGA25-371 620 rpm 12V Brushless DC motor with Encoder            | $12.78                                                                 | [aliexpress.com](https://www.aliexpress.com/item/1005007546764319.html)                       |
| WItoys 144010 Metal Differential Gearbox                         | $40.58                                                                 | [aliexpress.com](https://www.aliexpress.com/item/1005005869550963.html)                       |
| L298N Motor Driver                                               | $11.99                                                                 | [Amazon.ca](https://www.amazon.ca/dp/B0D8G2PZBB)                                              |
| MG90S Servo motors                                               | $17.02                                                                 | [Amazon.ca](https://www.amazon.ca/dp/B0BWJ4RKGV)                                              |
| Short USB to USB-C cable                                         | $11.39                                                                 | [Amazon.ca](https://www.amazon.ca/dp/B0DG8J6S1G)                                              |
| Rocker Switch                                                    | $2.65                                                                  | [aliexpress.com](https://www.aliexpress.com/item/1005007044175800.html)                       |
| Push Button breakout module                                      | $1.52                                                                  | [aliexpress.com](https://www.aliexpress.com/item/32820437436.html)                            |
| Lego wheels, axles and connectors                                | $50.00                                                                 | Approximately                                                                                 |
| 3D printed parts for the chassis                                 | $25.00                                                                 | Approximately for one spool of PLA filament                                                   |
| Total                                                            | $473.49                                                                |                                                                                               |


# 3. Software Architecture

## 3.1 Code Origins
Our  coding journey had humble origins following our research of other teams since many of the concepts required to build and program an autonomous vehicle were new to us, 

1. Started with just a raspberry pi and a camera to detect walls and objects
2. Added proportional steering angle calculation to drive a virtual servo
3. Had a Micro\:bit microcontroller connected to the Raspberry Pi over USB for serial connection, and coded it to receive the servo values from the Pi to drive an actual servo
4. Added a DC motor and driver to the microcontroller for vehicle movement, and ensured all of these worked in tandem
5. Put this entire setup on a Lego chassis for iteration 1 
6. Tested this iteration on a table-top test bench with actual walls and 3 printed color blocks to tune the steering values for walls and obstacles. The rotating test bench was made out of Lego.
7. We then changed to Arduino since it was harder to connect components to a Micro\:bit directly and the expansion board we used had problems driving the servo motor consistently


## 3.2 Open Challenge Algorithm

We started by defining a Frames class which 

- encapsulated a Region of Interest (ROI) 
- Initialization included defining the boundary coordinates of the ROI and the colors it must be able to detect
- Had functions to return all the colored pixels matching a specific color range in HSV, return contours by matching adjacent colored pixels, return the biggest contour, return the area of all contours

### 3.2.1 Iteration 1

We spent a lot of time in open challenge iterations since the idea was to have a solid foundation for obstacle challenge. The frames class and the state transitions must be reusable for obstacle challenge where needed
Iteration 1 had 2 distinct states

- STRAIGHT - Drive the robot straight following the walls
- TURN - When a colored line is seen, check if it is safe to turn and perform a timed turn

**MAIN LOOP**

<img width="1167" height="607" alt="image" src="https://github.com/shadyahajela/WRO-2026-FE-SN/blob/main/v-photos/camera%20images/open%20challenge%20iteration%201.png" />

- Two rectangular ROIs placed on the left and right sides of the frame, each able to detect black colored pixels
  - Keep a record of the number of black pixels detected within each ROI which represent how much of the inner and outer walls are visible to the camera on each side
- One rectangular ROI placed on the bottom of the frame to detect orange and blue colored pixels > 200 (to avoid detecting noise)
  - If Orange was detected first, set direction to clockwise (CW) for the robot to travel
  - If Blue was detected, set direction to counter-clockwise (CCW)
  - As soon as a colored line is detected, since it may take several frames to pass over that line, use a timer to avoid reading the same line again
  - Keep track of number of orange and blue lines crossed
- If colored line was detected
  - If there is no wall in the direction of travel, initiate TURN state
- Else
  - Initiate STRAIGHT state
- If the number of lines passed > 12, stop the robot after a fixed time (depending on speed) to position it in the starting section
- Compute steering value, speed, a direction value (forward/reverse/stop, chosen by the Pi based on what the vehicle currently needs to do), the current lap-line count, and the current state are packed into a single message and sent over USB serial (19200 baud) to an Arduino Nano, which handles the physical motor and servo output

**STRAIGHT**
<br/>
- `Steering error` was calculated as `right-side black pixel count - left-side black pixel count`
- If `Steering error` was positive 
  - This means more wall visible on the left 
  - Vehicle should steer right
- If `Steering error` was negative
  - This means more wall visible on the right
  - Vehicle should steer left
- The steering should be proportional to the error, so a simple proportional (P) controller was used meaning the steering angle was simply the error multiplied by a constant gain value (KP),  the further off-center the pixel counts were, the harder the robot steered
  - KP constant is simply calculated by scaling the error down to the range of the steering margin (45)
  - In this case, the error was in the range of 6000 pixels nominally
  - So we started with a KP of 30 / 6000 = 0.005 and then tuned it from there
- In summary `Steering angle = Steering error * kp`

**TURN**

- If a `wall missing` condition is detected in the direction of turn, turn steering hard for a fixed amount of time.
  - No feedback mechanism was used for the turn 
- The time was tuned until the turn completed properly 

**Observations and improvements made**

- Problem: Bright lighting conditions caused the pixel detection to be fuzzy
  - **Idea implemented**: Lowered the camera angle by about 8 degrees so as to catch less of ambient light while still being able to see the walls
- Problem: It would be ideal for a vehicle to travel closer to the inner wall to save time. However, if the algorithm just measured raw pixels on either side, a robot pointed straight but traveling closer to one wall would report a large error even though no correction was actually needed.
  - **Idea implemented**: Add an offset to the pixel count on the outer perimeter to guide the robot to run close to the inner wall
  - **Idea for next iteration**: Alternatively, derive the wall geometry/position rather than raw pixel volume, so the vehicle would travel straight regardless of the robot's exact distance from a wall or the lighting conditions on a given day
- Problem: While a simple proportional (P) controller was mostly sufficient, the robot steering wasn’t as smooth as we wanted it to be, especially after completing a turn. 
  - **Idea for next iteration**: Add a Derivative (D) to the controller which would account for the rate of change of error to damp out sudden changes in steering and make for smoother steering

### 3.2.2 Iteration 2

**STRAIGHT**

<img width="1167" height="607" alt="image" src="https://github.com/shadyahajela/WRO-2026-FE-SN/blob/main/v-photos/camera%20images/open%20challenge%20iteration%202%20and%203.png" />

- Replaced the two side ROIs with a single thin horizontal ROI band closer spanning the full width of the frame. This ROI was placed just at the vertical center so the algorithm can sufficiently look ahead to calculate steering
- Scans inward from each edge of the ROI to find the nearest wall pixel on the left half and the nearest wall pixel on the right half, on the same row, marking each as a boundary point
- If a wall drops out of frame, the boundary point is simply the far edge of the ROI on that side
- The midpoint between the two boundary points is the "path center"
- The horizontal offset between this path center and the frame's true horizontal center is the steering error
- A small dead zone (±20px) is applied around the centered wall-following error, errors inside this band are treated as zero, preventing constant tiny steering corrections once the robot is already well-centered

* Added a derivative term to the corridor-center error, moving from P to PD control: `control_signal = (kp × error) + (kd × (error − previous_error))`
  - The derivative term reacts to how quickly the error is changing frame to frame, which damps the overcorrection behavior from Iteration 1 and produces a visibly smoother line, especially noticeable at higher driving speeds where a pure-P response tends to overreact to small changes

**TURN**

- A turn only actually triggers after the line-detected + wall-missing condition holds continuously for a short confirmation window (0.15s), rather than on the first frame it's seen. This prevents the robot from running into corners

**Observations and improvements made**

- Problem: Timed turns still needed per-corner retuning, since the inner wall's length and shape vary between corners depending on the round's randomized wall configuration,  a duration tuned for one corner would overshoot or undershoot the next. Turn duration also drifted over the course of a run as battery voltage sagged and motor speed dropped slightly, or if the wheels lost grip occasionally. Duration of turn had to be adjusted every time we had to change the speed of the robot for testing
  - **Idea for next iteration**: Use a feedback mechanism to detect when a turn ends and implement a PD controller

### 3.2.3 Iteration 3 (Current)

**MAIN LOOP**

- Added an IMU (BNO055) to track heading (0–360°)
  - Note: We built a utility program [bno055.py](https://github.com/shadyahajela/WRO-2026-FE-SN/blob/main/src/sensor/bno055.py) to calibrate and test the BNO055 IMU sensor. The IMU integration was also tested on the table-top test bench and tuned before the robot was tested on the field
- Reset heading to 0 at startup
- Each time a colored line is passed, a turn counter increments, and the target heading is set as an **absolute** multiple of 90°,  `turn_count × 90°` (mod 360°, sign flipped for CCW),  rather than the previous target plus 90°. This is to ensure small heading errors from one turn can't accumulate and drift into the next turn's target

**TURN**

- `Turn error` is the shortest signed angular distance between target and current heading (normalized to −180°…180°, so the robot always turns the short way)
- `Steering value = turn error * kp_turn`

- `kp_turn` is the proportional gain constant for  proportional (P) control, consistent with the steering approach used elsewhere in the system
- The turn is considered complete once heading error is within 2°, at which point the robot returns to STRAIGHT and resets its tracking flags

## 3.3 Obstacle Challenge Algorithm

**Pass rule:** green blocks are passed on the left, red blocks are passed on the right.

### 3.3.1 Iteration 1 - Treating Obstacles as Virtual Walls

**Approach**

- Reused the Open Challenge STRAIGHT corridor-centering algorithm rather than building separate obstacle-handling logic
- Red is passed on the right, so a detected red block sits to the robot's left as it passes,  its bottom-right corner (the edge nearest the robot's path) was fed in as a substitute **left-wall** boundary point
- Green is passed on the left, so a detected green block sits to the robot's right as it passes,  its bottom-left corner was fed in as a substitute **right-wall** boundary point
- The corridor-center calculation then treated that corner exactly like a real wall-edge point, steering around it the same way it would steer around a wall

**Issues**

- The corridor-centering math was tuned for two continuous, roughly parallel wall surfaces,  a single point from a small discrete block gave a much shakier corridor estimate, since the "wall" on that side was only a few centimeters wide instead of an extended surface
- With only one corner representing the whole block, the estimated boundary jumped noticeably frame to frame as viewing angle changed on approach, so steering reacted inconsistently to what should have been a smooth approach
- No explicit sense of distance to the block,  correction strength was identical whether the block was far away or dangerously close, since wall-following gains (tuned for gradual convergence) were being reused for what should've been a more urgent, close-range maneuver
- Blocks near a real corner or wall created ambiguity about which "wall" the algorithm was actually reacting to, occasionally steering toward the block instead of away from it

**Why We Improved**

- Needed obstacle avoidance treated as its own problem with its own reference points, not a block disguised as a wall
- Needed distance to the block to explicitly scale the strength of the correction, not just its direction

### 3.3.2 Iteration 2,  Dual-Axis "Pulling" Control

<img width="400" alt="image" src="https://github.com/shadyahajela/WRO-2026-FE-SN/blob/main/v-photos/camera%20images/obsg.png" />
<img width="400" alt="image" src="https://github.com/shadyahajela/WRO-2026-FE-SN/blob/main/v-photos/camera%20images/obsr.png" />

**Approach**

- Green block: reference dot at the block's bottom-left corner, fixed target dot at the bottom-right corner of the screen
- Red block: mirrored,  reference dot at the block's bottom-right corner, fixed target dot at the bottom-left corner of the screen
- Horizontal (x) offset between reference and target dots drives a PD controller,  correction is added to center for red (steers right) and subtracted for green (steers left)
- Vertical (y) offset doesn't drive steering directly,  it produces a distance falloff factor that scales the whole PD correction: strongest as the block's bottom edge nears the bottom of the frame (close), fading toward zero as it sits higher in the frame (far). This is exactly what Iteration 1 was missing
- Overall effect is like an elastic band strung between the two dots,  as the robot approaches, the "pull" tightens and the correction strengthens, instead of a constant-strength nudge regardless of distance
- Falls back to the same corridor-centering wall-following logic from Open Challenge whenever no block is visible, but with its own separately tuned gains,  Obstacle Challenge's field and required correction strength weren't identical to Open Challenge's, so tuning each independently gave a tighter fit for both
- If both a red and green block are visible in the same frame, only the physically nearer one (by vertical offset) is used to compute steering that frame,  the farther block is detected but ignored until it becomes the closer one
- Steering output is clamped to the same safe range used everywhere else in the system

**Issues**

- Inherited the same timed-turn unreliability as Open Challenge Iteration 2,  turn duration needed per-corner retuning and drifted with battery voltage and wheel grip over a run

**Why We Improved**

- Wanted the same closed-loop, drift-independent turning fix already validated in Open Challenge Iteration 3

### 3.3.3 Iteration 3 (Current),  IMU Heading-Based Turns

**Approach**

- Kept the Iteration 2 dual-axis pulling logic and wall-following fallback unchanged
- Replaced the fixed-duration turn with the same IMU heading-based approach as Open Challenge,  absolute cardinal target headings, incremented 90° per corner, closed-loop control to close the heading error, exiting once within tolerance rather than after a fixed time
- Mid-turn bail-out preserved: if a red or green block becomes visible above threshold while still turning, the robot exits the turn immediately and hands back to obstacle avoidance that same frame,  reacting to a visible block takes priority over finishing a scheduled turn

**Issues Resolved**

- Turning no longer depends on constant motor speed, wheel grip, or battery voltage, and generalizes across corner shapes without retuning
- No fixed duration to separately tune for Obstacle Challenge's corner geometry, since heading is now the closing condition

### Obstacle-Clear Behavior

No explicit "cleared" trigger exists,  obstacle avoidance is recomputed fresh every single frame based on whether a red or green block is currently visible above the detection threshold. The moment a block's contour area drops below that threshold,  out of frame, passed, or occluded,  the very next frame simply falls through to wall-following instead, with no dedicated timer or debounce needed for the handoff.

### Corner Safety Layer

<img width="400" alt="image" src="https://github.com/shadyahajela/WRO-2026-FE-SN/blob/main/v-photos/camera%20images/obstacle%20edge%20case%20detection.png" />

- Independent of whichever behavior is currently driving steering, four small ROIs step diagonally in from each top corner of the frame, checking how "full" of wall color they are
- A fully filled side nudges steering away from it more strongly than a partially filled reading; this correction is added on top of the frame's already-computed steering value, before the final clamp
- Runs every frame except while parked, acting as a standing safety margin against cutting a corner too tightly, on top of and independent from the primary steering decision

### 3.3.4 State Machine

Same two core behaviors as Open Challenge (wall-following and turning), with additional states layered in for obstacles, wrong-side blocks, and parking:

- **LEAVE**,  starting state; drives straight out of the parking bay, then hands off to normal driving once clear
- **STRAIGHT**,  corridor-centering wall-following; active by default whenever no red/green block is visible
- **OBSTACLE**,  dual-axis pulling control; active by default whenever a red or green block is visible
- **TURNING**,  IMU heading-based turn triggered at a lap line; exits early back to OBSTACLE if a block becomes visible mid-turn, otherwise exits to STRAIGHT once heading target is reached
- **OBS\_CRITICAL**,  triggered by spotting a "wrong-side" block early, near an upcoming turn; holds a near-straight course via IMU-held heading until a threshold amount of wall fills the center of the frame, then forces a hard turn for a fixed duration to physically clear the block, then resumes normal driving
- **BETWEEN**,  entered after an OBS\_CRITICAL sequence completes near the end of the run; treats every remaining block as if it were red regardless of actual color, and watches for the parking wall to appear
- **PARK**,  final state; speed and steering held at rest

### 3.3.5 PARKING

Parking state initiates when the 12th floor line is read. Parking is achieved by 5 distinct maneuvers explained below

1. Continue straight (IMU guided) until a small ROI detect the wall ahead to stop the robot at a fixed position 
2. Turn 90 degrees (IMU guided) in the direction of the parking blocks
3. Continue straight ahead towards the parking blocks, by following a fixed offset against the walls until the robot positions against the first parking block
4. Depending on the direction, move forward or backward straight until the robot positions itself ahead of the second parking block. This requires the camera to detect the second parking block in an ROI on the side
5. Execute a timed reverse double turn into the parking area similar to we perform parallel parking in real life

<table>
  <tr>
    <td align="center"><strong>Parking clockwise</strong><img src="https://github.com/shadyahajela/WRO-2026-FE-SN/blob/main/v-photos/camera%20images/Clockwise.png" width="2500"></td>
  </tr>
  <tr>
    <td align="center"><strong>Parking counter-clockwise</strong><img src="https://github.com/shadyahajela/WRO-2026-FE-SN/blob/main/v-photos/camera%20images/Counter%20Clockwise.png" width="2500"></td>
  </tr>
</table>

## Microcontroller Code (Arduino Nano)

**Role**

- The Arduino Nano is the low-level hardware controller: it receives one serial command per frame from the Raspberry Pi and converts it into actual electrical signals for the drive motor, steering servo, an OLED status display, and a NeoPixel LED strip
- The Pi handles all vision processing and decision-making, including which direction the vehicle should currently be driving; the Arduino has no awareness of *why* a command was sent, only how to execute it
- We built a utility program [pi2nano_bi_test.py](https://github.com/shadyahajela/WRO-2026-FE-SN/blob/main/src/test/pi2nano_bi_test.py) to rigorously test the transmission of robot control data (servo angle and steering, newline \n delimited) to Arduino over serial connection. This program helped us identify that 30 millisecs is the minimum time between the Pi transmitting control data over USB serial connection and for the Arduino to finish processing it. If the data transfer is any faster (irrespective of whether the standard USB or USB 3 port was used), Arduino won’t be ready to receive the next byte stream over serial connection - even with non-blocking code. This would cause the outgoing message flush to fail on the Pi and if left unhandled, it would kill the challenge program

**Communication Protocol**

- Listens on serial at 19200 baud, matching the Pi's configured rate
- Reads one line at a time into a fixed 20-byte buffer via `readBytesUntil('\n', ...)`, null-terminating it once a full message arrives,  the fixed buffer size is a deliberate guard against uncontrolled memory allocation on a small microcontroller
- Expected message format: `steering speed direction lineCount state` (e.g. `110 255 1 12 OPEN`), parsed in one call with `sscanf`
- `direction` (0 = stop, 1 = forward, 2 = reverse) is actively chosen by the Pi each frame based on what the vehicle currently needs to do, not a fixed constant,  the Arduino simply executes whatever direction it's told
- The Arduino also talks back to the Pi: when the physical start button is pressed, it sends a plain `"START"` line over the same serial connection, with a short debounce delay

**Steering & Motor Control**

- Before applying anything, the received `servo` value is range-checked (`30 < servo < 160`),  if it falls outside this window, the entire steering + motor update is skipped for that frame, acting as a basic sanity check against corrupted or garbled serial data rather than driving on a bad command
- If valid, steering is applied first (`myservo.write(servo)`), then motor output, so both actuators respond to values from the same command packet rather than a mix of an old and new one
- Motor control goes through the L298N driver: `speed` (0–255) is written as a PWM duty cycle via `analogWrite`, while `direction` sets two digital direction pins,  forward and reverse drive the pins in opposite states, and stop (or an out-of-range `direction` value) sets both pins low, cutting drive entirely regardless of speed

**Status Display (OLED)**

- A 128×64 monochrome OLED (SH1106 driver, addressed over I²C) shows live telemetry every frame: current steering angle, line/lap count, motor speed, the current challenge state string, and a directional indicator (`>>` / `<<` / `XX`) showing forward, reverse, or stopped,  laid out in a fixed grid with divider lines, intended for operators to diagnose communication or control issues during testing at a glance

**LED Strip Status Indicator**

- An 8-pixel NeoPixel strip gives an at-a-glance visual state readout without needing to read the OLED up close: intended to show white for the OPEN challenge state and red otherwise
- **Known issue:** the state check currently compares the character array directly (`state == "OPEN"`) rather than with `strcmp()`, which compares memory addresses rather than string contents in C/C++ and will essentially never evaluate true,  the code already carries a comment flagging the correct fix, but it hasn't been applied yet, so in practice the LED likely always shows red regardless of actual state
- A full rainbow sweep runs once at boot as a power-on self-test, and both end pixels turn green once initialization completes, signaling the robot is ready to start

**Startup Sequence**

- On boot: configure pins, attach the servo, open serial, initialize the OLED and show a "BOOTING" message, initialize the LED strip, sweep the servo through a small range as a functional check before returning it to its calibrated center, run the rainbow self-test, then display "-READY-" and light the ready-indicator LEDs

# 4. 3D Model Files

## 4.1 Onshape CAD

We used Onshape to design the 3D models used to make the robot. The files can be found here (hyperlink).

## 4.2 STL Files

Below are the the STL files which are used in building the robot, which can be downloaded and 3D printed. Refer to detailed [step-by-step instructions](Build_Instructions.md) to reproduce the chassis of the robot and assemble all the components on it.  

**Chassis and Core Structure**

- [`BasePlateV8.stl`](mech/3D_CAD_Files/BasePlateV8.stl)

**Steering Linkages**

- [`AckermannLRodV3.stl`](mech/3D_CAD_Files/AckermannLRodV3.stl)
- [`AckermannSRodV2.stl`](mech/3D_CAD_Files/AckermannSRodV2.stl)
- [`DirectionTabsLH.stl`](mech/3D_CAD_Files/DirectionTabsLH.stl)
- [`DirectionTabsRH.stl`](mech/3D_CAD_Files/DirectionTabsRH.stl)
- [`AckermannHexaRod.stl`](mech/3D_CAD_Files/AckermannHexaRod.stl)

**Motor and Transmission**

- [`DifferentialHousingV4.stl`](mech/3D_CAD_Files/DifferentialHousingV4.stl)

**Wheel and Axle Components**

- [`Axle Spacer.stl`](mech/3D_CAD_Files/Axle_Spacer.stl)

**Mounting Components**

- [`CameraMountV3.stl`](mech/3D_CAD_Files/CameraMountV3.stl)
- [`MidPlateV2.stl`](mech/3D_CAD_Files/MidPlateV2.stl)
- [`PiPlateV3.stl`](mech/3D_CAD_Files/PiPlateV3.stl)
* Servo Box Mount (link)

**Coupling Parts**

- [`MotorDifferentialCoupling.stl`](mech/3D_CAD_Files/MotorDifferentialCoupling.stl)
- [`DifferentialAxleCoupling.stl`](mech/3D_CAD_Files/DifferentialAxleCoupling.stl)
* Servo-Axle Coupling (link)

**Miscellaneous**

- [`GuideArrowV3.stl`](mech/3D_CAD_Files/GuideArrowV3.stl)
- [`Battery_Holder.stl`](mech/3D_CAD_Files/Battery_Holder.stl)

