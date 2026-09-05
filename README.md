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

We are Team Sub-Atomica, Niva, Jeevesh and Shadya. We are united by a passion for STEM, Electronics, coding and jalapeño poppers. We wanted to bring our jouney to many others with the same passions and inspire teenagers around the world. To us its more than just coding or building, its the experiences and connections along the way.

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

To build a self-driving car we need to build a car.

Objectives:

As per the rules, the car must have a steering mechanism and the rear axle driven by a motor.
Our team also wanted the car to be stable and fast (and colorful if we can help it).

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

 **Selection Reasoning** 

Built-in encoder, allowing for precise movement and less wiring hassle.
High torque for pushing the robot with vigor and avoiding stalling.
The JGA25-371 motor was selected because it combines high torque with integrated encoder feedback, providing both the power and precision required for the WRO Future Engineers competition. The motor drives the rear wheels through a metal WLTOYS 144001 differential, which allows the robot to maintain smooth and balanced turns while reducing the difference in rotational speed between the left and right wheels. By distributing torque between both LEGO SPIKE Prime wheels, the differential minimizes wheel slip and improves stability during cornering. The integrated Hall encoder provides real-time feedback for closed-loop control, allowing the robot to accurately measure wheel rotation, regulate speed, and maintain consistent movement throughout each run. Unlike smaller motors, the JGA25-371 provides sufficient torque to support the drivetrain while maintaining reliable performance under varying loads. The combination of encoder feedback, the metal differential, and the SPIKE Prime wheels creates a drivetrain that balances power, stability, and precision, making it well-suited for the competition environment.*
<br/>

***<Naga - Above is good enough. Perhaps convert to jot notes to clearly indicate the reasons it was selected. Primarily because the 620RPM would allow for quick laps around the circuit at full speed (fastest lap was 16 seconds) and the 0.22kg.cm torque allows to move our 800gm robot slow enough to achieve parking.The integrated Hall encoder provides real-time feedback for closed-loop control, allowing the robot to accurately measure wheel rotation, regulate speed, and maintain consistent movement especially during parking>***
  
  **Selection Reasoning:**
  The JGA25-371 was selected for its high torque, compact size, reliability, and integrated Hall-effect encoder. Its torque provides sufficient force under varying loads while reducing the likelihood of stalling, and the encoder provides rotational feedback for precise movement. Compared with smaller motors, it adds some weight and increases the possibility of wheel slip, but we accepted this tradeoff because torque and controllability were more important to our drivetrain requirements.

The motor drives the rear wheels through a metal WLTOYS 144001 differential, which distributes torque while allowing the left and right LEGO SPIKE Prime wheels to rotate at different speeds during turns. This reduces drivetrain binding and tire scrub while providing a durable connection capable of handling the motor's torque. The motor, differential, and wheels were therefore selected as one system to balance power, traction, and turning consistency.

The encoder is integrated into the vehicle's closed-loop control system through the Arduino Nano and motor driver. It measures wheel rotation so the controller can regulate speed and distance, improving repeatability between runs and allowing the system to respond to changes in load. To manage risks such as wheel slip, mechanical backlash, and motor stress, motor acceleration and speed are controlled in software and the drivetrain is calibrated using encoder feedback. This integration allows the mechanical and software systems to work together to produce controlled and repeatable motion.

  **Differential**
*We used a metal WLTOYS 144001 differential in our robot, which allows the left and right rear wheels to rotate at different speeds while turning, reducing tire drag and improving turning efficiency. This setup provides smoother and more controlled cornering by distributing torque between both rear wheels and compensating for the difference in the distance each wheel travels during a turn. The differential improves maneuverability and stability, particularly during the obstacle challenge and parallel parking, where precise and repeatable movements are essential.*

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

  IMAGE

  * Wires connected to Arduino Nano and Motor Driver.
  * Spike Prime wheels fitted onto the LEGO motor axle.


  **Considerations:**
  An alternative would be a NEMA 17 stepper motor, which can provide precise speed and position control. However, the NEMA 17 is larger and heavier, and would require a more complex driver setup, making it less suitable for our robot. The JGA25-371 was chosen instead because it provides sufficient speed and torque while being more compact and easier to integrate.
  
## 1.2 Steering System

**Steering:** Initially Parallel Steering, then changed to Ackermann Steering Geometry, prototyped with LEGO technic parts and later made with 3D printed parts.
  insert images of prototypes

**Steering Motor:** We chose the MG90S Micro Servo for precise steering and weight reduction. The compact size and PWM interface make the MG90S easy to integrate and control using the Arduino Nano. It provides sufficient torque to steer the front wheels accurately and responsively. Its fast response and metal gear construction provide reliable and stable steering during turns and lane changes. The MG90S is widely used in hobby robotics, making replacement parts, mounting hardware, and documentation readily available.

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

Although Ackerman geometry is more complex to implement, we believe its advantages are especially important for obstacle navigation and parking, where precise control and a small turning radius are essential. We also considered parallel steering because it was simpler to implement, but chose Ackermann as a calculated risk for its improved maneuverability.

Our implementation uses a custom 3D-printed Ackermann steering mechanism, with the following key design considerations:
We used Onshape to experiment with different pivot points, linkage positions, and steering angles throughout the design process.

Through 2 iterations, we shortened both rods and increased the length of the side rods to achieve a higher turning radius while keeping the mechanism       compact. In our first iteration we made our tabs 2mm longer and the neck of the robot, this gave us a 15 degree increase in our turning radius. We also experimented with shortening the rods to make our wheels streamlined and finalized with 9.53cm for the long shaft and 8.5cm for the short one. 
  
A major constraint was the chassis neck, which limited the steering angle because the wheels could collide with it. We therefore cut part of the chassis around the steering area to provide sufficient clearance.

Since achieving perfect Ackermann geometry at our robot's small scale is difficult, we focused on achieving a practical approximation with a wide steering angle, smooth turns, and minimal wheel slip.

The MG90S servo horn and steering angles were adjusted directly in Onshape before 3D printing and physical testing to reduce the risk of overloading the servo or having the wheels contact the chassis.

  


<img src="v-photos/ackerman steering.png" alt="Ackerman Steering">

*This was an early prototype of the Ackerman Steering model before we added it to the first iteration of our Lego car chassis.*

*Although this steering geometry is more complex to implement, we believe its advantages are especially important for obstacle navigation and parking, where precise control and a small turning radius are essential. It allows the robot to maneuver smoothly and maintain accurate alignment in tighter spaces. Our implementation uses a custom 3D-printed Ackermann steering mechanism, with the following key design considerations: We used Onshape to experiment with different pivot points, linkage positions, and steering angles throughout the design process. Since achieving perfect Ackermann geometry at the robot's small scale is difficult, we aimed to closely approximate the ideal geometry through multiple design iterations. The MG90S servo horn and steering angles were adjusted directly in Onshape before 3D printing and physical testing.*
  


**Calibration and Implementation:**
To achieve accurate and consistent steering, we used a combination of CAD adjustments and physical testing:
* Different servo positions, linkage lengths, and steering angles were tested in Onshape to determine the most suitable configuration.
* The physical steering assembly was then tested by turning the wheels fully in both directions and checking for smooth, consistent movement.
* Based on the test results, we modified the CAD model and repeated the process until the steering geometry provided the desired range of motion and wheel alignment.
* The final Onshape design was then used to produce the 3D-printed steering assembly.

**Mounting:**

* Screwed into a platform plate in front of the chassis, connected to the steering mechanism.

image

**Considerations:**
An alternative would be the MG996R servo, which provides higher torque and more durable metal gears. However, it is larger and requires more power than the MG90S, which would require modifications to our chassis and power system. The MG90S was chosen instead because it is more compact and sufficient for our steering system.

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

The power system is essential to the robot’s ability to operate reliably throughout the competition. Our robot is powered by a 3-cell (3S) Lithium Polymer (LiPo) battery. The battery provides a nominal voltage of 11.1 V and a fully charged voltage of 12.6 V. Its high discharge capability allows it to supply sufficient current to the motors and other components. The battery’s high energy density provides a good balance between runtime, weight, and available power, making it well suited for a mobile competition robot.

  The JGA25-371 motors operate from the 12V supply provided by the battery, while the L298N motor driver regulates the power delivered to the motors based on commands from the Arduino Nano. The higher voltage of the 3S battery allows the motors to operate without requiring a step-up converter, simplifying the power system and reducing unnecessary conversion losses. A step-down converter is used to reduce the battery voltage to the appropriate levels required by the Raspberry Pi 5, Arduino Nano, camera, IMU, and other electronics. This separates the high-current motor supply from the lower-voltage electronics while allowing the entire robot to be powered from a single battery.

  The Raspberry Pi 5 serves as the vehicle’s main processing unit and requires a stable 5V supply. The step-down converter provides the required regulated voltage, ensuring that the Raspberry Pi and other electronics receive consistent power during operation. This setup allows the robot to run its sensors, process camera data, control the motors, and make navigation decisions without relying on separate batteries for each subsystem.

  IMAGE OF BATTERY ON ROBOT

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

  We chose the L298N motor driver because it provides reliable control of our JGA25-371 DC motors while being compatible with the Arduino Nano. It supports the 12V supply from our 3-cell LiPo battery and allows the Arduino to control motor direction and speed through its PWM inputs. This provides a simple and reliable connection between the robot’s power system and drivetrain, making it well suited for our differential drive system.


OLD MOTOR DRIVER IMAGE: DRV8871

<img width="140" height="92.7" alt="image" src="https://github.com/user-attachments/assets/1ebcd3af-6231-4427-b338-a13d668f537a" />


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

160° wide field of view captures more of the course, reducing blind spots and improving environmental awareness.
5MP resolution provides enough detail to reliably distinguish course features and colored markers.
1080p at 30 FPS provides responsive visual data for real-time image processing and object detection.
Compact and lightweight design reduces its impact on vehicle weight and balance while allowing flexible mounting.

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

Considering we wanted to have a relative compass to reduce additional code in the Arduino, we chose the BNO055. The BNO055 is a compact 9-axis IMU that combines a 3-axis accelerometer, 3-axis gyroscope, and 3-axis magnetometer with an onboard processor for sensor fusion. It provides orientation data such as heading, Euler angles, linear acceleration, and gravity direction, reducing the need for complex external calculations.
    
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

  *Subsystem Integration:*

  The Arduino Nano was selected as the robot’s low-level controller to handle time-sensitive tasks such as motor control, steering, and encoder/sensor input. This architecture separates high-level decision-making from low-level control, allowing the Raspberry Pi 5 to focus on vision processing, navigation, and path planning while the Arduino maintains consistent motor and steering responses.

  *Hardware and Software Architechture:*

  The Arduino Nano communicates with the Raspberry Pi 5 through a serial connection, creating a modular hardware and software architecture. This reduced the risk of timing conflicts caused by running motor-control tasks alongside computationally intensive processes on the Raspberry Pi. It also allows each subsystem to be tested independently, making troubleshooting and future modifications easier.

  *Iterations and Trade-Offs:*

  Our initial design used a microbit, but research showed that the Arduino Nano offered greater flexibility for external hardware and simpler bidirectional communication with the Raspberry Pi 5. We therefore changed to the Arduino Nano, trading the microbit’s built-in buttons and LEDs for improved subsystem integration and expandability.

  The control system was then iteratively tested and adjusted to improve motor response, steering accuracy, and communication reliability. Using a dedicated microcontroller introduced additional hardware and communication complexity, but provided more predictable real-time control and reduced the dependence of critical drivetrain functions on the Raspberry Pi’s processing workload.

  *Risk Analysis and Reliability:*

  This subsystem architecture improves reliability by isolating critical low-level control from higher-level software. If the vision or navigation software requires significant processing, the Arduino can continue executing motor and steering commands without relying on continuous high-level computation. This separation therefore supports more consistent movement and provides a more robust architecture for the challenges defined by WRO Future Engineers.
  

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
  


###  Power System Architecture

<img width="1167" height="607" alt="image" src="https://github.com/user-attachments/assets/c2637fd0-5c40-4e5f-b6e0-8fb43fe7991f" />


# 3. Software Architecture

# 4. Source Code

# 5. List of Components

| Major components                                                 | Reference Cost in CAD<br>\- As of Sep 1, 2026<br>\- Taxes not included | Purchase Link for Future Reference                                                            |
| ---------------------------------------------------------------- | ---------------------------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| Raspberry Pi 5 4GB                                               | $153.95                                                                | [pishop.ca](https://www.pishop.ca/product/raspberry-pi-5-4gb)                                 |
| Arduino Nano                                                     | $27.19                                                                 | [Amazon.ca](https://www.amazon.ca/dp/B0F6Y7GS4Q)                                              |
| Arduino Nano Expansion Board                                     | $13.99                                                                 | [Amazon.ca](https://www.amazon.ca/dp/B08198MSJ2)                                              |
| 1.3" OLED display module SH1106 128x64                           | $5.33                                                                  | [aliexpress.com](https://www.aliexpress.com/item/1005006827988792.html)                       |
| Sainsmart Wideangle 5MP Camera 160 degree FoV (OV5647 sensor)    | $12.91                                                                 | [Amazon.ca](https://www.amazon.ca/SainSmart-Fish-Eye-Camera-Raspberry-Arduino/dp/B00N1YJKFS)  |
| Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 | $38.38                                                                 | [Aliexpress.com](https://www.aliexpress.com/item/1005010734176030.html)                       |
| VL53L0X TOF sensor                                               | $13.99                                                                 | [Amazon.ca](https://www.amazon.ca/dp/B0F1MRW55R)                                              |
| 12V Battery                                                      | $27.99                                                                 | [Amazon.ca](https://www.amazon.ca/12V-2800mAh-Rechargeable-Battery-Replacement/dp/B0FJ27ZT28) |
| 12V to 5V Step-down converter (XL4015 with Display)              | $4.80                                                                  | [aliexpress.com](https://www.aliexpress.com/item/1005008401247033.html)                       |
| JGA25-371 620 rpm 12V Brushless DC motor with Encoder            | $12.78                                                                 | [aliexpress.com](https://www.aliexpress.com/item/1005007546764319.html)                       |
| WItoys 144010 Metal Differential Gearbox                         | $40.58                                                                 | [aliexpress.com](https://www.aliexpress.com/item/1005005869550963.html)                       |
| L298N Motor Driver                                               | $11.99                                                                 | [Amazon.ca](https://www.amazon.ca/dp/B0D8G2PZBB)                                              |
| MG90S Servo motors                                               | $17.02                                                                 | [Amazon.ca](https://www.amazon.ca/dp/B0BWJ4RKGV)                                              |
| Short USB to USB-C cable                                         | $11.39                                                                 | [Amazon.ca](https://www.amazon.ca/dp/B0DG8J6S1G)                                              |
| Rocker Switch                                                    | $2.65                                                                  | [aliexpress.com](https://www.aliexpress.com/item/1005007044175800.html)                       |
| Push Button breakout module                                      | $1.52                                                                  | [aliexpress.com](https://www.aliexpress.com/item/32820437436.html)                            |
| RGB LED SMD Module                                               | $2.03                                                                  | [aliexpress.com](https://www.aliexpress.com/item/1005006764822374.html)                       |
| Lego wheels, axles and connectors                                | $50.00                                                                 | Approximately                                                                                 |
| 3D printed parts for the chassis                                 | $25.00                                                                 | Approximately for one spool of PLA filament                                                   |
| Total                                                            | $473.49                                                                |                                                                                               |

# 7. 3D Model Files

## 7.1 Onshape CAD Files

We used Onshape to design the 3D models used to make the robot. The files can be found here (hyperlink).

* Assembly Files (hyperlink) - contains the full robot assembly including the part placements.
* Individual Parts (hyperlink) - includes each 3D component separately.

## 7.2 STL Files

**Chassis and Core Structure**

* Main chassis (link)

**Steering Linkages**

* Ackermann Steering Long Rod (link)
* Ackermann Steering Short Rod (link)
* Ackermann Steering Left Tab (link)
* Ackermann Steering Right Tab (link)
* Ackermann Steering Hexagonal Rod (link)

**Motor and Transmission**

* Differential Housing (link)

**Wheel and Axle Components**

* Axle Spacer (link)

**Mounting Components**

* Camera and Servo Mount (link)
* Middle Mount Plate (link)
* Raspberry Pi Mount Plate (link)
* Servo Box Mount (link)

**Coupling Parts**

* Motor-Differential Coupling (link)
* Differential-Axle Coupling (link)

**Miscellaneous**

* Guide Arrow (link)

## 7.3 Slicer Files

All slicer project files (.3mf) used for printing the robot’s components can be found here.
These files contain optimized slicing settings such as layer height, infill, support, and print orientation for each part.

bjsihgiwobwe


# 8. Building Instructions

Step 0: Print the 3D parts

Before assembly, prepare the components listed above and print the 3D parts.

<details>

<summary> <b><span style="font-size:1.1em; background-color:#f2f2f2; padding:4px 8px; border-radius:5px;">Click here to show the 3D printed parts used in the building process</span></b></summary>

</summary>

| Part Name | Quantity |
| ---------------------------- | -------- |
| Axle Holder | 3 |
| Back Wheel Axle | 2 |
| Back Wheel Connector | 2 |
| Back Wheel Stopper | 2 |
| Chassis | 1 |
| Front Cover | 1 |
| Front Wheel Axle (Left) | 1 |
| Front Wheel Axle (Right) | 1 |
| Front Wheel Stopper | 2 |
| Lidar Plate | 1 |
| Motor Gear | 1 |
| Motor Holder | 1 |
| Motor Plate | 1 |
| T-Bone Linkage (Bottom) | 1 |
| T-Bone Linkage (Top) | 1 |
| Transfer Linkage (Left) | 1 |
| Transfer Linkage (Right) | 1 |
| Wheel Linkage (Bottom Left) | 1 |
| Wheel Linkage (Bottom Right) | 1 |
| Wheel Linkage (Top Left) | 1 |
| Wheel Linkage (Top Right) | 1 |

</details>

**Step 1: Assemble the steering system**

  1. On the edges of the long ackermann steering rod, attach the left and right ackermann tabs with _______ inch VEX screws and lock nuts, positioned such that the tabs are curved inward.



  2. Attach the short ackermann steering rod on the same plane as the long ackermann steering rod, with the same 2 screws and nuts.



  3. Mount the long ackermann steering rod onto the front of the chassis using two ______ inch VEX screws and lock nuts, positioned such that the short ackermann rod is facing the back of the chassis and the ackermann tabs are on a lower plane than the long and short ackermann rods.



  4. Attach the two steering LEGO Spike Prime wheels onto the outsides of the ackermann tabs using ________________ LEGO piece.



  5. Mount the camera and servo mount onto the fixed long ackermann rod using __________ inch VEX screws and lock nuts.



  6. Attatch the servo box mount using _____ (#) ___________ inch VEX screws and lock nuts, angled such that the servo hole is positioned over the remaining hole in the long ackermann rod. Remember to keep space under the box for the hexagonal ackermann rod.



  7. Fit a _______ stud long LEGO axle into the servo-axle coupling. Thread the axle down through the servo box, through the hexagonal ackermann rod, and through the aforementioned hole. Use a _____ inch VEX screw and a lock nut to attach the other side of the hexagonal ackermann rod to the hole in the middle of the short ackermann rod.


  8. Screw in the MG90S servo into the servo box using a ______ M2 screw, with the servo fitted into the servo-axle coupling (MAKE SURE: before you fit the servo, the steering is straight, and the servo angle is 90 degrees. If by human error it is not at 90 degrees, use the servo calibration code provided here (LINK) to figure out the degree at which the servo is straight, which you can change in the main code.



**Step 2: Assemble the drivetrain and chassis body**

**Step 3: Mount electronics**

**Step 4: Upload the software**
