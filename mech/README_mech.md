# **Mechanical Design Process**
## **1. Rudimentary/Fundamental Mechanical Choices**

1.1 Wheel Choice

We chose Spike Prime wheels (56mm diameter, 14mm thickness) for their compactness, grip on vinyl, and familiarity. Larger wheels would make the robot harder to control and take too much space while smaller wheels would complicate the design and lower ground clearance. (PARAPHRASE!!!!!)

[insert image of Spike Prime wheel]

1.2 Steering System

**Prototype:** Our first prototype was a parallel steering system that used a servo mounted directly on the axle. It had issues with drifting on turns because both wheels were turning on the same radius, making their extension lines parallel so there is no instant center.
<img width="530" height="503" alt="image" src="https://github.com/user-attachments/assets/4d499e88-9f4a-4663-be12-de957adbe566" />

**Ackermann Steering:** We built an Ackermann Steering System to solve the drifting problem. An instant center is found where both wheels can rotate around with two different turn radii. The final iteration is 3D printed to accommodate the chassis base plate. The joints are joined using flat screws and locktight nuts, ensuring a tight fit while allowing full range of movement.
<img width="985" height="764" alt="image" src="https://github.com/user-attachments/assets/7f8faa13-2fd5-4230-8878-34856fd6f9d8" />


1.3 Rear Axle Differential

A physical differential composed of metal gears and casing allows the rear wheels to turn at different speeds to avoid wheel skipping when turning corners. Using a prioritization model, we concluded that printing our own gears would have a low Value vs. Complexity Ratio, and would not have provided such compact and smooth movement.

1.4 Dimensions Reasoning

Factoring in all the components we would need to fit on the chassis (Raspberry Pi 5, Arduino Nano, Motor, Step-Down Voltage Converter, ...), we decided to have a minimum width margin of approximately 30mm margin, meaning maximum 130mm. After testing multiple dimension iterations, we decided to make the robot about 230mm long and 120mm wide. We decided on these dimensions based on results of tests that focused on efficiently getting around tight edges, keeping balanced center of gravity (to avoid tipping and stay stable on sharp turns), and overall maneuverability. We had to take into account the large turning radius that a car with this length/width ratio has, similar to a go kart, and change our turning radius accordingly to leave minimal room when turning around pillars and corners.

## **2. Structural Prototypes**

+ **LEGO Prototype:** son
+ **Differential and Motor Positioning:** Motor positioned in the center of the base, axle connected to differential with custom coupling.
+ **Wheel Steering System:** Neck was made narrow to allow for greater steering range / turn radius.
+ **Base Layer:** Held drive motor, differential, servo, battery, and motor driver.
+ **Mid Plate:** Held Step-Down voltage converter and Arduino Nano.


## **3. Final 3D Printed Chassis**




## **4. Custom Mounts and Coupling**

+ Custom CAD designed holders for servo, raspberry pi, and camera

+ Motor coupler connects motor to differential
<img width="1773" height="1121" alt="image" src="https://github.com/user-attachments/assets/bc5ead5a-fb95-4a0e-95ba-731882625506" />

+ Servo coupler connects servo to Ackermann steering arm

+ Differential coupler connects servo to wheel axle
<img width="1986" height="1063" alt="image" src="https://github.com/user-attachments/assets/01cac487-a78d-4b40-b865-b69b3dcf6cbe" />

+ VEX standoffs, nuts, bolts, and locknuts
