# Team Paraducks | WRO Future Engineers 2024

## Introduction:
This repository provides all the information about the WRO Future Engineers 2024 Team Paraducks. The team members are Micquel Devlaliwalla, Manish Ganeshan & Aryan Pai mentored by Akshay Sir and Ajinkya Sir. We are three 10th graders studying at DAIS, Mumbai and are being mentored at The Innovation Story.

All 3 of us have prior *FIRST* Tech Challenge and *FIRST* Robotics Competition experience, and wanted to try our hands at something new!

**Aryan Pai** is the captain of an FTC Team, and a drive team member of an FRC team. He's a 10th grader at the Dhirubhai Ambani International School and an avid software developer. As hobbies, he enjoys music production and acting.

**Micquel Devlaliwalla** is a drive team member for an FRC team. He's a full-stack engineer, having experience with HTML, CSS and Javascript alongside Java. He's also a 10th grader at the Dhirubhai Ambani International School, as well as being a part of the school's student council.

**Manish Ganeshan** TO BE ADDED

## Content:
* Electricals
* Open Challenge Code
* Obstacle Challenge Code
* Robot CAD
* Robot Structure + Photos
* Youtube Channel
* Team Photos
* Debugging

## Electricals
The electrical architecture of this WRO bot is a sophisticated blend of computing power, sensor technology, and motor control systems. At its core, the Raspberry Pi 4B serves as the primary computational unit, offering robust processing capabilities for complex algorithms and decision-making processes. This is complemented by the ESP32 Devkit V1, which, although marked for potential replacement, currently provides additional processing power and wireless connectivity options. The bot's user interface is facilitated by a high-resolution 5-inch Waveshare DSI LCD with capacitive touchscreen, enabling intuitive control and real-time data visualization.

Sensor integration plays a crucial role in the bot's perception and navigation capabilities. The Adafruit BNO085 9-DoF IMU provides accurate orientation and motion data, essential for maintaining balance and tracking movement. Three TFMini Plus LiDAR distance sensors offer precise obstacle detection and distance measurement, while the Adafruit TCS34725 RGB color sensor allows for color-based decision making or line following. This sensor array enables the bot to build a comprehensive understanding of its environment, crucial for autonomous operation in various challenge scenarios.

Power management and motor control are carefully designed to ensure efficient and reliable operation. The system is powered by a 3-cell 11.1V LiPo battery with a 1300mAh capacity, providing a good balance of power output and duration. A 12V to 5V buck converter module ensures appropriate voltage levels for different components. The Cytron MD10C R3 motor driver manages the SE DC geared motor, while the high-torque Robokits India digital servo provides precise control for steering or manipulator functions. A power switch and battery voltage tester are included for convenient operation and monitoring.

The inclusion of a breadboard and butt joint cable connectors suggests a degree of flexibility in the electrical setup, allowing for easy modifications and expansions. This adaptability is crucial in a competitive robotics environment where quick adjustments may be necessary. The addition of a button provides a simple physical interface for initiating routines or emergency stops. Overall, this electrical architecture demonstrates a well-rounded approach, balancing computational power, sensory input, motor control, and power management to create a versatile and capable robotics platform suitable for the challenges of the World Robot Olympiad.

## Game Analysis
Our team has carefully examined the requirements for this season's World Robot Olympiad (WRO) Self-Driving Car Challenge. We recognize that the competition adopts a Time Attack format, presenting us with two distinct challenges that will test our autonomous vehicle technology.

**Open Challenge**
Our objective is to complete three laps on a track with randomized inner wall placements. We have identified several key areas for focus:

* Development of adaptive path planning algorithms
* Enhancement of real-time obstacle detection capabilities
* Ensuring consistent performance across multiple laps
* Optimizing the balance between speed and safety

Our strategy involves:

* Creating robust algorithms to handle unpredictable track layouts
* Implementing efficient cornering techniques
* Optimizing our sensor utilization for rapid environment mapping


**Obstacle Challenge**
This challenge requires us to navigate three laps while adhering to lane instructions indicated by colored traffic signs. We must consider:

* Red pillars instruct us to keep to the right side of the lane
* Green pillars direct us to the left side
* The final sign of the second lap determines our direction for the third lap

Our strategic considerations include:

* Perfecting color recognition and rapid decision-making processes
* Developing smooth transitions between left and right positioning
* Preparing for potential direction reversal on the final lap
* Maintaining optimal speed while adhering to sign instructions

Additional factors we must account for:
* Ensuring our vehicle does not disturb the traffic signs
* Maintaining consistent performance across all three laps
* Balancing speed with precision to achieve the best overall time

We recognize that this challenge will test multiple aspects of our autonomous driving technology, including computer vision, object recognition, dynamic path planning, precise vehicle control, and decision-making algorithms.
Our team understands the need to create a versatile and robust system capable of handling various scenarios while maintaining speed and accuracy. The randomized elements in both challenges necessitate the development of truly adaptive and intelligent self-driving solutions.

## Rapid Prototyping

Our rapid prototyping process evolved significantly as we developed our robot. Initially, we started with the Arduino IDE and a HuskyLens for vision processing. This setup allowed us to quickly get a basic prototype up and running. However, we soon realized that the processing capabilities were insufficient for our more advanced vision and decision-making algorithms.

To address these limitations, we pivoted to a more powerful setup. We transitioned to using a Raspberry Pi 4 as our main processing unit, running Python code developed in PyCharm. This shift provided us with much more computational power and flexibility. For motor control and servo movements, we retained an ESP32 microcontroller, establishing communication between the Raspberry Pi and ESP32 via UART. This separation of concerns allowed us to leverage the strengths of both systems: the Raspberry Pi excelled at complex computations and vision processing, while the ESP32 efficiently handled low-level hardware control. This new architecture significantly improved our robot's performance and allowed us to implement more sophisticated algorithms.

## Debugging
