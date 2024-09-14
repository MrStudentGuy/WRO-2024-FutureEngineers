# Team Paraducks | WRO Future Engineers 2024

## Introduction
This repository provides all the information about the WRO Future Engineers 2024 Team Paraducks. The team members are Micquel Devlaliwalla, Manish Ganeshan & Aryan Pai mentored by Akshay Sir and Ajinkya Sir. We are three 10th graders studying at DAIS, Mumbai and are being mentored at The Innovation Story.

All 3 of us have prior *FIRST* Tech Challenge and *FIRST* Robotics Competition experience, and wanted to try our hands at something new!

**Aryan Pai** is the captain of an FTC Team, and a drive team member of an FRC team. He's a 10th grader at the Dhirubhai Ambani International School and an avid software developer. As hobbies, he enjoys music production and acting.

**Micquel Devlaliwalla** is a drive team member for an FRC team. He's a full-stack engineer, having experience with HTML, CSS and Javascript alongside Java. He's also a 10th grader at the Dhirubhai Ambani International School, as well as being a part of the school's student council.

**Manish Ganeshan** is a construction team member for a FRC team. He's an avid constructer and loves STEM. He's a 10th grader and a student council member at the Dhirubhai Ambani International School. He is also a sports enthusiast, WRO being the perfect link between conpetitve sport and robotics.

## Content
* Electrical Architecture
* Game Analysis
* Open Challenge Code
* Obstacle Challenge Code
* Robot CAD
* Robot Structure + Photos
* Youtube Channel
* Team Photos
* Debugging

## Electrical Architecture

The electrical system of this WRO bot is built around a Raspberry Pi 4B as the main processing unit, providing powerful computation capabilities for complex algorithms and decision-making processes. This is complemented by an ESP32 Devkit V1, which, although noted as temporary, offers additional wireless connectivity options and computational power. The combination of these two processors allows for distributed computing, potentially offloading tasks such as sensor data processing or low-level control to the ESP32, while the Raspberry Pi handles high-level planning and coordination.

The bot's sensory array is comprehensive and designed for precise environmental perception. At its core is the Adafruit BNO085 9-DoF IMU (Inertial Measurement Unit), which provides accurate orientation, acceleration, and angular velocity data crucial for navigation and stability control. The inclusion of three TFMini Plus LiDAR distance sensors enables the bot to create a detailed spatial map of its surroundings, facilitating obstacle avoidance and path planning. An Adafruit TCS34725 RGB color sensor rounds out the perceptual capabilities, allowing the bot to distinguish between different colored objects or lines, which is often critical in robotics competitions.

User interaction and system monitoring are facilitated through a 5-inch Waveshare DSI LCD with a capacitive touchscreen, providing a high-resolution 800x480 pixel display. This interface allows for real-time feedback, control inputs, and visualization of sensor data, enhancing the bot's usability and debugging capabilities. The inclusion of a breadboard in the design allow a degree of flexibility, allowing for quick circuit modifications and prototyping of new features.

Power management is a critical aspect of the bot's design, centered around a 3-cell 11.1V LiPo battery with a capacity of 1300mAh and a 30C discharge rate. This power source provides a good balance of energy density and output capability, suitable for powering both the high-draw motors and the sensitive electronic components. A 12V to 5V buck converter module ensures that components requiring lower voltages receive clean, regulated power. Safety features include a LiPo battery voltage tester for monitoring battery levels and a main power switch for convenient system control.

Motion control is achieved through a combination of a Cytron MD10C R3 motor driver and a high-torque servo. The motor driver manages the SE DC geared motor, which operates at 200RPM and 12V, likely providing the main propulsion for the bot. Fine positioning and manipulation tasks are handled by the Robokits India RKI-1203 digital servo, boasting an impressive 35kgcm of torque. This servo's metal gear construction and coreless design suggest it's capable of precise, high-force movements, potentially used for steering, gripper control, or other mechanical operations.

The overall electrical architecture demonstrates a well-thought-out approach to creating a versatile and capable robotics platform. The combination of powerful processing, diverse sensors, robust power management, and precise motor control provides a solid foundation for tackling the varied challenges presented in the World Robot Olympiad. The inclusion of elements like butt joint cable connectors and a breadboard also indicates a design philosophy that values modularity and ease of modification, critical features in a competitive robotics environment where quick adjustments and iterative improvements can make the difference between success and failure.

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

## Robot Photos

![image](https://github.com/user-attachments/assets/d9219694-5fc7-42ac-bf8e-7ec5185aebed)

![image](https://github.com/user-attachments/assets/89cd5e08-d41c-428e-8fff-790a9f905dad)

![image](https://github.com/user-attachments/assets/915d3601-9517-48dd-935f-37e742e6c9c8)

![image](https://github.com/user-attachments/assets/76ab2de8-cfe6-48e6-9306-c60922ddec73)


![image](https://github.com/user-attachments/assets/b861fbc4-687c-4396-977f-e003d53092af)

![image](https://github.com/user-attachments/assets/d344d820-f7d5-47eb-8287-e00daeffea4f)


## Debugging
    
