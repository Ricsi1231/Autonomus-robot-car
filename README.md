# Autonomus-robot-car

In our days, mobile robots are increasingly present in many areas of our lives. We can find them in the automotive industry, healthcare, modern warfare, space research, and the entertainment industry alike. The development of this field is ensured by the intensive research currently ongoing. These research efforts encompass the study of the appropriate kinematic structures of mobile robots, the exploration of applicable sensors and actuators, various electronic and software solutions, and not least, the search for control technology solutions that can be effectively applied to mobile robots.

As a result of this work, a four-wheeled mobile robot has been realized in practice. The robot controller is a board with an RP2040 microcontroller, the Raspberry Pi Pico. The structure's additional components include four DC motors, a servo motor, a distance sensor, two infrared sensors, three batteries, an H-bridge motor controller, a push button, a Bluetooth module, and a gyroscope. The connection between the actuators and sensors and the controller is ensured by a custom-developed interface card.

We can switch between the robot's programs using a push button, so there is no need to upload a different program to the microcontroller each time we want to change the program. The robot has four programs: Bluetooth control, autonomous mode, line-following mode, and voice control.

The documentation presents the robot's operating principle, the software's operating principle, the circuit diagram and printed circuit board (PCB) layout of the interface card, the placement diagram, and, not least, the operating principle of the components.

![alt text](https://github.com/[Ricsi1231]/[Autonomus-robot-car]/blob/[branch]/robot.jpg?raw=true)
