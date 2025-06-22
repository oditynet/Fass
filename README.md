# FASS - PROJECT IN DEVELOPMENT

Robot on Arduino and servos with 2 degrees of freedom.

# Theory:
Forward and inverse kinematics:
<img src="https://github.com/oditynet/Fass/blob/main/dog/kinematic.png" title="example" width="800" />

Forward kinematics is knowing the impulses to find the coordinates of the end point X;Y.

For the first arm:

- x1 = L1 * cos(a);
- z1 = L1 * sin(a);

For the second arm. The minus sign is because the knee is pointing down and to get the second angle, you need to subtract from the first:

- x = x1 + L2 * cos((a - b));
- z = z1 + L2 * sin((a - b));

Inverse kinematics is to find angles (motor impulses) by X;Y coordinates. The graph shows 4 possible positions of the paw and therefore 4 different calculation methods. The angle t is always constant (equal to 45.5744 degrees), since the servo can rotate from 0 to 180 degrees (130 - 600 impulses) and for the motor this line is considered the abscissa. For the researcher, the abscissa is the X line. Arduino works with radians (1 rad = 57.29577 degrees) and therefore it is necessary to translate the values.

The same kinematics is for the movement of the paw to the side for turning. turning. walking sideways. The task is to find the angle y_c and to what height to compensate for the paw lift to the height z_H
<img src="https://github.com/oditynet/Fass/blob/main/dog/kinematic-side.png" title="example" width="800" />

# Step theory:

There are 2 types of steps in walking:

1) 2x2 - 1.4 paws move forward at the same time and then 2.3. The algorithm is successful when the dog's body is perfectly centered
2) 1x3 - the step cycle is divided into 4 stages (by the number of paws). The first paw moves forward in 1/4 of the period, and the other 3 continue to move backward. In the second period, 2/4, the second paw moves forward, and the others continue to move backward, and so on in a circle.

In my project, the center of gravity is unstable, so I chose option 2.

Here is a video of the old kinematics motion logic. The new one works very accurately and takes into account all possible states.

<img src="https://github.com/oditynet/Fass/blob/main/leg.gif" title="leg" width="800" />

<img src="https://github.com/oditynet/Fass/blob/main/jostic/jostic-info.png" title="jostic" width="800" />

<img src="https://github.com/oditynet/Fass/blob/main/dog//fass.jpg" title="fass" width="800" />

Main features of the implementation:
1) Control:
- Radio control via NRF24L01
- Automatic sensor priority
2) Movement algorithms:
- Smooth walking forward/backward
- Turns on the spot
- Automatic return to neutral position
- Adjustable movement speed
3) Inverse kinematics:
- Accurate calculation of joint angles
- Taking into account design constraints
- Smooth movement of legs
4) Safety system:
- Limiters for servos
- Smooth transitions between states

Hardware requirements:
- Arduino nano
- NRF24L01 module
- PCA9685 (servo driver)
- 12 servos (3 for each leg)
- MPU6050 (gyroscope / accelerometer)
- 2 HC-SR04 sensors (ultrasonic rangefinders)
- Remote control with two joysticks KY-023 and NRF24L01

origin project STL: https://novaspotmicro.com/parts-list.html with conversion to the metric system and my functionality. The software is written from scratch, inverse kinematics is developed and written from scratch. The algorithm is mine.
