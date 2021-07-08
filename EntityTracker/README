The Entity Tracking and Surveillance Robot
==========================================

	The Entity Tracker is a four wheeled robot that has Mecanum wheels for locomotion. Its computing
power consists of a Raspberry Pi B+ and a Bluno (Arduino Uno, with Bluetooth 4.0 LE,
http://www.robotshop.com/en/bluno-arduino-uno-ble-bluetooth-4-microcontroller.html). It will use
a Dagu Compound Infrared Sensor (http://www.robotshop.com/en/dagu-compound-infrared-sensor.html) as
its main sensor for tracking moving entities and getting distance measurements. The Raspberry Pi B+
will have a Raspberry Pi Camera Module, and a 2.8" Resistive PiTFT Touch Screen.

	The Dagu Compound Infrared Sensor and Raspberry Pi Camera Module will be mounted on a 3DOF arm,
above a 2DOF gripper. The gripper will have wrist rotation and grabbing, so will be able to physically
interact with its environment. As of now (subject to change, without notice!), the Bluno will handle the
sensor and servos. The Raspberry Pi B+ will communicate with the Bluno via USB serial to get sensor data
and give servo movement commands. I alsoSurveillance have a Pololu Mini Maestro 24-Channel USB Servo Controller
(http://www.robotshop.com/en/mini-maestro-24-channel-usb-servo-controller-assembled.html) that may come
into play if controlling the servos and handling the sensor(s) is too much of a load on the Bluno. Either
board is capable of using the servo controller, since it has three different control modes.

	Locomotion will be provided by a Dagu 4 Channel 5-12V, 2A Brushed DC Motor Controller
(http://www.robotshop.com/en/dagu-4-channel-brushed-dc-motor-controller.html), driving four Pololu 50:1 Micro
Metal HP Gearmotors (with Extended Motor Shaft, http://www.pololu.com/product/2213), with Pololu Optical Encoders
for Micro Metal Gearmotors, 5V (http://www.pololu.com/product/2590), and Mecanum wheels
(http://www.robotshop.com/en/mecanum-wheel-4-pack-w-metal-hubs.html) by FingerTech. The Entity Tracker will
be able to move in any direction, without changing its orientation.

