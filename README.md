# Crawling-Robot

 Reinforcement learning for crawling robot

 # Hardware

 URDF, CAD files are in the hardware folder.

 To create your own custom URDF for bullet using fusion 360 for CAD checkout this github repo

 * https://github.com/yanshil/Fusion2PyBullet (Need to change the color, remove transmissions)

 ## Bill of Materials
 
 1. [8x MG-995 Servos Motors](https://www.amazon.com/Control-Angle180-Digital-Torque-Helicopter/dp/B07NQJ1VZ2)
 2. [Arduino Uno](https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6)
 3. [PCA9685 Servo Board](https://www.amazon.com/HiLetgo-PCA9685-Channel-12-Bit-Arduino/dp/B07BRS249H)
 4. [2s Lipo Battery 50C 2200mAh](https://www.amazon.com/gp/product/B07L6BVRDG)

# Python Dependencies

* pybullet==3.2.6 (For windows need to also install Microsoft C++ Build Tools)
* matplotlib==3.6.3
* numpy==1.26.4
* gym==0.23.1
* tensorflow == 2.17.0
* stable-baselines3 == 2.3.2
* shimmy == 0.2.1

# References (Resources got inspiration from)

* Kamidi, Vinay R., Wael Saab, and Pinhas Ben-Tzvi. "Design and analysis of a novel planar robotic leg for high-speed locomotion." 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2017.

* Singla, Abhik, et al. "Realizing learned quadruped locomotion behaviors through kinematic motion primitives." 2019 International Conference on Robotics and Automation (ICRA). IEEE, 2019.

* Tan, Jie, et al. "Sim-to-real: Learning agile locomotion for quadruped robots." arXiv preprint arXiv:1804.10332 (2018).

* Rudin, Nikita, et al. "Learning to walk in minutes using massively parallel deep reinforcement learning." Conference on Robot Learning. PMLR, 2022.
