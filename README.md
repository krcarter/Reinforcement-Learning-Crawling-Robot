# Crawling-Robot

 Reinforcement learning for crawling robot

 # Hardware

 URDF, CAD files, and robot BOM can be found in the hardware folder. Currently the pybullet code reference the urdf from the pybullet directory, so you can either change that reference in the code or move the urdf and mesh files to the pybullet directory you are working on.

 To create your own custom URDF for bullet using fusion 360 for CAD checkout this github repo

 * https://github.com/yanshil/Fusion2PyBullet (Need to change the color, remove transmissions)
 

# Dependencies

* Pybullet (Microsoft C++ Build Tools)
* Microsoft C++ Build Tools
* matplotlib
* numpy

# References (Resources got inspiration from)

* Kamidi, Vinay R., Wael Saab, and Pinhas Ben-Tzvi. "Design and analysis of a novel planar robotic leg for high-speed locomotion." 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2017.

* Singla, Abhik, et al. "Realizing learned quadruped locomotion behaviors through kinematic motion primitives." 2019 International Conference on Robotics and Automation (ICRA). IEEE, 2019.

