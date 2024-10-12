# Reinforcement Learning Crawling-Robot

Built a simple quadruped robot that used reinforcement learning for walking. I trained the robot in [Pybullet](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?tab=t.0#heading=h.2ye70wns7io3).

Youtube Video: 


![Table_Crawl](img/table_crawl.gif)  

![Pronk Gait](img/pronk.gif) 

# Hardware

 URDF files are in the models folder. CAD files (step & STL) are in the hardware folder

# Python Dependencies

* pybullet==3.2.6 (For windows need to also install Microsoft C++ Build Tools)
* matplotlib==3.6.3
* numpy==1.26.4
* gym==0.23.1
* tensorflow == 2.17.0
* stable-baselines3 == 2.3.2
* shimmy == 0.2.1

# Setting up the Training Enviornment

![Pronk Sim](img/pronk_sim.gif) 

Pybullet's documentation was a little hard for me to first understand on how to setup the learning enviornment. Two main things is 

* initializing the enviornment in the `_init_.py` in the pybullet_envs folder

```
C:\Users\My PC\AppData\Local\Programs\Python\Python311\Lib\site-packages\pybullet_envs
```

Code to add to the `_init_.py`

```
register(
    id='CrawlingRobot-v0',
    entry_point='pybullet_envs:RobotEnv',
    max_episode_steps=1000,
    reward_threshold=1000.0
)
```

* Creating the `robot_env.py`. This is the big code on how to initilize robot into world how to reset, how to step simulation, how to get robot observation, and how to computer reward. This is is the main script that has to be modified

This script also has to be placed inside the pybullet_envs folder

# References (Cool papers)

* Kamidi, Vinay R., Wael Saab, and Pinhas Ben-Tzvi. "Design and analysis of a novel planar robotic leg for high-speed locomotion." 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2017.

* Singla, Abhik, et al. "Realizing learned quadruped locomotion behaviors through kinematic motion primitives." 2019 International Conference on Robotics and Automation (ICRA). IEEE, 2019.

* Tan, Jie, et al. "Sim-to-real: Learning agile locomotion for quadruped robots." arXiv preprint arXiv:1804.10332 (2018).

* Rudin, Nikita, et al. "Learning to walk in minutes using massively parallel deep reinforcement learning." Conference on Robot Learning. PMLR, 2022.

* Schulman, John, et al. "Proximal policy optimization algorithms." arXiv preprint arXiv:1707.06347 (2017).

