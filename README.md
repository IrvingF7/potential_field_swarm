# Introduction
This is the final project of NYU ROB-GY 6333 Swarm Robotics done by [Irving Fang](https://irvingf7.github.io/). If you find this repo while you are doing your final project, I suggest you use this as a bit of inspiration for your own design and implementation instead of just copying my code. It's better for your learning (and grading if you got caught of course).


![](pics/circle.png) ![](pics/push_purple.png)

# Goal
The project involve designing a controller for a swarm of mobile robots in [PyBullet](https://pybullet.org/wordpress/) environment so they can:
1. Make a square formation in the room
2. Get all the robot out of the room and make a circle formation outside
3. Move the purple ball on the purple square
4. Move the red ball on the red square
5. Get back into the room and make a diamond formation

While doing all these tasks, there should be no collision between different robots and between robots and wall.

[Video Demo](https://drive.google.com/drive/folders/1m32RRFqdAcRXkb7oRwXD429tJHWYJiEU?usp=sharing)

# Implementation
My design involves using [potential field theory for **obstacle avoidance**](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=980728) by Naomi Ehrich Leonard and Edward Fiorelli and edge tension function for **formation control**. 

Most of the above algorithms are implemented in the `python/robot.py` and `python/wall.py`.

Please refer the the [report](https://github.com/IrvingF7/potential_field_swarm/blob/main/Final_Project_Report.pdf) for more details in math and implementation.

# Run the Code
Make sure you have `PyBullet` package installed, then run `python/run_simulation.py` for the animation to appear.