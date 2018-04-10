# Autonomous Indoor Navigation using Image Processing

The arena is an abstraction of an hospital floor which consists of a service zone and a patient zone separated by a corridor and obstacles. The task is to detect the various provisions requested by the patients such as Medicine, Thermometer and Water, which are indicated by various color markers placed. The robot pickups the required provisions (maximum of 2 at a time, indicated using the RGB LEDs) and delivers them to the patients.

The task is performed using Image Processing in Python with OpenCV, using input from solely an overhead webcam. All the markers, the position and orientation of the robot and obstacles are detected and overall efficient path is planned using A* algorithm. The commands are wirelessly sent to the Firebird V robot using the XBee modules.

![](https://github.com/heethesh/Autonomous-Indoor-Navigation-using-Image-Processing/blob/master/images/eyrcplus_1673_python_code%20(SAMPLE%20OUPUT%20IMAGES).JPG)

## Video Demonstration
[Caretaker Robot Video Demonstration](https://www.youtube.com/watch?v=ije_Gh3m6FA) 

Team ID: **eYRC+#1673** | 
First prize winner at National Level e-Yantra Robotics Competition 2014-15 conducted by e-Yantra, CSE Department, IIT Bombay, sponsored by MHRD, Government of India

## Repository Contents
- **python** - The code running on the PC (sorry for the long script without modules)
- **firebird_code** - The code running on Firebird V
- **images** - Images of the robot, arena and path planning
- **docs** - Documentation on the working of the robot and some of the task submissions

## Caretaker Robot
![](https://github.com/heethesh/Autonomous-Indoor-Navigation-using-Image-Processing/blob/master/images/caretaker_robot.jpg)


