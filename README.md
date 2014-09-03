mono-slam
=========

MonoSLAM implementation in ROS

This package is in beta version.


INSTALLATION
============

Required OpenCV2.4.x and LibConfig++1.9.x



Reference: Russo L.O., Rosa S., Bona B., Matteucci M., "A ROS IMPLEMENTATION OF THE MONO-SLAM ALGORITHM",
International Journal of Computer Science & Information Technology, Vol. 6 Issue 1, p339

http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.401.8518


Usage:

rosrun mono-slam mono-slam configuration.cfg /image:=/your_image_topic

( configuration file contains parameters for the mono-slam algorithm 
  and camera calibration. 
  Sample configuration files are provided in the "conf" folder ) 
