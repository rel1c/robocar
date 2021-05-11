# Autonomous Racecar

Table of Contents
=================

- [About](#about)
- [Equipment](#equipment)
- [License](#license)

### About

This project originally started as an assignment for the class CSci 5551
Introdution to Intelligent Robotic Systems at the University of Minnesota
during the Spring semester of 2020. Rather than taking a final, students are
encouraged to solve a problem of their choosing with Robot Operating System
([ROS](https://www.ros.org)). My class partner and I chose to design and build
a racetrack navigating robot by starting with a RC car, a Raspberry Pi and a
camera, among other things. Our previous attempt is preserved on this
repository, but now abandoned. Long story short, we were stymied by a lack of
knowledge in both the artificial intelligence and electrical engineering
fields.

I am now attempting to restore this project to its former glory with a
new perspective and knowledge of the problem's domain.

### Equipment

- The race car is built upon a hobbyist remote control car chassis, specifically a 1/18 scale 
  [Horizon Hobby ECX Torment](https://www.horizonhobby.com/ECX01001T1).
- For a computer, I am currently using a 
  [Raspberry Pi 3 Model B](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/).
- The steering servo and wheel motor are powered by a standard, RC 7.2V LiPo battery.
- Rather than the stock ESC, I'm using an easily sourced
  [Hobbywing QuicRun 1060](http://hobbywing.com/goods.php?id=358). which communicates with
  the Pi through a [PCA9685 Servo Driver](https://www.adafruit.com/product/815) to control
  both the steering servo and wheel motor.
- The camera is an [Arducam 5MP Fisheye with M12 Lens](https://www.amazon.com/gp/product/B013JWEGJQ),
  which was chosen for its wide field of view. Previously I had used a camera with a much narrower
  field of view and it was too restrictive to be useful.
- The Pi is powered with a portable 5000mAh USB battery. They are all over eBay for around $10.

### License

[![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://badges.mit-license.org)
**[MIT license](http://opensource.org/licenses/mit-license.php)**

Copyright © 2021 Adam Peterson
