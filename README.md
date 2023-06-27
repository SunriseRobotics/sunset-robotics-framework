# pyROSe
Built for user simplicity, pyROSe is a python robotics framework inspired by ROS. 

<h2>What is this?</h2>
Inspired by the flexibility of the [Robot Operating System](https://www.ros.org/) we wanted to enable the use of a few of its core features such as robust logging and the Topic/Subscriber architecture in a way that is more accessible and written 100% in Python. 

<p>Our goal is to bring intelligent robot control to everyone.</p>

<h3>Features:</h3>

* Topic / Subscriber Paradigm
  * Structure your code in a way that forces ease of logging 
  * Circular Dependency Detection using tortoise and hare algorithm 
  * Easy to understand object oriented design patterns
  * Async runtime without threads enabled by the use of homogeneous finite state machines
* Complete runtime reconstruction from logs 
  * Debug issues using real data, default in EVERY pyROSe project
  * No extra time spent trying to recreate issues:  Retrieve your log file and get to work
* Comprehensive Lie theory based geometry for precise modeling of robotic motion. 
  * Built in UDP client for visualizing position information without complex config settings. 
