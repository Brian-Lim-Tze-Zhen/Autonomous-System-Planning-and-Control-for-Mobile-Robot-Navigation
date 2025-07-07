# Autonomous System: Planning and Control for Mobile Robot Navigation

This repository contains a ROS-based navigation system for a TurtleBot3 Burger robot. The project implements global path planning using the A* algorithm, local waypoint tracking with a unicycle kinematic controller, and real-time obstacle avoidance via a potential field method. The system is fully simulated in Gazebo and visualized with RViz.

## 🚀 Features

- **Global Path Planning** using A* on an occupancy grid map
- **Local Path Following** with a unicycle-based kinematic controller
- **Obstacle Avoidance** via LIDAR-driven Potential Fields
- **Modular ROS Node Architecture** (Python)
- **Real-time Visualization** in RViz and Simulation in Gazebo
- **Performance Metrics Logging**

## 🧠 System Architecture

- `global_planner.py`: Computes a global path using A* algorithm
- `navigator.py`: Tracks waypoints and switches control logic when obstacles are detected
- `kinematic_controller.py`: Unicycle model-based velocity control
- `potential_fields.py`: Reactive obstacle avoidance using LIDAR

## 🛠️ Dependencies

- Ubuntu 20.04  
- ROS Noetic  
- TurtleBot3 packages (`turtlebot3`, `turtlebot3_simulations`)  
- Python 3  
- `numpy`, `scipy`, `rospy`, `tf`, `matplotlib`  

Install required ROS packages:
```bash
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations

## 🗂️ Folder Structure
turtlebot3_nav_assignment/
├── launch/                                 # Launch files
├── maps/                                   # Static map for A* planning
├── rviz/                                   # RViz config files
├── scripts/                                # Python nodes (global planner, navigator, etc.)
│   ├── global_planner.py
│   ├── navigator.py
│   ├── kinematic_controller.py
│   └── potential_fields.py
├── worlds/                                 # Custom Gazebo world
├── demo.mp4                                # Demo video (1–3 min)
├── turtlebot3_assignment_full_description.pdf     # Project overview report
├── Planning_and_Control_for_Mobile_Robot_Navigation.pdf  # IEEE-style report
├── CMakeLists.txt
├── package.xml
└── README.md



⚙️ How to Run
1. Clone this repository inside your catkin workspace

cd ~/catkin_ws/src
git clone https://github.com/Brian-Lim-Tze-Zhen/Autonomous-System-Planning-and-Control-for-Mobile-Robot-Navigation.git
cd ..
catkin_make

2. Launch the simulation
roslaunch turtlebot3_nav_assignment simulation.launch

3. Run the nodes (in separate terminals)
rosrun turtlebot3_nav_assignment global_planner.py
rosrun turtlebot3_nav_assignment navigator.py
rosrun turtlebot3_nav_assignment kinematic_controller.py
rosrun turtlebot3_nav_assignment potential_fields.py

4. Send a goal in RViz using 2D Nav Goal

🎬 Watch the [demo video](demo.mp4)
Trajectories with and without obstacle shown in the report

Final performance:

Time to Goal: 103.53 seconds

Path Length: 8.91 meters

Average Speed: 0.09 m/s

📄 Report
Read the full IEEE-style report here: Brian_Lim_Tze_Zhen_IEEE_Report.pdf

👤 Author
Brian Lim Tze Zhen
Technische Hochschule Deggendorf
Dept. of Mechatronics and Cyber-Physical Systems
📧 brian.lim-tze-zhen@stud.th-deg.de




