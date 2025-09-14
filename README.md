# ROS2-TurtleBot3-Maze-Navigation
Created a custom maze world in Gazebo and used a TurtleBot3 robot to map and autonomously navigate it using SLAM (Cartographer) and Nav2.


## 🚀 Features
- Created a custom maze environment in Gazebo based on a generated maze.
- Used **Cartographer SLAM** to build the map.
- Refined the map manually in GIMP for better accuracy.
- Used **Nav2** with 2D Pose Estimate and Nav2 Goals to autonomously navigate the maze.
- Problem-solved various launch files, mapping, and navigation issues to achieve smooth end-to-end robot navigation.

## 🛠️ Technologies Used
- ROS 2 (Foxy/Humble)  
- TurtleBot3 Simulation Packages  
- Gazebo  
- Cartographer SLAM  
- Nav2 (Navigation2)  
- GIMP (for manual map editing)  

## 📖 How It Works

1. **Create & Launch Maze World**
   ```bash
   # Use the maze generator as a template for creating a pathway for Turtlebot3. 
   cd ~/turtlebot3_ws
   colcon build
   source install/setup.bash
   ros2 launch turtlebot3_gazebo turtlebot3_project1.launch.py
