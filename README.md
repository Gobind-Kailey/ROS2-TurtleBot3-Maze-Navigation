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

## 🛠️ Step-by-Step Workflow

### 1️⃣ Generate a Maze Layout
I began by using an online maze generator to create a 2D maze layout. This provided the blueprint for my simulation environment.

### 2️⃣ Import Maze into Gazebo
I launched Gazebo and imported the generated maze image.  
In Gazebo, I traced the 2D maze image to build a **3D version** of the maze using walls and obstacles. This gave me a full 3D maze environment for the TurtleBot3 to explore.

![IMG_6546](https://github.com/user-attachments/assets/dd15cd2d-db15-416e-951d-129bb79440bb)

### 3️⃣ Save the World and Add a Launch File
Once the maze was complete:
- I saved the world as `project1.world` inside the `worlds/` folder of the `turtlebot3_gazebo` package.
- I created a new launch file named `turtlebot3_project1.launch.py` (copied and modified from `turtlebot3_house.launch.py`) inside the `launch/` folder of the same package.

![IMG_6548](https://github.com/user-attachments/assets/354e37d0-a89b-461b-bde8-6e19b0905b7a)

### 4️⃣ Launch the Maze World in Gazebo
Build your workspace and launch the custom world:

cd ~/turtlebot3_ws
colcon build
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_project1.launch.py

This spawns a TurtleBot3 in your custom maze world inside Gazebo.

### 5️⃣ Run Cartographer SLAM

Start SLAM to generate a map while the robot explores:

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

This opens RViz2 with Cartographer running, ready to map the environment.

### 6️⃣ Teleoperate the Robot to Build the Map

Manually drive the robot around to build the map:

ros2 run turtlebot3_teleop teleop_keyboard


Use the keyboard to navigate through all corridors so the Cartographer can produce a complete map.

![IMG_6549](https://github.com/user-attachments/assets/36f192bf-3ae4-46e1-8cb4-b1cef41262c0)


### 7️⃣ Save the Map

Once the map is sufficiently built:

ros2 run nav2_map_server map_saver_cli -f maps/project1


This saves two files: project1.yaml and project1.pgm in the maps/ directory.

### 8️⃣ Clean Up the Map

The SLAM-generated map may contain noise or slight misalignments.
I opened the .pgm file in GIMP and manually cleaned up the lines and walls to better reflect the actual maze layout.

Before: 


After: 

![IMG_6550](https://github.com/user-attachments/assets/492326fb-d545-4128-a100-6c2bbc0cdd1e)


### 9️⃣ Run Navigation2 with the Clean Map

With a clean map saved, launch Nav2 to enable autonomous navigation:

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/project1.yaml

Map at Launch
![IMG_6551](https://github.com/user-attachments/assets/6f4f27f7-30b7-4e17-8290-cd7c25846697)


### 🔟 Localize and Set Navigation Goals in RViz2

In RViz2:

Use 2D Pose Estimate to set the robot’s initial pose on the map.

Use Nav2 Goal to set a destination within the maze.

The TurtleBot3 will plan and execute a path through the maze using the cleaned map.

Map when Finished Nav2 goal reached:

![IMG_6556](https://github.com/user-attachments/assets/196f436b-fbf3-4440-90ea-bb11bdcdfa25)


### 🧩 Challenges and Problem-Solving

Map Noise: The initial Cartographer map had small errors; manual editing in GIMP improved navigation accuracy.

Launch Integration: Creating a dedicated launch file for the custom world streamlined running Gazebo with the new maze.

Localization: Ensured correct pose estimation before sending goals to Nav2 to prevent path planning failures.

### 📌 Future Improvements

Test different SLAM packages (Hector SLAM, GMapping) for comparison.

Add dynamic obstacles to evaluate path replanning.
   
