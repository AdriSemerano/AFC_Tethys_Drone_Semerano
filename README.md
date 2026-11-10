# AFC_Tethys_Drone_Semerano
3D Autonomous Underwater Navigation: Optimizing Vector Field Histograms via Evolution Strategies in Gazebo

This github details the development and optimization of an autonomous navigation system for the Tethys Autonomous Underwater Vehicle (AUV) within a Gazebo-based simulated environment. Rather than relying on traditional manual tuning of geometric controllers, this study applies OpenAI Evolution Strategies (ES) to automate the parameterization of a 3D Vector Field Histogram (VFH) algorithm. By removing hardware-level pitch constraints and applying a 3D Euclidean reward function, the VFH controller is adapted to support unconstrained 3D movement. The report outlines the generation of the simulation environments, the architectural adaptations of the VFH node in ROS 2, and the performance analysis of the evolutionary training over successive generations.

In the VFH_Project folder there are the .py and .sdf codes for training and testing including the strategy scripts, the generation of the procedural world and the results and logs
In the uuv_random_policy there are the .cpp codes for training and testing the connection between the drone controlls using the VFH algorithm and Gazebo


Here there is a video showing an iteration of the test phase with a 10 drones swarm: https://drive.google.com/drive/folders/193Eftk3uN1Fo5fCIwcPROSfT9AYlk0rO?usp=sharing 
