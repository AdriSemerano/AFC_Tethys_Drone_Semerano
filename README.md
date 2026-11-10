# AFC_Tethys_Drone_Semerano
# 3D Autonomous Underwater Navigation: Optimizing Vector Field Histograms via Evolution Strategies in Gazebo

This repository details the development and optimization of an autonomous navigation system for the Tethys Autonomous Underwater Vehicle (AUV) within a Gazebo-based simulated environment. Rather than relying on traditional manual tuning of geometric controllers, this study applies OpenAI Evolution Strategies (ES) to automate the parameterization of a 3D Vector Field Histogram (VFH) algorithm. By removing hardware-level pitch constraints and applying a 3D Euclidean reward function, the VFH controller is adapted to support unconstrained 3D movement. 

The project outlines the generation of the simulation environments, the architectural adaptations of the VFH node in ROS 2, and the performance analysis of the evolutionary training over successive generations.

## 🎥 Demo Video
Watch a demonstration of an iteration from the testing phase, featuring a **10-drone swarm** navigating the procedural 3D environment:  
👉 **[View the Demo Video on Google Drive](https://drive.google.com/drive/folders/193Eftk3uN1Fo5fCIwcPROSfT9AYlk0rO?usp=sharing)**

## 📂 Repository Structure

* **`VFH_Project/`**  
  Contains the Python (`.py`) and Gazebo (`.sdf`) scripts for both the training and testing phases. This folder includes the Evolution Strategies orchestrator, the procedural world generation scripts, and all generated results and logs.

* **`uuv_random_policy/`**  
  Contains the core C++ (`.cpp`) ROS 2 nodes. This package implements the customized 3D VFH algorithm and manages the bidirectional communication bridge between the drone's low-level controls and the Gazebo physics engine.
