# Obstacle Avoidance With Motion Primitives and A*
### PDM Project
### **Course:** RO47005 Planning and Decision Making  
### **Academic Period:** 2024/25 Q2  
### **Date:** 13/01/2025  

---

## Maintained by  
- **Group 26**  
  - **4798740** Stan Marseille  
  - **5096979** Pieter Klopper  
  - **5171598** Jolle Verhoog  
  - **5452333** Marcin Poplawski  

---


https://github.com/user-attachments/assets/35a297a5-b55a-447e-ad98-3f7275b300e2



## Project Overview  

This repository implements a motion planning system for autonomous vehicle, simulated in a `gym_envs_urdf` environment. It uses a **kinematic bicycle model** for realistic motion, combining **Motion Primitives** and the **A*** algorithm for efficient real-time path optimization. The system includes:  
1. A **global planner** for navigating static environments.  
2. A **local planner** for avoiding dynamic obstacles in real-time.  

---

## Repository Contents  

### Executable  
- **`simulation.py`**: Combines helper files and provides visualization for the full simulation.  

### Folders

1. **`maps`** (Simulation Environments):  
   - **`rectangular_environment.py`**: Straight road setup with wall boundaries and static obstacles.  
   - **`l_shaped_environment.py`**: Corner road setup with wall boundaries, static obstacles, and dynamic obstacles.  

2. **`planners`** (Planning Algorithms):  
   - **`global_planner.py`**: Implements pathfinding with A* algorithm, heatmap-based centering, and polynomial smoothing.  
   - **`motion_primitives.py`**: Framework for generating smooth, collision-free trajectories and visualizing them.  

3. **`urdf`** (Robot and Visualization Files)  
   
4. **`images`** (Outputs Folder)  

---

## Pre-requisites  
- **Python** >= 3.8  
- **pip3**  
- **git**  

---

## Installation  

1. **Set Up the Environment**:  
   - Clone and install the `gym_envs_urdf` library:
     ```bash
     git clone git@github.com:maxspahn/gym_envs_urdf.git
     ```

2. **Clone This Repository**:  
   - Clone the PDM Group 26 repository:
     ```bash
     git clone git@github.com:marpopl/PDM-Group26.git
     ```

---

## Changing Parameters  

### 1. **Environment Selection**:  
- Choose the environment by setting `L_shaped` and `Rectangular` booleans in the simulation script.  

### 2. **Global Settings**:  
By default Test D is executed for rectangular environment, and Test I for L-shaped environment.  

To change between different tests you need to uncomment relevant lines in `simulation.py` (lines 132-161 for rectangular and lines 188-217 for L-shaped) and, if needed, modify the `safety_margin` (default to 1.0) in `planners/motion_primitives.py` (line 13).  

The velocity is set at the beginning of `run_prius_with_walls` function in `simulation.py` (line 104). It is set to 1.0 by default, although can be changed (if too large velocity is selected the path will not be created).  

PID control gains are defined at line 290 of `simulation.py` and can be adjusted.  

Optionally, obstacle parameters for rectangular (lines 124-125) and L-shaped (lines 174-179) environments can be edited in the function `run_prius_with_walls` in `simulation.py`, with detailed descriptions of parameter meaning in `maps/rectangular_environment.py` and `maps/l_shaped_environment.py`.  

### 3. **Local Planner Settings**:  
In `simulation.py` under `# local planner settings` three safety margins are set (lines 304-306) and are relevant for Tests G-L in L-shaped environment with dynamic obstacles. These margins work by default for Test I and can be adjusted.  

---

## Run Instructions  

1. **Activate the Environment**:  
   - Source the `gym_envs_urdf` virtual environment:  
     ```bash
     source urdfenvs/bin/activate
     ```

2. **Run the Main Program**:  
   - Navigate to the repository and execute the main script:  
     ```bash
     cd PDM-Group26
     python3 simulation.py
     ```

---

## Possible Issues  

- **Virtual Environment Activation**: Ensure `gym_envs_urdf` is correctly installed and sourced before running the simulation.  
- **Parameter Configuration**: Ensure parameters in `run_prius_with_walls` are properly set for the chosen environment.  
- **Too high velocity**: Line 154 of `motion_primitives.py` will return a KeyError if the selected velocity is too high. Try setting velocity to default value of 1.0.  


