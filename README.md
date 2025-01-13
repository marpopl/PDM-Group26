# PDM Project
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

## Project Overview  

This repository implements a motion planning system for autonomous vehicle, simulated in a `gym_envs_urdf` environment. It uses a **kinematic bicycle model** for realistic motion, combining **Motion Primitives** and the **A*** algorithm for efficient real-time path optimization. The system includes:  
1. A **global planner** for navigating static environments.  
2. A **local planner** for avoiding dynamic obstacles in real-time.  

---

## Repository Contents  

### Main Executable  
- **`simulation.py`**: Combines helper files and provides visualization for the full simulation.  

### Helper Files by Theme  

1. **`maps`** (Simulation Environments):  
   - **`rectangular_environment.py`**: Straight road setup with wall boundaries, static obstacles, and dynamic obstacles.  
   - **`l_shaped_environment.py`**: Corner road setup with wall boundaries, static obstacles, and dynamic obstacles.  

2. **`planners`** (Planning Algorithms):  
   - **`global_planner.py`**: Implements pathfinding with A* algorithm, heatmap-based centering, and polynomial smoothing.  
   - **`motion_primitives.py`**: Framework for generating smooth, collision-free trajectories and visualizing them.  

3. **`urdf`** (Robot and Visualization Files)  
   
4. **`images`** (Outputs)

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
- Edit the function `run_prius_with_walls` in `simulation.py` to modify:  
  - **Velocity**  
  - **Start/End points**  
  - **Obstacle parameters** for both environments.  
  
- Edit the `safety_margin` in `motion_primitives.py` to:  
  - 1.0 for rectangular environment  
  - 0.5 for L_shaped environment  

### 3. **Local Planner Settings**:  
- Adjust safety margins or local velocity under the `# local planner settings` section in the script.  

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
     python3 visualisation/path_planner.py
     ```

---

## Possible Issues  

- **Virtual Environment Activation**: Ensure `gym_envs_urdf` is correctly installed and sourced before running the simulation.  
- **Dependency Issues**: Check all required Python packages (e.g., `matplotlib`, `numpy`) are installed using `pip install -r requirements.txt`.  
- **Parameter Configuration**: Ensure parameters in `run_prius_with_walls` are properly set for the chosen environment.  

