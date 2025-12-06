-----

# FS-Autonomous-Python

First model of a driverless formula student car, all the project is made in python and uses unreal engine for the tests.

## 📋 Table of Contents

  * [Project Overview](https://www.google.com/search?q=%23project-overview)
  * [The Model (Logic)](https://www.google.com/search?q=%23the-model-logic)
  * [Prerequisites](https://www.google.com/search?q=%23prerequisites)
  * [Installation](https://www.google.com/search?q=%23installation)
  * [Simulation Settings](https://www.google.com/search?q=%23simulation-settings)
  * [Usage](https://www.google.com/search?q=%23usage)

-----

## 🏎️ Project Overview

This project implements an autonomous control stack for a Formula Student vehicle using the **[Formula Student Driverless Simulator (FSDS)](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/)**.

The system connects to the Unreal Engine simulation via the FSDS Python API, processes raw sensor data, and executes real-time control commands. It features a reactive driving algorithm capable of navigating track limits defined by cones.

### Key Features

  * **LiDAR Perception:** Custom point-cloud clustering algorithm to identify cones without cameras.
  * **Reactive Control:** Centroid-based steering logic with dynamic speed adjustment.
  * **Live Telemetry:** Real-time visualization of cone detection using Matplotlib.
  * **Smart Braking:** Automatic corner detection and velocity-dependent braking logic.

-----

## 🧠 The Model (Logic)

The autonomous system operates on a continuous `while True` loop with the following pipeline:

### 1\. Perception (LiDAR Clustering)

Instead of using cameras, this model relies purely on **LiDAR data**.

  * **Input:** Raw point cloud from the simulator.
  * **Processing:** The algorithm iterates through points and groups them based on Euclidean distance (threshold: 0.1m).
  * **Output:** Calculates the geometric center (centroid) of each point group to determine the precise $(x, y)$ location of cones.

### 2\. Lateral Control (Steering)

The steering logic is a **Reactive P-Controller**:

  * It calculates the average lateral position ($y$) of all detected cones within range.
  * It applies a Proportional gain ($K_p = 0.25$) to steer the vehicle toward the center of the track (minimizing the average $y$ offset).

### 3\. Longitudinal Control (Throttle & Brake)

  * **Throttle:** Adjusted dynamically based on the difference between current velocity and `target_speed`.
  * **Cornering Logic:** If the steering angle indicates a sharp turn, the throttle is cut. If the speed is high ($> 4.0 m/s$) and the turn is sharp, the brakes are applied to maintain traction.

-----

## 🛠️ Prerequisites

To run this simulation, you need the following environment set up:

  * **Simulator:** [Formula Student Driverless Simulator (FSDS)](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases)
  * **Python:** Version 3.7+
  * **Libraries:** `fsds`, `numpy`, `matplotlib`

-----

## ⚙️ Simulation Settings (Crucial)

For the Python script to function correctly, the simulator must be configured with specific sensor parameters (specifically a single-layer LiDAR with 500 points per scan).

**⚠️ Important:**
You must use the `settings.json` file provided in this repository.

1.  Locate the `settings.json` file in the root of this project.
2.  Copy it and replace the existing settings file in your FSDS configuration folder (typically located in `Documents/AirSim` or your simulator's root folder).

*Failure to use this specific settings file will result in the car not detecting cones correctly.*

-----

## ⚙️ Installation

1.  **Clone this repository:**

    ```bash
    git clone https://github.com/your-username/FS-Autonomous-Python.git
    cd FS-Autonomous-Python
    ```

2.  **Setup the FSDS Library:**

      * Locate the `fsds` folder inside your downloaded Simulator directory.
      * Ensure the `fsds` library is accessible to your Python environment (either copy it to the project root or add it to PYTHONPATH).

3.  **Install Dependencies:**

    ```bash
    pip install numpy matplotlib
    ```

-----

## 🚀 Usage

1.  **Start the Simulator:** Launch the FSDS executable.
2.  **Run the Agent:**
    ```bash
    python besttry.py
    ```
3.  **Operation:**
      * The script will connect to the API.
      * A Matplotlib window will appear showing the real-time cone detection.
      * The car will automatically accelerate and navigate the track.

-----

## 🎛️ Python Configuration

Vehicle behavior constants can be tuned directly in `besttry.py`:

```python
max_throttle = 0.5       # Maximum gas output
target_speed = 7.0       # Target cruising speed
max_steering = 0.4       # Steering limiter
cones_range_cutoff = 8   # Perception horizon (meters)
```
