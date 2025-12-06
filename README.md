-----

# FS-Autonomous-Python

First model of a driverless formula student car, all the project is made in python and uses unreal engine for the tests.

## 📋 Table of Contents

  * [Project Overview](https://www.google.com/search?q=%23project-overview)
  * [The Model (Logic)](https://www.google.com/search?q=%23the-model-logic)
  * [Prerequisites](https://www.google.com/search?q=%23prerequisites)
  * [Installation](https://www.google.com/search?q=%23installation)
  * [Usage](https://www.google.com/search?q=%23usage)
  * [Configuration](https://www.google.com/search?q=%23configuration)

-----

## 🏎️ Project Overview

This project implements an autonomous control stack for a Formula Student vehicle using the **[Formula Student Driverless Simulator (FSDS)](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/)**.

The system connects to the Unreal Engine simulation via the FSDS Python API, processes raw sensor data, and executes real-time control commands. It features a reactive driving algorithm capable of navigating track limits defined by cones.

### Key Features

  * **LiDAR Perception:** Custom point-cloud clustering algorithm to identify cones.
  * **Reactive Control:** Centroid-based steering logic with dynamic speed adjustment.
  * **Live Telemetry:** Real-time visualization of cone detection using Matplotlib.
  * **Smart Braking:** Automatic corner detection and velocity-dependent braking.

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
  * **Cornering Logic:**
      * If the steering angle exceeds `0.3` (entering a turn), the throttle is cut to `0.0`.
      * **Active Braking:** If velocity is high ($> 4.0 m/s$) and the turn is sharp ($> 0.5$ steering), the brakes are applied softly to maintain traction.

-----

## 🛠️ Prerequisites

To run this simulation, you need the following environment set up:

### Simulator

  * **Formula Student Driverless Simulator (FSDS):** You must download and run the FSDS binary.
      * [Download FSDS](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases)
      * [FSDS Documentation](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/)

### Python Environment

  * **Python 3.7+**
  * **FSDS Python Client:** The `fsds` library found in the simulator's `python/` folder.

### Python Libraries

Install the dependencies using pip:

```bash
pip install numpy matplotlib
```

-----

## ⚙️ Installation

1.  **Clone this repository:**

    ```bash
    git clone https://github.com/your-username/FS-Autonomous-Python.git
    cd FS-Autonomous-Python
    ```

2.  **Setup the FSDS Library:**

      * Locate the `fsds` folder inside your downloaded Simulator directory.
      * Either copy that `fsds` folder into this project's root, or add it to your PYTHONPATH.
      * *Note: The script currently looks for the library at a specific path. You may need to update the `sys.path.append(...)` line in the script to match your computer.*

-----

## 🚀 Usage

1.  **Launch the Simulator:**
    Open `FSDS.exe` (or the binary for your OS). Select a map (e.g., *Time Trial*).

2.  **Run the Python Agent:**

    ```bash
    python besttry.py
    ```

3.  **Operation:**

      * The script will connect to the API.
      * A Matplotlib window will open, showing the "Bird's Eye View" of the cones relative to the car.
      * The car will automatically accelerate and navigate the track.

-----

## 🎛️ Configuration

You can tune the vehicle behavior by modifying the constants at the top of `besttry.py`:

```python
max_throttle = 0.5       # Maximum gas output (0.0 to 1.0)
target_speed = 7.0       # Target cruising speed (m/s)
max_steering = 0.4       # Steering limiter
cones_range_cutoff = 8   # How far ahead the car "sees" (meters)
```

-----
