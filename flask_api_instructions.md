# xArm Planner Flask API Instructions

This document provides instructions on how to set up and run the `xarm_planner_flask_api.py` script, which provides a web API to control the xArm motion planner.

## 1. Setup

Before running the script, you need to install the required dependencies and prepare your ROS 2 workspace.

### 1.1. Install Python Dependencies

This script requires `flask` and `scipy`. You can install them using `pip`:

```bash
pip install flask scipy
```

### 1.2. Make the script executable

```bash
chmod +x /home/keyser/ros2_ws/src/xarm_ros2/xarm_planner/test/xarm_planner_flask_api.py
```

### 1.3. Rebuild your workspace

Navigate to your ROS 2 workspace and rebuild the `xarm_planner` package:

```bash
cd /home/keyser/ros2_ws
colcon build --packages-select xarm_planner
```

## 2. Running the Application

You will need two terminals to run the application.

### 2.1. Start the xArm Planner

In your first terminal, source your workspace and start the xArm planner. We'll use the "fake" planner for this, which doesn't require a real robot.

```bash
source /home/keyser/ros2_ws/install/setup.bash
ros2 launch xarm_planner xarm6_planner_fake.launch.py
```

### 2.2. Run the Flask API Server

In your second terminal, source your workspace again and run the Flask API script:

```bash
source /home/keyser/ros2_ws/install/setup.bash
ros2 run xarm_planner xarm_planner_flask_api.py
```

You should see output indicating that the Flask server is running on `http://0.0.0.0:5000/`.

## 3. Testing the API

You can now send a 6-DOF pose to the API to have the robot plan and execute the motion. Open a third terminal and use `curl` to send a POST request:

```bash
curl -X POST -H "Content-Type: application/json" -d '{
  "x": 0.3,
  "y": -0.1,
  "z": 0.2,
  "roll": 3.14,
  "pitch": 0.0,
  "yaw": 0.0
}' http://localhost:5000/plan_and_execute
```

If successful, you will see a JSON response confirming the action, and the robot will move in the RViz window.
