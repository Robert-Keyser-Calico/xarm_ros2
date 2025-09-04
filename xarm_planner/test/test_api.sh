#!/bin/bash

# This script provides curl commands to test the new endpoints of the xarm_planner_flask_api.py script.

# --- Test for /move_tool_position ---
# This command tests the endpoint for moving the tool relative to its current position.
# The example payload will move the tool 5cm forward along its X-axis.
echo "Testing /move_tool_position endpoint..."
curl -X POST http://localhost:5000/move_tool_position \
-H "Content-Type: application/json" \
-d '{
    "x": 0.05,
    "y": 0,
    "z": 0,
    "roll": 0,
    "pitch": 0,
    "yaw": 0,
    "speed": 0.1,
    "acc": 1
}'

echo "
"

# --- Test for /plan_and_execute_trajectory ---
# This command tests the endpoint for executing a pre-defined trajectory.
# The example payload defines a trajectory with two waypoints.
echo "Testing /plan_and_execute_trajectory endpoint..."
curl -X POST http://localhost:5000/plan_and_execute_trajectory \
-H "Content-Type: application/json" \
-d '{
    "waypoints": [
        {
            "x": 0.4,
            "y": 0.1,
            "z": 0.2,
            "roll": 3.14,
            "pitch": 0.0,
            "yaw": 0.0
        },
        {
            "x": 0.4,
            "y": -0.1,
            "z": 0.2,
            "roll": 3.14,
            "pitch": 0.0,
            "yaw": 0.0
        }
    ]
}'

echo "
"
