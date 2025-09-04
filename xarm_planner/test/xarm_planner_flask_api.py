#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from xarm_msgs.srv import PlanPose, PlanExec, PlanJoint, MoveCartesian
from visualization_msgs.msg import Marker, MarkerArray

from flask import Flask, jsonify, request
from flasgger import Flasgger
from scipy.spatial.transform import Rotation as R
import threading
from concurrent.futures import TimeoutError

# --- ROS 2 Node ---

class XArmPlannerAPI(Node):
    def __init__(self):
        super().__init__('xarm_planner_api_node')
        self.pose_plan_client = self.create_client(PlanPose, 'xarm_pose_plan')
        self.joint_plan_client = self.create_client(PlanJoint, 'xarm_joint_plan')
        self.exec_plan_client = self.create_client(PlanExec, 'xarm_exec_plan')
        self.cartesian_plan_client = self.create_client(PlanPose, 'xarm_cartesian_plan')
        self.tool_position_client = self.create_client(MoveCartesian, '/xarm/set_tool_position')
        self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

    def move_to_holdup_pose(self):
        # Note: This pose is for a 6-DOF arm. Ensure it matches your robot's configuration.
        holdup_pose = [0.0, 0.0, 0.0, 0.0, -1.5708, 0.0] 
        plan_success = self.call_joint_plan(holdup_pose)
        if not plan_success:
            return False, "Planning to holdup pose failed"
        
        exec_success = self.call_exec_plan()
        if not exec_success:
            return False, "Execution of holdup pose failed"

        return True, "Successfully moved to holdup pose"

    def publish_axes(self, axes_data):
        marker_array = MarkerArray()
        marker_id = 0
        for axis_name, axis_info in axes_data.items():
            origin = axis_info['origin']
            x_vector = axis_info['x_vector']
            y_vector = axis_info['y_vector']
            z_vector = axis_info['z_vector']

            # X Axis
            marker_x = self.create_arrow_marker(marker_id, origin, x_vector, [1.0, 0.0, 0.0, 1.0], axis_name)
            marker_array.markers.append(marker_x)
            marker_id += 1

            # Y Axis
            marker_y = self.create_arrow_marker(marker_id, origin, y_vector, [0.0, 1.0, 0.0, 1.0], axis_name)
            marker_array.markers.append(marker_y)
            marker_id += 1

            # Z Axis
            marker_z = self.create_arrow_marker(marker_id, origin, z_vector, [0.0, 0.0, 1.0, 1.0], axis_name)
            marker_array.markers.append(marker_z)
            marker_id += 1
        
        self.marker_publisher.publish(marker_array)
        return True, "Axes visualized"

    def create_arrow_marker(self, marker_id, origin, vector, color, ns):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        start_point = Point()
        start_point.x = float(origin['x'])
        start_point.y = float(origin['y'])
        start_point.z = float(origin['z'])

        end_point = Point()
        end_point.x = float(origin['x'] + vector['x'])
        end_point.y = float(origin['y'] + vector['y'])
        end_point.z = float(origin['z'] + vector['z'])

        marker.points.append(start_point)
        marker.points.append(end_point)

        marker.scale.x = 0.02  # Shaft diameter
        marker.scale.y = 0.04  # Head diameter
        marker.scale.z = 0.0  # Head length (defaults to 0.1 * shaft diameter)

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        return marker

    def _create_pose_from_rpy(self, x, y, z, roll, pitch, yaw):
        # Convert RPY to quaternion
        rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
        quat = rot.as_quat()

        # Create Pose message
        pose = Pose()
        pose.position.x = float(x) 
        pose.position.y = float(y) 
        pose.position.z = float(z) 
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

    def plan_only_pose(self, x, y, z, roll, pitch, yaw):
        self.get_logger().info(f"Received pose for planning: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}")
        pose = self._create_pose_from_rpy(x, y, z, roll, pitch, yaw)

        # Call planning service
        plan_success = self.call_pose_plan(pose)
        if not plan_success:
            return False, "Planning failed"

        return True, "Planning successful" 

    def plan_and_execute_pose(self, x, y, z, roll, pitch, yaw):
        self.get_logger().info(f"Received pose: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}")
        pose = self._create_pose_from_rpy(x, y, z, roll, pitch, yaw)

        # Call planning service
        plan_success = self.call_pose_plan(pose)
        if not plan_success:
            return False, "Planning failed"
        
        # Call execution service
        exec_success = self.call_exec_plan()
        if not exec_success:
            return False, "Execution failed"
        
        return True, "Plan and execution successful"

    def plan_and_execute_linear_pose(self, x, y, z, roll, pitch, yaw):
        self.get_logger().info(f"Received pose for linear motion: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}")
        pose = self._create_pose_from_rpy(x, y, z, roll, pitch, yaw)

        # Call linear planning service
        plan_success = self.call_cartesian_plan(pose)
        if not plan_success:
            return False, "Linear planning failed"
        
        # Call execution service
        exec_success = self.call_exec_plan()
        if not exec_success:
            return False, "Execution failed"
        
        return True, "Linear plan and execution successful"

    def plan_and_execute_trajectory(self, waypoints):
        self.get_logger().info(f"Received trajectory with {len(waypoints)} waypoints.")
        for i, waypoint in enumerate(waypoints):
            self.get_logger().info(f"Executing waypoint {i+1}/{len(waypoints)}: {waypoint}")
            # Here we reuse the linear pose execution logic for each waypoint
            success, message = self.plan_and_execute_linear_pose(
                waypoint['x'], waypoint['y'], waypoint['z'],
                waypoint['roll'], waypoint['pitch'], waypoint['yaw']
            )
            if not success:
                error_message = f"Failed at waypoint {i+1}: {message}"
                self.get_logger().error(error_message)
                return False, error_message
        return True, "Trajectory executed successfully"

    def move_tool_position(self, x, y, z, roll, pitch, yaw, speed, acc):
        self.get_logger().info(f"Executing relative tool motion: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}")
        
        request = MoveCartesian.Request()
        request.pose = [float(x), float(y), float(z), float(roll), float(pitch), float(yaw)]
        if speed is not None:
            request.speed = float(speed)
        if acc is not None:
            request.acc = float(acc)
        request.wait = True # Wait for motion to complete

        if not self.tool_position_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {self.tool_position_client.srv_name} not available')
            return False, f"Service {self.tool_position_client.srv_name} not available"

        future = self.tool_position_client.call_async(request)
        
        while rclpy.ok() and not future.done():
            pass
        
        if future.result() is not None:
            if future.result().ret == 0:
                self.get_logger().info(f'Service {self.tool_position_client.srv_name} call successful: {future.result().message}')
                return True, future.result().message
            else:
                self.get_logger().error(f'Service {self.tool_position_client.srv_name} call failed with code {future.result().ret}: {future.result().message}')
                return False, f"Failed with code {future.result().ret}: {future.result().message}"
        else:
            self.get_logger().error(f'Service {self.tool_position_client.srv_name} call failed (no result)')
            return False, "Service call failed"

    def call_joint_plan(self, joint_values):
        request = PlanJoint.Request()
        request.target = joint_values
        return self.call_service(self.joint_plan_client, request)

    def call_pose_plan(self, pose):
        request = PlanPose.Request()
        request.target = pose
        return self.call_service(self.pose_plan_client, request)

    def call_cartesian_plan(self, pose):
        request = PlanPose.Request()
        request.target = pose
        return self.call_service(self.cartesian_plan_client, request)

    def call_exec_plan(self):
        request = PlanExec.Request()
        request.wait = True
        return self.call_service(self.exec_plan_client, request)
    
    def call_service(self, client, request):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {client.srv_name} not available')
            return False
        
        future = client.call_async(request)
        # We can't spin here because it would block the web server
        # For this example, we will assume the service call succeeds quickly
        # In a real application, you would want a more robust way to handle this,
        # for example, by returning a task ID and providing another endpoint to check the status.
        self.get_logger().info(f'Calling service {client.srv_name}')
        # A simple wait for the future to complete
        while rclpy.ok() and not future.done():
            pass
        
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Service {client.srv_name} call success')
            return True
        else:
            self.get_logger().error(f'Service {client.srv_name} call failed')
            return False


# --- Flask Web Server ---

app = Flask(__name__)
app.config['SWAGGER'] = {
    'title': 'XArm Planner API',
    'uiversion': 3,
    'description': 'An API to control the xArm robot planner via HTTP requests.',
    'termsOfService': '',
    'contact': {
        'name': 'API Support',
        'url': '',
        'email': ''
    },
    'license': {
        'name': 'BSD 3-Clause',
        'url': 'https://opensource.org/licenses/BSD-3-Clause'
    },
    'specs_route': '/apidocs/'
}
swagger = Flasgger(app)
ros_node = None

@app.route('/holdup', methods=['POST'])
def holdup():
    """Move the robot to a predefined 'holdup' pose.
    This is a safe, known pose for the robot.
    ---
    responses:
      200:
        description: Successfully moved to holdup pose.
        schema:
          type: object
          properties:
            message:
              type: string
              example: Successfully moved to holdup pose
      500:
        description: Error during planning or execution.
    """
    success, message = ros_node.move_to_holdup_pose()
    if success:
        return jsonify({"message": message})
    else:
        return jsonify({"error": message}), 500

@app.route('/visualize_axes', methods=['POST'])
def visualize_axes():
    """Visualize one or more coordinate systems in RViz.
    ---
    parameters:
      - in: body
        name: body
        required: true
        schema:
          type: object
          additionalProperties:
            type: object
            properties:
              origin:
                type: object
                properties:
                  x: {type: number, description: "X coordinate of origin"}
                  y: {type: number, description: "Y coordinate of origin"}
                  z: {type: number, description: "Z coordinate of origin"}
              x_vector:
                type: object
                properties:
                  x: {type: number}
                  y: {type: number}
                  z: {type: number}
              y_vector:
                type: object
                properties:
                  x: {type: number}
                  y: {type: number}
                  z: {type: number}
              z_vector:
                type: object
                properties:
                  x: {type: number}
                  y: {type: number}
                  z: {type: number}
          example:
            axis1:
              origin: {x: 0.5, y: 0.0, z: 0.2}
              x_vector: {x: 0.1, y: 0.0, z: 0.0}
              y_vector: {x: 0.0, y: 0.1, z: 0.0}
              z_vector: {x: 0.0, y: 0.0, z: 0.1}
        description: A dictionary where each key is an axis name and the value contains the origin and vectors for that axis.
    """
    if not request.is_json:
        return jsonify({"error": "Request must be JSON"}), 400

    data = request.get_json()
    # Basic validation
    if 'axis1' not in data or 'axis2' not in data:
        return jsonify({"error": "Missing 'axis1' or 'axis2' in request"}), 400

    success, message = ros_node.publish_axes(data)

    if success:
        return jsonify({"message": message})
    else:
        return jsonify({"error": message}), 500

@app.route('/plan_only', methods=['POST'])
def plan_only():
    """Plan a trajectory to a target pose without executing it.
    ---
    definitions:
      PoseRPY:
        type: object
        required:
          - x
          - y
          - z
          - roll
          - pitch
          - yaw
        properties:
          x:
            type: number
            description: The x-coordinate of the target position in meters.
            example: 0.3
          y:
            type: number
            description: The y-coordinate of the target position in meters.
            example: 0.1
          z:
            type: number
            description: The z-coordinate of the target position in meters.
            example: 0.2
          roll:
            type: number
            description: The roll angle of the target orientation in radians.
            example: 3.14
          pitch:
            type: number
            description: The pitch angle of the target orientation in radians.
            example: 0.0
          yaw:
            type: number
            description: The yaw angle of the target orientation in radians.
            example: 0.0
    parameters:
      - in: body
        name: body
        required: true
        schema:
          $ref: '#/definitions/PoseRPY'
    responses:
      200:
        description: Planning was successful.
      400:
        description: Invalid request, e.g., not JSON or missing keys.
      500:
        description: Planning failed.
    """
    if not request.is_json:
        return jsonify({"error": "Request must be JSON"}), 400

    data = request.get_json()
    required_keys = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
    if not all(key in data for key in required_keys):
        return jsonify({"error": "Missing one or more required keys: x, y, z, roll, pitch, yaw"}), 400

    success, message = ros_node.plan_only_pose(
        data['x'], data['y'], data['z'],
        data['roll'], data['pitch'], data['yaw']
    )

    if success:
        return jsonify({"message": message})
    else:
        return jsonify({"error": message}), 500

@app.route('/plan_and_execute', methods=['POST'])
def plan_and_execute():
    """Plan and execute a trajectory to a target pose.
    This uses joint-space planning (PTP).
    ---
    parameters:
      - in: body
        name: body
        required: true
        schema:
          $ref: '#/definitions/PoseRPY'
    responses:
      200:
        description: Plan and execution was successful.
      400:
        description: Invalid request.
      500:
        description: Plan or execution failed.
    """
    if not request.is_json:
        return jsonify({"error": "Request must be JSON"}), 400

    data = request.get_json()
    required_keys = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
    if not all(key in data for key in required_keys):
        return jsonify({"error": "Missing one or more required keys: x, y, z, roll, pitch, yaw"}), 400

    success, message = ros_node.plan_and_execute_pose(
        data['x'], data['y'], data['z'],
        data['roll'], data['pitch'], data['yaw']
    )

    if success:
        return jsonify({"message": message})
    else:
        return jsonify({"error": message}), 500

@app.route('/plan_and_execute_linear', methods=['POST'])
def plan_and_execute_linear():
    """Plan and execute a linear trajectory to a target pose.
    This uses Cartesian-space planning (linear motion).
    ---
    parameters:
      - in: body
        name: body
        required: true
        schema:
          $ref: '#/definitions/PoseRPY'
    responses:
      200:
        description: Linear plan and execution was successful.
      400:
        description: Invalid request.
      500:
        description: Linear plan or execution failed.
    """
    if not request.is_json:
        return jsonify({"error": "Request must be JSON"}), 400

    data = request.get_json()
    required_keys = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
    if not all(key in data for key in required_keys):
        return jsonify({"error": "Missing one or more required keys: x, y, z, roll, pitch, yaw"}), 400

    success, message = ros_node.plan_and_execute_linear_pose(
        data['x'], data['y'], data['z'],
        data['roll'], data['pitch'], data['yaw']
    )

    if success:
        return jsonify({"message": message})
    else:
        return jsonify({"error": message}), 500

@app.route('/plan_and_execute_trajectory', methods=['POST'])
def plan_and_execute_trajectory():
    """Plan and execute a trajectory consisting of multiple waypoints.
    This uses Cartesian-space planning (linear motion) for each segment.
    ---
    parameters:
      - in: body
        name: body
        required: true
        schema:
          type: object
          required:
            - waypoints
          properties:
            waypoints:
              type: array
              description: A list of waypoints to follow.
              items:
                $ref: '#/definitions/PoseRPY'
          example:
            waypoints:
              - {x: 0.4, y: 0.1, z: 0.2, roll: 3.14, pitch: 0.0, yaw: 0.0}
              - {x: 0.4, y: -0.1, z: 0.2, roll: 3.14, pitch: 0.0, yaw: 0.0}
    responses:
      200:
        description: Trajectory execution was successful.
      400:
        description: Invalid request.
      500:
        description: Trajectory execution failed at some waypoint.
    """
    if not request.is_json:
        return jsonify({"error": "Request must be JSON"}), 400

    data = request.get_json()
    if 'waypoints' not in data or not isinstance(data['waypoints'], list):
        return jsonify({"error": "Missing 'waypoints' key or it's not a list"}), 400

    waypoints = data['waypoints']
    required_keys = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
    for i, waypoint in enumerate(waypoints):
        if not all(key in waypoint for key in required_keys):
            return jsonify({"error": f"Waypoint {i} is missing one or more required keys: x, y, z, roll, pitch, yaw"}), 400

    success, message = ros_node.plan_and_execute_trajectory(waypoints)

    if success:
        return jsonify({"message": message})
    else:
        return jsonify({"error": message}), 500

@app.route('/move_tool_position', methods=['POST'])
def move_tool_position():
    """Move the robot tool relative to its current position.
    This uses the set_tool_position service.
    ---
    parameters:
      - in: body
        name: body
        required: true
        schema:
          type: object
          required:
            - x
            - y
            - z
            - roll
            - pitch
            - yaw
          properties:
            x:
              type: number
              description: The relative x-distance in meters.
              example: 0.05
            y:
              type: number
              description: The relative y-distance in meters.
              example: 0
            z:
              type: number
              description: The relative z-distance in meters.
              example: 0
            roll:
              type: number
              description: The relative roll angle in radians.
              example: 0
            pitch:
              type: number
              description: The relative pitch angle in radians.
              example: 0
            yaw:
              type: number
              description: The relative yaw angle in radians.
              example: 0
            speed:
              type: number
              description: The tool speed in m/s.
              example: 0.1
            acc:
              type: number
              description: The tool acceleration in m/s^2.
              example: 1
    responses:
      200:
        description: Motion was successful.
      400:
        description: Invalid request.
      500:
        description: Motion failed.
    """
    if not request.is_json:
        return jsonify({"error": "Request must be JSON"}), 400

    data = request.get_json()
    required_keys = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
    if not all(key in data for key in required_keys):
        return jsonify({"error": "Missing one or more required keys: x, y, z, roll, pitch, yaw"}), 400

    # Convert to mm, mm/s, mm/s^2 as expected by the service
    x_mm = data['x'] * 1000
    y_mm = data['y'] * 1000
    z_mm = data['z'] * 1000
    speed_mms = data.get('speed') * 1000 if data.get('speed') is not None else None
    acc_mmss = data.get('acc') * 1000 if data.get('acc') is not None else None

    success, message = ros_node.move_tool_position(
        x_mm, y_mm, z_mm,
        data['roll'], data['pitch'], data['yaw'],
        speed_mms, acc_mmss
    )

    if success:
        return jsonify({"message": message})
    else:
        return jsonify({"error": message}), 500

# --- Main ---

def run_ros_node():
    global ros_node
    rclpy.init()
    ros_node = XArmPlannerAPI()
    ros_node.get_logger().info("XArm Planner ROS Node is ready and spinning.")
    rclpy.spin(ros_node)
    # This block is executed after rclpy.shutdown() is called from the main thread
    ros_node.destroy_node()
    rclpy.shutdown()

def main():
    # Run ROS 2 node in a separate thread
    ros_thread = threading.Thread(target=run_ros_node, daemon=True)
    ros_thread.start()

    # Run Flask app in the main thread.
    # Use debug=False if the auto-reloader causes issues with the ROS thread.
    try:
        app.run(host='0.0.0.0', port=5000, debug=True)
    except KeyboardInterrupt:
        print("shutting down.")
    finally:
        # Ensure the ROS node is shut down gracefully
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=2)
        print("Application shut down.")

if __name__ == '__main__':
    main()
