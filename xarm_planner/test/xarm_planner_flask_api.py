#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from xarm_msgs.srv import PlanPose, PlanExec, PlanJoint
from visualization_msgs.msg import Marker, MarkerArray

from flask import Flask, request, jsonify
from scipy.spatial.transform import Rotation as R
import threading

# --- ROS 2 Node ---

class XArmPlannerAPI(Node):
    def __init__(self):
        super().__init__('xarm_planner_api_node')
        self.pose_plan_client = self.create_client(PlanPose, 'xarm_pose_plan')
        self.joint_plan_client = self.create_client(PlanJoint, 'xarm_joint_plan')
        self.exec_plan_client = self.create_client(PlanExec, 'xarm_exec_plan')
        self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

    def move_to_holdup_pose(self):
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

    def plan_only_pose(self, x, y, z, roll, pitch, yaw):
        self.get_logger().info(f"Received pose for planning: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}")

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

        # Call planning service
        plan_success = self.call_pose_plan(pose)
        if not plan_success:
            return False, "Planning failed"

        return True, "Planning successful"

    def plan_and_execute_pose(self, x, y, z, roll, pitch, yaw):
        self.get_logger().info(f"Received pose: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}")

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

        # Call planning service
        plan_success = self.call_pose_plan(pose)
        if not plan_success:
            return False, "Planning failed"

        # Call execution service
        exec_success = self.call_exec_plan()
        if not exec_success:
            return False, "Execution failed"

        return True, "Plan and execution successful"

    def call_joint_plan(self, joint_values):
        request = PlanJoint.Request()
        request.target = joint_values
        return self.call_service(self.joint_plan_client, request)

    def call_pose_plan(self, pose):
        request = PlanPose.Request()
        request.target = pose
        return self.call_service(self.pose_plan_client, request)

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
ros_node = None

@app.route('/holdup', methods=['POST'])
def holdup():
    success, message = ros_node.move_to_holdup_pose()
    if success:
        return jsonify({"message": message})
    else:
        return jsonify({"error": message}), 500

@app.route('/visualize_axes', methods=['POST'])
def visualize_axes():
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

# --- Main ---

def run_ros_node():
    global ros_node
    rclpy.init()
    ros_node = XArmPlannerAPI()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

def main():
    # Run ROS 2 node in a separate thread
    ros_thread = threading.Thread(target=run_ros_node)
    ros_thread.start()

    # Run Flask app in the main thread
    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    main()
