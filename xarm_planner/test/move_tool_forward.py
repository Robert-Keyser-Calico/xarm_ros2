#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from xarm_msgs.srv import PlanPose, PlanExec
import tf2_ros
from tf_transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion
import numpy as np
import time

class ToolMover(Node):
    def __init__(self):
        super().__init__('tool_mover')
        self.pose_plan_client = self.create_client(PlanPose, 'xarm_pose_plan')
        self.exec_plan_client = self.create_client(PlanExec, 'xarm_exec_plan')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def wait_for_transform(self, from_frame, to_frame, timeout_sec=5.0):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            try:
                if self.tf_buffer.can_transform(to_frame, from_frame, rclpy.time.Time()):
                    return True
            except tf2_ros.TransformException as e:
                self.get_logger().warn(f'Could not transform {to_frame} to {from_frame}: {e}')
            time.sleep(0.1)
        return False

    def get__current_pose(self, from_frame, to_frame):
        if not self.wait_for_transform(from_frame, to_frame):
            self.get_logger().error(f'Transform from {from_frame} to {to_frame} not available')
            return None
        try:
            transform = self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time(seconds=0))
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation.x = transform.transform.rotation.x
            pose.orientation.y = transform.transform.rotation.y
            pose.orientation.z = transform.transform.rotation.z
            pose.orientation.w = transform.transform.rotation.w
            return pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Could not get transform: {e}')
            return None

    def move_tool_forward(self, distance):
        # Get the current pose of the end-effector in the base frame
        current_pose = self.get_current_pose('link_eef', 'link_base')
        if not current_pose:
            return

        # Create a transformation for the tool motion (10cm forward in X)
        tool_translation = np.array([distance, 0.0, 0.0])
        tool_rotation = quaternion_from_euler(0, 0, 0)

        # Get the orientation of the end-effector
        current_orientation = np.array([
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
        ])

        # Rotate the tool translation by the current orientation
        # This is a simplified approach, for more complex rotations, a full quaternion rotation is needed
        # For this example, we assume the tool's X-axis is aligned with the desired motion
        
        # A more robust way to do this is to use a library like PyKDL
        # but for this simple case, we can do it manually.
        
        # Get the rotation matrix from the quaternion
        r, p, y = euler_from_quaternion(current_orientation)
        rotation_matrix = np.array([
            [np.cos(y)*np.cos(p), np.cos(y)*np.sin(p)*np.sin(r) - np.sin(y)*np.cos(r), np.cos(y)*np.sin(p)*np.cos(r) + np.sin(y)*np.sin(r)],
            [np.sin(y)*np.cos(p), np.sin(y)*np.sin(p)*np.sin(r) + np.cos(y)*np.cos(r), np.sin(y)*np.sin(p)*np.cos(r) - np.cos(y)*np.sin(r)],
            [-np.sin(p), np.cos(p)*np.sin(r), np.cos(p)*np.cos(r)]
        ])
        
        # Apply the rotation to the tool translation
        translated_vector = rotation_matrix.dot(tool_translation)

        # Calculate the new target pose
        target_pose = Pose()
        target_pose.position.x = current_pose.position.x + translated_vector[0]
        target_pose.position.y = current_pose.position.y + translated_vector[1]
        target_pose.position.z = current_pose.position.z + translated_vector[2]
        target_pose.orientation = current_pose.orientation

        # Plan and execute the motion
        self.call_pose_plan(target_pose)
        self.call_exec_plan()

    def call_pose_plan(self, pose):
        request = PlanPose.Request()
        request.target = pose
        return self.call_service(self.pose_plan_client, request)

    def call_exec_plan(self):
        request = PlanExec.Request()
        request.wait = True
        return self.call_service(self.exec_plan_client, request)

    def call_service(self, client, request):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service {client.srv_name} not available, waiting...')
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'Service {client.srv_name} not available')
                return None
        
        future = client.call_async(request)
        self.get_logger().info(f'Calling service {client.srv_name}')
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Service {client.srv_name} call success')
            return future.result()
        else:
            self.get_logger().error(f'Service {client.srv_name} call failed')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = ToolMover()
    try:
        node.move_tool_forward(0.1) # Move 10cm forward
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()