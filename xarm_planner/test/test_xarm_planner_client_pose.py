#!/usr/bin/env python3

# Copyright 2021 UFACTORY Inc. All Rights Reserved.
#
# Software License Agreement (BSD License)
#
# Author: Vinman <vinman.cub@gmail.com>

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from xarm_msgs.srv import PlanPose, PlanExec

class XArmPlannerClientPose(Node):
    def __init__(self):
        super().__init__('test_xarm_planner_client_pose_py')
        self.pose_plan_client = self.create_client(PlanPose, 'xarm_pose_plan')
        self.exec_plan_client = self.create_client(PlanExec, 'xarm_exec_plan')

        self.target_poses = [
            self.create_pose(0.3, -0.1, 0.2, 1.0, 0.0, 0.0, 0.0),
            self.create_pose(0.3, 0.1, 0.2, 1.0, 0.0, 0.0, 0.0),
            self.create_pose(0.3, 0.1, 0.4, 1.0, 0.0, 0.0, 0.0),
            self.create_pose(0.3, -0.1, 0.4, 1.0, 0.0, 0.0, 0.0),
        ]

    def create_pose(self, x, y, z, ox, oy, oz, ow):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = ox
        pose.orientation.y = oy
        pose.orientation.z = oz
        pose.orientation.w = ow
        return pose

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

    def run(self):
        while rclpy.ok():
            for pose in self.target_poses:
                self.call_pose_plan(pose)
                self.call_exec_plan()

def main(args=None):
    rclpy.init(args=args)
    node = XArmPlannerClientPose()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
