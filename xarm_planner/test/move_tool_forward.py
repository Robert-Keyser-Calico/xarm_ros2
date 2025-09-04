#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
import time
import threading

class ToolMover(Node):
    def __init__(self):
        super().__init__('tool_mover')
        
        # Start servo server
        self.servo_start_client = self.create_client(Trigger, '/servo_server/start_servo')
        while not self.servo_start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        request = Trigger.Request()
        future = self.servo_start_client.call_async(request)
        
        # Use a separate thread to spin the node for the service call
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self)
        
        # Spin the executor in a separate thread
        self.executor_thread = threading.Thread(target=executor.spin, daemon=True)
        self.executor_thread.start()

        while not future.done():
            time.sleep(0.1)

        if future.result() and future.result().success:
            self.get_logger().info("Servo server started successfully")
        else:
            self.get_logger().error("Failed to start servo server")

        self.twist_pub = self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)

    def move_tool_forward(self, distance, speed=0.1):
        self.get_logger().info(f"Moving tool forward by {distance}m at {speed}m/s")
        
        if speed == 0:
            self.get_logger().error("Speed cannot be zero.")
            return
            
        duration = abs(distance / speed)
        
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = 'link_eef'
        twist_msg.twist.linear.x = speed if distance > 0 else -speed
        
        start_time = self.get_clock().now()
        
        while rclpy.ok() and (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.twist_pub.publish(twist_msg)
            time.sleep(0.02) # Corresponds to 50hz
            
        # Stop the robot
        stop_msg = TwistStamped()
        stop_msg.header.frame_id = 'link_eef'
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        self.twist_pub.publish(stop_msg)
        self.get_logger().info("Movement finished.")

def main(args=None):
    rclpy.init(args=args)
    node = ToolMover()
    
    # The node is now spinning in a background thread, 
    # so we can directly call the methods.
    
    time.sleep(1.0) # Give some time for connections to be established
    try:
        # Move 10cm forward
        node.move_tool_forward(0.1) 
        time.sleep(1.0)
        # Move 10cm backward
        node.move_tool_forward(-0.1)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        node.executor_thread.join() # Wait for the executor thread to finish

if __name__ == '__main__':
    main()