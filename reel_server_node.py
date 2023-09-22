#!/usr/bin/env python3

"""
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Description: 
Action Server that distributes commands to reel through cmd_vel and mechanical winder through serial communication
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Test with: 

ros2 action send_goal --feedback turn_reel lgs_interfaces/action/Reel '{command: {velocity: 1, interval: '2.0', continuous: True}}'
ros2 action send_goal --feedback turn_reel lgs_interfaces/action/Reel '{command: {velocity: 1, interval: '2.0', continuous: False}}'

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
"""
import math
import threading
from argparse import Action
from os import pipe
import serial
import time
import rclpy
from lgs_interfaces.action import Reel
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from std_msgs.msg import Int16
from std_msgs.msg import String

sign = lambda x: math.copysign(1, x)

class ReelActionServer(Node):
    def __init__(self):
        super().__init__('reel_server')  # Node instance name ()must be matched)
        self.get_logger().info('Initializing reel control node')
        self.ser = serial.Serial('/dev/arduino_nano', 9600, timeout=1)
        self.ser.reset_input_buffer()
        self._revolution_counter = 0
        self._previous_dir = 0
        self._goal_lock   = threading.Lock()
        self._goal_handle = None
        self._publisher   = self.create_publisher(Twist, 'cmd_vel', 10)
        self._cmd_vel_msg = Twist()
        self._action_server = ActionServer(
            self,                                           
            Reel,                                     
            'turn_reel',                              
            execute_callback =self.execute_callback,      
            goal_callback = self.goal_callback,
            handle_accepted_callback =self.handle_accepted_callback,
            cancel_callback =self.cancel_callback,
            callback_group = ReentrantCallbackGroup())
    
    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def publish_messages(self, message, newdir = 0):
        if (sign(message) != self._previous_dir and message != 0):
            newdir = 1
            self._previous_dir = sign(message)

        self._cmd_vel_msg.linear.x = float(message)
        self.get_logger().info('Publishing cmd_vel: "%s"' % self._cmd_vel_msg.linear.x)
        self._publisher.publish(self._cmd_vel_msg)
        t_units_abs = abs(message)
        pub_str = "<Hello," + str(newdir) + "," + str(t_units_abs)  + ">"
        pub_bytes = pub_str.encode('utf-8')
        self.ser.write(pub_bytes)
        time.sleep(self._goal_handle.request.command.interval)

    def execute_callback(self, goal_handle):
        if goal_handle.request.command.continuous:                                                      
            self.get_logger().info("Executing continuous reel command")
            while goal_handle.is_active:     
                
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Cancelling previous goal')
                
                self.publish_messages(goal_handle.request.command.velocity)
                self._revolution_counter += 1
                feedback_msg = Reel.Feedback()
                feedback_msg.turns = self._revolution_counter
                goal_handle.publish_feedback(feedback_msg)

        else: 
            self.get_logger().info("Executing single command")
            self.publish_messages(goal_handle.request.command.velocity)
            feedback_msg = Reel.Feedback()
            feedback_msg.turns = self._revolution_counter
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info("Goal Succeeded")
            goal_handle.succeed()
        
        resultant = Reel.Result()
        resultant.successfulturn = True
        return resultant

def main(args=None):
    rclpy.init(args=args)
   
    try: 
        # Create the Action Server Node instance
        reel_server_instance = ReelActionServer()
        
        # Set up mulithreading
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(reel_server_instance)
        try:
            # Spin the nodes to execute the callbacks
            executor.spin()
        finally:
            # Shutdown the nodes
            executor.shutdown()
            reel_server_instance.destroy_node()
    finally:
        # Shutdown
        rclpy.shutdown()

if __name__ == '__main__':
    main()
 
