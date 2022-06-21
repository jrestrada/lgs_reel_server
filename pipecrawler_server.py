#!/usr/bin/env python3

"""
Description:
Action Server that controls a pipe crawler's pneumatics
through raspberry pi pins, relay, and solenoid valves

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Subscription Topics:
    pipecrawler/Crawlpattern

Publishing Topics:
    
"""

from argparse import Action
from os import pipe
import time
import rclpy
from pipecrawler.action import Crawlaction
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int16

class CrawlactionServer(Node):
    def __init__(self):
        super().__init__('pipecrawler_server') # node name

        self._action_server = ActionServer(
            self,                               # Node
            Crawlaction,                        # Action Type
            'pipecrawler_server', 
            self.execute_callback_funct)

        self.count_publisher = self.create_publisher(
            Int16, 
            'crawl_count_topic', 
            10) 
        
        self._crawl_counter = 0

    def execute_callback_funct(self, goal_handle):
        """
        action server callback to execute 
        """
        self.get_logger().info('executing goal')

        # Interim Feedback
        feedback_msg = Crawlaction.Feedback()
        feedback_msg.count = self._crawl_counter
        goal_handle.publish_feedback(feedback_msg)
        
        self._crawl_counter += 1




def main(args=None):
    rclpy.init(args=args)
    try: 
   
        # Create the Action Server Node
        pipecrawler_server = CrawlactionServer()

        # Set up mulithreading
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(pipecrawler_server)

        try:
            # Spin the nodes to execute the callbacks
            executor.spin()
        finally:
            # Shutdown the nodes
            executor.shutdown()
            pipecrawler_server.destroy_node()
 
    finally:
        # Shutdown
        rclpy.shutdown()

if __name__ == '__main__':
    crawl_count = 0
    main()
 