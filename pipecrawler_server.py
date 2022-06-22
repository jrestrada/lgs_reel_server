#!/usr/bin/env python3

"""
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Description:
Action Server that controls a pipe crawler's pneumatics
through raspberry pi pins, relay, and solenoid valves
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Test with: 

ros2 action send_goal --feedback crawl_through_pipe pipecrawler/action/Crawlaction '{crawlercommand: {crawlpattern: [2,5,3,6,1,4], continuous: True}}'
ros2 action send_goal --feedback crawl_through_pipe pipecrawler/action/Crawlaction '{crawlercommand: {crawlpattern: [0,0,0,0,0,0], continuous: False}}'

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Subscription Topics:
    pipecrawler/Crawlpattern
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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


def gb__on():
    print("GB on Test")

def gb__off():
    print("GB off test")

def e__on():
    print("e on Test")

def e__off():
    print("e off test")

def gf__on():
    print("GF on Test")

def gf__off():
    print("GF off test")

def none():
    print("none")

gripper_command_names = {
    1:"gb.on",
    2:"gb.off",
    3:"e.on()",
    4:"e.off()",
    5:"gf.on()",
    6:"gf.off()",
    0:"none()"
}

gripper_commands = {
    1:gb__on,
    2:gb__off,
    3:e__on,
    4:e__off,
    5:gf__on,
    6:gf__off,
    0:none
}

def executeCommands(command_list):
    for i in range(len(command_list)):
            gripper_commands[command_list[i]]()
            time.sleep(0.2)

class CrawlactionServer(Node):
    def __init__(self):
        super().__init__('pipecrawler_server')  # Node instance name

        self._action_server = ActionServer(
            self,                               # Node
            Crawlaction,                        # Action Type, imported
            'crawl_through_pipe',               # Action Name (Must be matched in other nodes or commands)
            self.execute_callback_funct)        # Callback Function to execute

        self._crawl_counter = 0

    def execute_callback_funct(self, goal_handle):
        current_commands_list = goal_handle.request.crawlercommand.crawlpattern

        if goal_handle.request.crawlercommand.continuous:
            self.get_logger().info("Executing continuous crawl command")
            while True:
                executeCommands(current_commands_list)
                self._crawl_counter += 1
                feedback_msg = Crawlaction.Feedback()
                feedback_msg.count = self._crawl_counter
                goal_handle.publish_feedback(feedback_msg)
                
        else: 
            self.get_logger().info("Executing single command")
            executeCommands(current_commands_list)
            self._crawl_counter+=1
            feedback_msg = Crawlaction.Feedback()
            feedback_msg.count = self._crawl_counter
            goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()
        resultant = Crawlaction.Result()
        resultant.successfulcrawl = True
        return resultant

def main(args=None):
    rclpy.init(args=args)
   
    try: 
        # Create the Action Server Node instance
        pipecrawler_server_instance = CrawlactionServer()
        
        # Set up mulithreading
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(pipecrawler_server_instance)
        try:
            # Spin the nodes to execute the callbacks
            executor.spin()
        finally:
            # Shutdown the nodes
            executor.shutdown()
            pipecrawler_server_instance.destroy_node()
    finally:
        # Shutdown
        rclpy.shutdown()

if __name__ == '__main__':
    main()
 