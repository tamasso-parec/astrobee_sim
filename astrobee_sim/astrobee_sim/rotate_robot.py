#!/usr/bin/env python3
import sys

import geometry_msgs.msg
import rclpy
import std_msgs.msg
from rclpy.node import Node
from rclpy.clock import Clock

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Twist, Vector3, PoseArray
from geometry_msgs.msg import TransformStamped

from tf_transformations import quaternion_matrix
import re



class RobotRotator(Node):

    def __init__(self):
        super().__init__('RobotRotator')
        self.qos_profile = QoSProfile(
                                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                    history=QoSHistoryPolicy.KEEP_LAST,
                                    depth=10
                                    )
        
        
        self.tf_broadcaster = TransformBroadcaster(self)


        self.declare_parameter('robot_name', 'astrobee')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.vel_topic_name = f'/{self.robot_name}/cmd_vel'
        # self.vel_topic_name = '/cmd_vel'
        self.body_frame = "base_link"
        self.world_frame = "world"

        self.robot_number = 0

        match = re.search(r'_(\d+)$', self.robot_name)
        if match:
            self.robot_number = int(match.group(1))
        


        self.ground_truth_pose_sub = self.create_subscription(
            PoseArray,
            '/gazebo_world/object_poses', 
            self.ground_truth_pose_callback, 
            1)
        
        self.rotation_publisher = self.create_publisher(
            Twist,
            self.vel_topic_name,
            1
        )

        self.is_spinning = False

        
        # Declare parameters for linear and angular velocities as 3D vectors
        self.declare_parameter('linear_velocity', [0.0, 0.0, 0.0])
        self.declare_parameter('angular_velocity', [0.0, 0.0, 1.0])

        # Get parameter values as lists
        self.linear_velocity = self.get_parameter('linear_velocity').get_parameter_value().double_array_value
        self.angular_velocity = self.get_parameter('angular_velocity').get_parameter_value().double_array_value

        
    

    def ground_truth_pose_callback(self, msg):
        # This function is called when a new PoseArray message is received
        # You can process the pose data here
        # self.get_logger().info(f"Ground truth poses received: {len(msg.poses)} poses")

        astrobee_pose = msg.poses[self.robot_number]

        # Extract position and orientation from the pose
        position = astrobee_pose.position
        orientation = astrobee_pose.orientation

        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.body_frame

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        t.transform.translation.z = position.z

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        t.transform.rotation.x = orientation.x
        t.transform.rotation.y = orientation.y
        t.transform.rotation.z = orientation.z
        t.transform.rotation.w = orientation.w

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

        # if (not self.is_spinning):
                
        rotMat = quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
        # rotated_linear_velocity = rotMat[:3, :3].T @ self.linear_velocity  # Rotate linear velocity
        rotated_linear_velocity = rotMat[:3, :3].T@ self.linear_velocity  # Rotate linear velocity
        rotated_angular_velocity = rotMat[:3, :3].T @ self.angular_velocity  # Rotate around z-axis

        self.get_logger().info(f"Sending spin command with linear velocity: {rotated_linear_velocity} and angular velocity: {rotated_angular_velocity}")

        # Create a Twist message to rotate the robot
        twist = Twist()
        twist.linear.x =rotated_linear_velocity[0]
        twist.linear.y =rotated_linear_velocity[1]
        twist.linear.z =rotated_linear_velocity[2]
        twist.angular.x = rotated_angular_velocity[0]
        twist.angular.y = rotated_angular_velocity[1]
        twist.angular.z = rotated_angular_velocity[2]

        # Publish the Twist message to rotate the robot
        self.rotation_publisher.publish(twist)
        self.is_spinning = True
        




def main():
    rclpy.init()

    rotate_robot_node = RobotRotator()

    rclpy.spin(    rotate_robot_node )


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rotate_robot_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
