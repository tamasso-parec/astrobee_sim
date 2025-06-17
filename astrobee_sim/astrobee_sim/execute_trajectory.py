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

from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

from geometry_msgs.msg import Twist, Vector3, PoseArray, Pose
from geometry_msgs.msg import TransformStamped

from tf_transformations import quaternion_matrix
import re
import yaml
import numpy as np
from scipy.interpolate import make_interp_spline

from gz.transport13 import Node as gz_Node

import gz.msgs10 as gz_msgs
from gz.msgs10.pose_pb2 import Pose as gz_pose
from gz.msgs10.boolean_pb2 import Boolean as gz_boolean
from gz.msgs10 import vector3d_pb2 
from gz.msgs10 import quaternion_pb2
from gz.msgs10.stringmsg_pb2 import StringMsg




class TrajectoryFollower(Node):

    def __init__(self):
        super().__init__('RobotRotator')
        self.qos_profile = QoSProfile(
                                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                    history=QoSHistoryPolicy.KEEP_LAST,
                                    depth=10
                                    )
        
        self.gz_node = gz_Node()
        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.trajectory = []
        self.times = []


        self.declare_parameter('robot_name', 'chaser')
        self.declare_parameter('robot_number', 0)

        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.robot_number = self.get_parameter('robot_number').get_parameter_value().integer_value

        
        self.declare_parameter('trajectory_offset', [0.0, 0.0, 0.0])
        self.trajectory_offset = self.get_parameter('trajectory_offset').get_parameter_value().double_array_value


        self.declare_parameter('trajectory_file', 'src/astrobee_sim/resource/chaserOCP.yaml')

        trajectory_file = self.get_parameter('trajectory_file').get_parameter_value().string_value

        self.read_trajectory_file(trajectory_file)
        self.interpolate_trajectory()


        self.start_motion = False

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = 0.0
        self.waypoint_id = 0

        self.service_name = '/world/default/set_pose'

        self.set_pose_client = self.create_client(SetEntityPose, '/world/default/set_pose')

        self.gz_request = gz_pose()
        self.timeout = 10

        # while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')

        self.req = SetEntityPose.Request()

        # while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('SetEntityPose service not available, waiting...')

        self.vel_topic_name = f'/{self.robot_name}/cmd_vel'

        self.body_frame = f'/{self.robot_name}/base_link'
        self.world_frame = "world"

        self.robot_number = 0

        self.ground_truth_pose_sub = self.create_subscription(
            PoseArray,
            '/gazebo_world/object_poses', 
            self.ground_truth_pose_callback, 
            1)
        

        self.rotation_subscirber = self.create_subscription(
            Twist,
            '/target/cmd_vel',
            self.rotation_callback,
            1
        )

        self.is_spinning = False

    def send_gz_request(self):

        service_name = '/world/default/set_pose'
        request = gz_pose()
        # request.data = "Hello world"
        response = gz_boolean()
        timeout = 5000
    
        result, response = self.gz_node.request(service_name, self.gz_request, gz_pose, gz_boolean, timeout)

        # result, response =  self.gz_node.request(self.service_name, self.gz_request, gz_pose, gz_boolean, self.timeout)
        # self.gz_node.request(self.service_name, self.gz_request, pose_pb2, boolean_pb2, self.timeout)

    def read_trajectory_file(self, file_path):
        # Read the trajectory file and extract the rotation angles
        with open(file_path) as res: 

            try: 

                data = yaml.safe_load(res)

                target = data[0]["Target"]
                
                centers = []
                samples = []
                last = 0

                for t in data: 
                    
                    self.times.append(float(t))

                    center = data[t]["center"]

                    centers.append(center)

                    samples = np.array(data[t]["samples"])

                    samples = np.reshape(samples, (-1, 3))

                    
                self.waypoints = np.array(centers)
    
            except yaml.YAMLError as exc:
                print(exc)

    def interpolate_trajectory(self):
        # Interpolate the 3D trajectory using a B-spline
        if len(self.times) < 2 or len(self.waypoints) < 2:
            self.get_logger().warn("Not enough points to interpolate trajectory.")
            return

        times = np.array(self.times)
        traj = np.array(self.waypoints)

        # Ensure trajectory is Nx3
        if traj.ndim == 1:
            traj = traj.reshape(-1, 3)

        self.trajectory = make_interp_spline(times, traj, k=3)

        
    def rotation_callback(self, msg):

        if not self.start_motion:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            self.start_motion= True 

    def sendGazeboRequest(self, position, orientation):
        # Create a request to set the pose of the entity
        self.gz_request.position.x = position.x
        self.gz_request.position.y = position.y
        self.gz_request.position.z = position.z
        self.gz_request.orientation.x = orientation.x
        self.gz_request.orientation.y = orientation.y
        self.gz_request.orientation.z = orientation.z
        self.gz_request.orientation.w = orientation.w

        # Set the entity name and type
        self.gz_request.name = 'chaser'
        self.gz_request.id = 9

        # Call the service to set the pose
        self.send_gz_request()


    def sendRequest(self, position, orientation):



        ent = Entity() 

        ent.id = 9
        ent.name = self.robot_name
        ent.type = 2

        self.req.entity = ent

        # Create a request to set the pose of the entity
        self.req.pose.position.x = position.x
        self.req.pose.position.y = position.y
        self.req.pose.position.z = position.z
        self.req.pose.orientation.x = orientation.x
        self.req.pose.orientation.y = orientation.y
        self.req.pose.orientation.z = orientation.z
        self.req.pose.orientation.w = orientation.w



        self.get_logger().info(f"Setting pose: {self.req.pose.position.x}, {self.req.pose.position.y}, {self.req.pose.position.z}")

        # Call the service to set the pose
        # future = self.set_pose_client.call_async(self.req)
        # rclpy.spin_until_future_complete(self, future)

    def timer_callback(self):
        if self.start_motion :
            delta_t = self.get_clock().now().nanoseconds / 1e9 - self.start_time
            self.get_logger().info(f"Delta time: {delta_t}")
            if delta_t < self.times[-1]:
                pp = self.trajectory(delta_t)
                # Get the current position and orientation from the trajectory
                position = Vector3()
                orientation = geometry_msgs.msg.Quaternion()

                position.x = pp[0] + self.trajectory_offset[0]
                position.y = pp[1] + self.trajectory_offset[1]
                position.z = pp[2] + self.trajectory_offset[2]

                self.get_logger().info(f"New position: {position.x}, {position.y}, {position.z}")
                # Send the request to set the pose
                # self.sendRequest(position, orientation)
                self.sendGazeboRequest(position, orientation)


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

        
        




def main():
    rclpy.init()

    follow_trajectory_robot_node = TrajectoryFollower()

    rclpy.spin(    follow_trajectory_robot_node )


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    follow_trajectory_robot_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
