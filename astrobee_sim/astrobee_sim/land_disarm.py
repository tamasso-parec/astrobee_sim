#!/usr/bin/env python3
import sys

import geometry_msgs.msg
import rclpy
import std_msgs.msg
from rclpy.node import Node
from rclpy.clock import Clock

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition

from geometry_msgs.msg import Twist, Vector3



class LandDisarmPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.qos_profile = QoSProfile(
                                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                    history=QoSHistoryPolicy.KEEP_LAST,
                                    depth=10
                                    )
        

        #Create subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            self.qos_profile)
        
        #Create subscriptions
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            self.qos_profile)
        
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        #creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = .5 # seconds
        # self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)
        self.wait_count = 0
        self.positioning = False
        self.armed = True
        self.landed = False
        self.landing = False

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        
        

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)
    
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command sent")
        self.armed = False
    
    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, param1 = 0.0, param7=0.0) # param7 is altitude in meters
        self.get_logger().info("Land command sent")
        self.landing = True
        # self.takeoff = True

    def set_positioning(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LOITER_UNLIM, param1 = 0.0, param7=0.0) # param7 is altitude in meters

    


    #receives and sets vehicle status values 
    def vehicle_status_callback(self, msg):


        self.nav_state = msg.nav_state
        self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

        if not self.landing:
            self.land()
            self.landing = True
            return

        if self.landing:
            self.get_logger().info("Landing in progress")
            if msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER or msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

                self.get_logger().info("Landing complete. Disarming...")
                self.disarm()
                self.set_positioning()
                self.landing = False
                self.landed = True
                # rclpy.shutdown()
                sys.exit()

        # if self.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND: 
            

        #     if self.landing==False and self.landed == False and self.positioning == False: 
        #         self.get_logger().info("Setting positioning")
        #         self.set_positioning()
        #         self.positioning = True
        #         return
            
            
        #     if self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER and self.landing==False and self.landed == True:
        #         self.get_logger().info("Landed, stopping node")
        #         self.disarm()
        #         sys.exit()
        #         return
            
        #     if self.landing==False and self.landed == False and self.positioning == True: 
        #         self.land()

        # elif self.landing == True:
        #     self.get_logger().info("Landing in progress")
        #     self.landed = True
        #     self.landing = False
        #     return
            

            



        # if self.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LOITER or self.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND and self.landed==False:
        #     self.set_positioning()
            
        #     return 
        

        # if self.landed == False:
        #     self.land() 


        # if self.landed == True and  self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
        #     self.get_logger().info("Landed, stopping node")
        #     self.disarm()
        #     sys.exit()

       
        
        self.wait_count += 1

    #receives and sets vehicle status values 
    def vehicle_local_position_callback(self, msg):


        self.altitude = -msg.z
        # self.get_logger().info(f"Altitude: {msg.z}")




def main():
    rclpy.init()

    land_disarm_publisher = LandDisarmPublisher()

    rclpy.spin(land_disarm_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    land_disarm_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()