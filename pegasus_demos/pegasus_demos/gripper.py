#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from pegasus_msgs.srv import SetMode
from pegasus_msgs.msg import AutopilotStatus, ControlMotors

import time

class Drone(Node):

    def __init__(self, id):

        super().__init__('drone_api_' + str('id'))

        self.id = id
        self.namespace = 'drone'

        # Create the service clients for the drone
        self.set_autopilot_srv = self.create_client(SetMode, '/drone' + str(id) +'/autopilot/change_mode')
        print('Initializing service: /drone' + str(id) +'/autopilot/change_mode')
        while not self.set_autopilot_srv.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('service not available, waiting again...')

        # Create subscriptions
        #self.create_subscription(AutopilotStatus, '/drone' + str(id) + '/autopilot/status', self.autopilot_status_callback, qos_profile_sensor_data)

        # Create publishers
        self.publisher_ = self.create_publisher(ControlMotors, '/drone' + str(id) + '/fmu/in/motors', 10)

        self.timer = None

        # Requests messages
        self.set_mode_req = SetMode.Request()
        #self.motors_req = Waypoint.Request()

    def set_autopilot_mode(self, mode='DisarmMode'):

        self.get_logger().info('Setting autopilot mode to: %s' % mode)
            
        # Set the mode request
        self.set_mode_req.mode = mode
        
        # Make an async request
        self.future = self.set_autopilot_srv.call_async(self.set_mode_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def publish_motor_data(self):
        msg = ControlMotors()
        msg.motor = [1.0] * 8
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicado: {msg.motor}')


def main(args=None):

    # Initiate the ROS2 nodes
    rclpy.init(args=args)

    # Create the drones
    drone1 = Drone(1)

    height = -1.30

    drone1.set_autopilot_mode('ArmMode')

    time.sleep(5)

    timer_period = 2  # seconds
    drone1.timer = drone1.create_timer(timer_period, drone1.publish_motor_data)
    rclpy.spin(drone1)

    # Shutdown the demo
    drone1.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()