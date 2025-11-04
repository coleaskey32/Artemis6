#!/usr/bin/env python3

import rclpy  # pyright: ignore[reportMissingImports]
from rclpy.node import Node  # pyright: ignore[reportMissingImports]
from std_msgs.msg import String  # pyright: ignore[reportMissingImports]
import Jetson.GPIO as GPIO  # type: ignore
import serial  # pyright: ignore[reportMissingModuleSource]
import time

class LidarMonitor(Node):
    def __init__(self):
        super().__init__('lidar_monitor')
        self.get_logger().info('Starting Lidar Monitor Node')

        self.lidar_publisher = self.create_publisher(
            String,
            '/safety_status',
            10
        )

        # Initialize serial connection to Arduino
        try:
            self.serial_port = serial.Serial(
                port='/dev/ttyTHS1',
                baudrate=115200,
                timeout=1.0
            )
            self.get_logger().info('Serial port /dev/ttyTHS1 opened successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.serial_port = None

    def run(self):
        self.get_logger().info('Starting sensor loop...')
        
        if self.serial_port is None:
            self.get_logger().error('Serial port not available, cannot run')
            return
        
        while rclpy.ok():
            try:
                # Read a line from the serial port
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    
                    # Parse the distance data (format: DIST:105.94)
                    if line.startswith('DIST:'):
                        try:
                            distance_str = line.split(':')[1]
                            distance = float(distance_str)
                            
                            self.get_logger().info(f'Distance: {distance} inches')
                            
                            # Determine safety status based on distance
                            if distance < 6:
                                safety_status = 'danger'
                            else:
                                safety_status = 'safe'
                            
                            # Publish the distance data
                            msg = String()
                            msg.data = safety_status
                            self.lidar_publisher.publish(msg)
                            
                        except (ValueError, IndexError) as e:
                            self.get_logger().warn(f'Failed to parse distance: {line} - {e}')
                else:
                    time.sleep(0.01)  # Small delay if no data available
                    
            except Exception as e:
                self.get_logger().error('Error in sensor loop: %s' % e)
                time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = LidarMonitor()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Lidar Monitor Node')
    finally:
        if node.serial_port is not None and node.serial_port.is_open:
            node.serial_port.close()
            node.get_logger().info('Serial port closed')
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()