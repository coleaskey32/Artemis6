#!/usr/bin/env python3

import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import String    # type: ignore
import Jetson.GPIO as GPIO  # type: ignore
import time

class LedStatus(Node):
    def __init__(self):
        super().__init__('led_status')
        self.get_logger().info('Starting LED Status Node')
        
        # Setup GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        self.led_pins = {
            'green': 31,
            'red': 7,
        }
        
        # Setup GPIO pins
        for pin in self.led_pins.values():
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        # Track current status
        self.current_status = None
        self.red_led_state = False
        
        # Create subscription to safety status
        self.safety_status_subscriber = self.create_subscription(
            String,
            '/safety_status',
            self.safety_status_callback,
            10
        )
        
        # Create a timer for blinking red LED when in danger mode
        self.blink_timer = self.create_timer(0.5, self.blink_callback)

    def safety_status_callback(self, msg):
        """Handle incoming safety status messages"""
        new_status = msg.data
        
        if new_status != self.current_status:
            self.get_logger().info(f'Status update received: {new_status}')
            self.current_status = new_status
            
            if new_status == 'safe':
                # Turn on green, turn off red
                GPIO.output(self.led_pins['green'], GPIO.HIGH)
                GPIO.output(self.led_pins['red'], GPIO.LOW)
                self.red_led_state = False
                self.get_logger().info('✓ Green LED ON - Status SAFE')
                
            elif new_status == 'danger':
                # Turn off green, red will blink via timer
                GPIO.output(self.led_pins['green'], GPIO.LOW)
                self.get_logger().warn('⚠ DANGER MODE - Red LED blinking')

    def blink_callback(self):
        """Timer callback to blink red LED when in danger mode"""
        if self.current_status == 'danger':
            # Toggle red LED
            if self.red_led_state:
                GPIO.output(self.led_pins['red'], GPIO.LOW)
                self.red_led_state = False
            else:
                GPIO.output(self.led_pins['red'], GPIO.HIGH)
                self.red_led_state = True

def main(args=None):
    rclpy.init(args=args)
    node = LedStatus()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LED Status Node')
    finally:
        # Turn off all LEDs
        GPIO.output(7, GPIO.LOW)
        GPIO.output(31, GPIO.LOW)
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
