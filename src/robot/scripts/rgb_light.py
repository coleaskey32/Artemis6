#!/usr/bin/env python3
"""
RGB LED Tester for Jetson Orin Nano
Tests RGB LED connected to GPIO pins
Pins: [29 (Red), 31 (Green), 33 (Blue)]
"""

import Jetson.GPIO as GPIO
import time
import sys

class RGBLEDTester:
    def __init__(self, red_pin=29, green_pin=31, blue_pin=33):
        """Initialize RGB LED on specified GPIO pins"""
        self.red_pin = red_pin
        self.green_pin = green_pin
        self.blue_pin = blue_pin
        
        # Setup GPIO
        GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
        GPIO.setwarnings(False)
        
        # Setup pins as outputs
        GPIO.setup(self.red_pin, GPIO.OUT)
        GPIO.setup(self.green_pin, GPIO.OUT)
        GPIO.setup(self.blue_pin, GPIO.OUT)
        
        # Create PWM instances for smooth color transitions (100 Hz)
        self.red_pwm = GPIO.PWM(self.red_pin, 100)
        self.green_pwm = GPIO.PWM(self.green_pin, 100)
        self.blue_pwm = GPIO.PWM(self.blue_pin, 100)
        
        # Start PWM with 0% duty cycle (off)
        self.red_pwm.start(0)
        self.green_pwm.start(0)
        self.blue_pwm.start(0)
        
        print("🌈 RGB LED Tester Initialized")
        print(f"   Red:   Pin {self.red_pin}")
        print(f"   Green: Pin {self.green_pin}")
        print(f"   Blue:  Pin {self.blue_pin}")
        print()
    
    def set_color(self, red, green, blue):
        """
        Set RGB color using values 0-100 (percentage)
        red, green, blue: 0-100
        """
        self.red_pwm.ChangeDutyCycle(red)
        self.green_pwm.ChangeDutyCycle(green)
        self.blue_pwm.ChangeDutyCycle(blue)
    
    def off(self):
        """Turn off all LEDs"""
        self.set_color(0, 0, 0)
    
    def test_individual_colors(self, duration=2):
        """Test each color channel individually"""
        print("🔴 Testing RED...")
        self.set_color(100, 0, 0)
        time.sleep(duration)
        
        print("🟢 Testing GREEN...")
        self.set_color(0, 100, 0)
        time.sleep(duration)
        
        print("🔵 Testing BLUE...")
        self.set_color(0, 0, 100)
        time.sleep(duration)
        
        self.off()
        time.sleep(0.5)
    
    def test_mixed_colors(self, duration=2):
        """Test mixed colors"""
        colors = [
            (100, 100, 0, "🟡 Yellow (Red + Green)"),
            (0, 100, 100, "🔵 Cyan (Green + Blue)"),
            (100, 0, 100, "🟣 Magenta (Red + Blue)"),
            (100, 100, 100, "⚪ White (All)"),
        ]
        
        for r, g, b, name in colors:
            print(name)
            self.set_color(r, g, b)
            time.sleep(duration)
        
        self.off()
        time.sleep(0.5)
    
    def color_fade(self, duration=5):
        """Smooth color fade through spectrum"""
        print("🌈 Color fade through spectrum...")
        steps = 100
        delay = duration / steps
        
        for i in range(steps):
            # Create a smooth rainbow effect using sine waves
            r = (1 + abs(2 * (i / steps) - 1)) * 50
            g = (1 - abs(2 * ((i / steps + 0.33) % 1) - 1)) * 100
            b = (1 - abs(2 * ((i / steps + 0.66) % 1) - 1)) * 100
            
            self.set_color(r, g, b)
            time.sleep(delay)
        
        self.off()
        time.sleep(0.5)
    
    def breathing_effect(self, color=(100, 0, 100), cycles=3, duration=2):
        """Breathing effect - fade in and out"""
        print(f"💨 Breathing effect (Magenta)...")
        r, g, b = color
        
        for _ in range(cycles):
            # Fade in
            for i in range(0, 101, 5):
                self.set_color(r * i / 100, g * i / 100, b * i / 100)
                time.sleep(duration / 40)
            
            # Fade out
            for i in range(100, -1, -5):
                self.set_color(r * i / 100, g * i / 100, b * i / 100)
                time.sleep(duration / 40)
        
        self.off()
        time.sleep(0.5)
    
    def strobe_effect(self, color=(100, 100, 100), flashes=10, speed=0.1):
        """Strobe/flash effect"""
        print("⚡ Strobe effect...")
        r, g, b = color
        
        for _ in range(flashes):
            self.set_color(r, g, b)
            time.sleep(speed)
            self.off()
            time.sleep(speed)
        
        time.sleep(0.5)
    
    def run_all_tests(self):
        """Run complete test suite"""
        print("\n" + "="*50)
        print("🧪 Starting RGB LED Test Suite")
        print("="*50 + "\n")
        
        try:
            print("Test 1: Individual Colors")
            print("-" * 30)
            self.test_individual_colors(duration=2)
            
            print("\nTest 2: Mixed Colors")
            print("-" * 30)
            self.test_mixed_colors(duration=2)
            
            print("\nTest 3: Color Fade")
            print("-" * 30)
            self.color_fade(duration=5)
            
            print("\nTest 4: Breathing Effect")
            print("-" * 30)
            self.breathing_effect(cycles=3)
            
            print("\nTest 5: Strobe Effect")
            print("-" * 30)
            self.strobe_effect(flashes=10)
            
            print("\n" + "="*50)
            print("✅ All tests completed successfully!")
            print("="*50 + "\n")
            
        except KeyboardInterrupt:
            print("\n⚠️  Test interrupted by user")
        finally:
            self.cleanup()
    
    def interactive_mode(self):
        """Interactive RGB control"""
        print("\n" + "="*50)
        print("🎮 Interactive RGB Control")
        print("="*50)
        print("Enter RGB values (0-100) separated by spaces")
        print("Example: 100 50 25")
        print("Commands: 'quit' or 'q' to exit, 'off' to turn off")
        print("="*50 + "\n")
        
        try:
            while True:
                user_input = input("RGB> ").strip().lower()
                
                if user_input in ['quit', 'q', 'exit']:
                    break
                elif user_input == 'off':
                    self.off()
                    print("LEDs turned off")
                else:
                    try:
                        values = user_input.split()
                        if len(values) == 3:
                            r, g, b = [max(0, min(100, int(v))) for v in values]
                            self.set_color(r, g, b)
                            print(f"Set: R={r}% G={g}% B={b}%")
                        else:
                            print("❌ Please enter exactly 3 values")
                    except ValueError:
                        print("❌ Invalid input. Enter three numbers (0-100)")
        
        except KeyboardInterrupt:
            print("\n⚠️  Exiting interactive mode")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up GPIO on exit"""
        print("\n🧹 Cleaning up GPIO...")
        self.off()
        self.red_pwm.stop()
        self.green_pwm.stop()
        self.blue_pwm.stop()
        GPIO.cleanup()
        print("👋 Goodbye!")


def main():
    """Main function with menu"""
    print("\n" + "🌈" * 25)
    print("RGB LED TESTER")
    print("🌈" * 25 + "\n")
    
    # Initialize RGB LED
    rgb = RGBLEDTester(red_pin=29, green_pin=31, blue_pin=33)
    
    print("Select mode:")
    print("  1. Run all tests (auto)")
    print("  2. Interactive mode (manual control)")
    print("  3. Quick test (primary colors only)")
    print()
    
    try:
        choice = input("Enter choice (1-3): ").strip()
        
        if choice == '1':
            rgb.run_all_tests()
        elif choice == '2':
            rgb.interactive_mode()
        elif choice == '3':
            print("\n🧪 Quick Test")
            rgb.test_individual_colors(duration=1)
            print("✅ Quick test complete!")
            rgb.cleanup()
        else:
            print("❌ Invalid choice. Running all tests by default.")
            rgb.run_all_tests()
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        rgb.cleanup()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        rgb.cleanup()


if __name__ == '__main__':
    main()
