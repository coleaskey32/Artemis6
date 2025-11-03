#!/usr/bin/env python3
"""
Simple GPIO Pin Tester
Test any pin with HIGH (1) or LOW (0)
"""

import Jetson.GPIO as GPIO  # type: ignore
import sys
import os
import time

def get_gpio_chip_and_line(board_pin):
    """Get the GPIO chip and line number for a board pin"""
    # Common Jetson Nano/Xavier mappings (you may need to adjust)
    # This is approximate - actual mapping depends on your Jetson model
    gpio_map = {
        7: (0, 216),   # GPIO 216
        11: (0, 50),   # GPIO 50
        12: (0, 79),   # GPIO 79
        13: (0, 14),   # GPIO 14
        15: (0, 194),  # GPIO 194
        16: (0, 232),  # GPIO 232
        18: (0, 15),   # GPIO 15
        19: (0, 16),   # GPIO 16
        21: (0, 17),   # GPIO 17
        22: (0, 13),   # GPIO 13
        23: (0, 18),   # GPIO 18
        24: (0, 19),   # GPIO 19
        26: (0, 20),   # GPIO 20
        29: (0, 149),  # GPIO 149
        31: (0, 200),  # GPIO 200
        32: (0, 168),  # GPIO 168
        33: (0, 38),   # GPIO 38
        35: (0, 76),   # GPIO 76
        36: (0, 51),   # GPIO 51
        37: (0, 12),   # GPIO 12
        38: (0, 77),   # GPIO 77
        40: (0, 78),   # GPIO 78
    }
    return gpio_map.get(board_pin, (None, None))

def check_sysfs_gpio(board_pin, expected_state):
    """Check GPIO state directly via sysfs (most reliable method)"""
    chip, line = get_gpio_chip_and_line(board_pin)
    if line is None:
        return "Unknown pin mapping"
    
    # Check if GPIO is exported
    gpio_path = f"/sys/class/gpio/gpio{line}"
    if not os.path.exists(gpio_path):
        return f"GPIO {line} not exported to sysfs"
    
    # Try to read value
    try:
        value_path = f"{gpio_path}/value"
        direction_path = f"{gpio_path}/direction"
        
        with open(direction_path, 'r') as f:
            direction = f.read().strip()
        
        with open(value_path, 'r') as f:
            value = int(f.read().strip())
        
        status = "✅ MATCH" if value == expected_state else "❌ MISMATCH"
        return f"{status} - sysfs shows: {value} (direction: {direction})"
    except Exception as e:
        return f"Error reading sysfs: {e}"

def verify_pin_thoroughly(board_pin, expected_state):
    """Run multiple verification checks on a pin"""
    print(f"\n{'='*50}")
    print(f"🔍 THOROUGH VERIFICATION - Pin {board_pin}")
    print(f"{'='*50}")
    
    expected_str = "HIGH (3.3V)" if expected_state == 1 else "LOW (0V)"
    print(f"Expected: {expected_str}\n")
    
    # Method 1: GPIO library readback
    try:
        readback = GPIO.input(board_pin)
        readback_str = "HIGH" if readback == GPIO.HIGH else "LOW"
        match1 = "✅" if readback == expected_state else "❌"
        print(f"1. GPIO.input() readback: {readback_str} {match1}")
    except Exception as e:
        print(f"1. GPIO.input() readback: ❌ Error: {e}")
    
    # Method 2: Check sysfs
    chip, line = get_gpio_chip_and_line(board_pin)
    if line:
        print(f"2. Sysfs (GPIO{line}): {check_sysfs_gpio(board_pin, expected_state)}")
    else:
        print(f"2. Sysfs: Unknown GPIO mapping for pin {board_pin}")
    
    # Method 3: Suggest LED test
    print(f"\n3. 💡 LED Test (best physical verification):")
    print(f"   - Get an LED + 220Ω resistor")
    print(f"   - LED long leg (anode) → Pin {board_pin}")
    print(f"   - LED short leg (cathode) → Resistor → GND")
    print(f"   - HIGH should light the LED, LOW should turn it off")
    
    # Method 4: Multimeter tips
    print(f"\n4. 🔌 Multimeter Test:")
    print(f"   - Black probe: GND (pin 6, 9, 14, 20, 25, 30, 34, or 39)")
    print(f"   - Red probe: Pin {board_pin}")
    print(f"   - Expected: {expected_str}")
    print(f"   - If reading 0V on HIGH, check:")
    print(f"     • Probes are on correct pins")
    print(f"     • Multimeter is set to DC voltage")
    print(f"     • Pin isn't shorted to ground")
    
    print(f"\n{'='*50}\n")

def main():
    print("\n" + "="*50)
    print("GPIO PIN TESTER - HIGH/LOW ONLY")
    print("="*50)
    print("\n⚠️  IMPORTANT: Run with sudo!")
    print("   sudo python3 pin_tester.py")
    print("\n📊 Multimeter Testing:")
    print("   - HIGH (1) = 3.3V")
    print("   - LOW (0)  = 0V")
    print("   - Black probe on GND (pins 6, 9, 14, 20, 25, 30, 34, 39)")
    print("   - Red probe on signal pin")
    print("\nCommands:")
    print("  <pin> <state>     - Set pin (e.g., '15 1' for HIGH, '32 0' for LOW)")
    print("  read <pin>        - Read back pin state")
    print("  verify <pin>      - Run thorough verification tests")
    print("  blink <pin>       - Blink pin 5 times (good for LED testing)")
    print("  quit / q          - Exit")
    print("="*50)
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    
    print("\n✅ GPIO initialized (BOARD mode)")
    
    active_pins = {}  # pin: current_state
    
    try:
        while True:
            # Show active pins
            if active_pins:
                print("\nActive pins:")
                for pin, state in active_pins.items():
                    state_str = "HIGH" if state == GPIO.HIGH else "LOW"
                    print(f"  Pin {pin}: {state_str}")
            
            # Get command
            cmd = input("\n> ").strip().lower()
            
            if cmd in ['quit', 'q', 'exit']:
                break
            
            # Parse command
            parts = cmd.split()
            if len(parts) < 1:
                continue
            
            # Handle 'read' command
            if parts[0] == 'read' and len(parts) == 2:
                try:
                    pin = int(parts[1])
                    if pin not in active_pins:
                        print(f"⚠️  Pin {pin} not initialized yet")
                    else:
                        state = GPIO.input(pin)
                        state_str = "HIGH (3.3V)" if state == GPIO.HIGH else "LOW (0V)"
                        print(f"📖 Pin {pin} reads: {state_str}")
                except ValueError:
                    print("❌ Usage: read <pin>")
                except Exception as e:
                    print(f"❌ Error reading pin: {e}")
                continue
            
            # Handle 'verify' command
            if parts[0] == 'verify' and len(parts) == 2:
                try:
                    pin = int(parts[1])
                    if pin not in active_pins:
                        print(f"⚠️  Pin {pin} not initialized. Set it to HIGH or LOW first.")
                    else:
                        current_state = 1 if active_pins[pin] == GPIO.HIGH else 0
                        verify_pin_thoroughly(pin, current_state)
                except ValueError:
                    print("❌ Usage: verify <pin>")
                except Exception as e:
                    print(f"❌ Error: {e}")
                continue
            
            # Handle 'blink' command
            if parts[0] == 'blink' and len(parts) == 2:
                try:
                    pin = int(parts[1])
                    if pin not in active_pins:
                        print(f"⚠️  Pin {pin} not initialized. Initializing now...")
                        GPIO.setup(pin, GPIO.OUT)
                        GPIO.output(pin, GPIO.LOW)
                        active_pins[pin] = GPIO.LOW
                    
                    print(f"🔄 Blinking pin {pin} 5 times (0.5s on, 0.5s off)...")
                    original_state = active_pins[pin]
                    
                    for i in range(5):
                        GPIO.output(pin, GPIO.HIGH)
                        print(f"  Blink {i+1}/5: HIGH", end='\r')
                        time.sleep(0.5)
                        GPIO.output(pin, GPIO.LOW)
                        print(f"  Blink {i+1}/5: LOW ", end='\r')
                        time.sleep(0.5)
                    
                    # Restore original state
                    GPIO.output(pin, original_state)
                    active_pins[pin] = original_state
                    print(f"\n✅ Blink complete, restored to original state")
                    print(f"💡 If you saw the LED blink, the pin is working!")
                    
                except ValueError:
                    print("❌ Usage: blink <pin>")
                except Exception as e:
                    print(f"❌ Error: {e}")
                continue
            
            # Handle set pin command
            if len(parts) != 2:
                print("❌ Usage: <pin> <state>, read <pin>, verify <pin>, or blink <pin>")
                continue
            
            try:
                pin = int(parts[0])
                state = int(parts[1])
                
                if state not in [0, 1]:
                    print("❌ State must be 0 (LOW) or 1 (HIGH)")
                    continue
                
                # Setup pin if not already setup
                if pin not in active_pins:
                    try:
                        GPIO.setup(pin, GPIO.OUT)
                        print(f"✅ Pin {pin} initialized as OUTPUT")
                    except Exception as e:
                        print(f"❌ Failed to initialize pin {pin}: {e}")
                        continue
                
                # Set the pin
                gpio_state = GPIO.HIGH if state == 1 else GPIO.LOW
                GPIO.output(pin, gpio_state)
                active_pins[pin] = gpio_state
                
                # Readback to verify
                readback = GPIO.input(pin)
                voltage = "3.3V" if readback == GPIO.HIGH else "0V"
                state_str = "HIGH" if state == 1 else "LOW"
                
                print(f"✅ Pin {pin} set to {state_str} (should be {voltage})")
                
                if readback != gpio_state:
                    print(f"⚠️  WARNING: Readback mismatch! Expected {gpio_state}, got {readback}")
                
            except ValueError:
                print("❌ Invalid input. Use numbers only (e.g., '15 1')")
            except Exception as e:
                print(f"❌ Error: {e}")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted")
    finally:
        # Cleanup - set all pins LOW
        print("\n🧹 Cleaning up...")
        for pin in active_pins:
            GPIO.output(pin, GPIO.LOW)
        GPIO.cleanup()
        print("✅ All pins set to LOW and cleaned up")


if __name__ == '__main__':
    main()

