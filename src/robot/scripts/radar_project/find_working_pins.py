#!/usr/bin/env python3
import Jetson.GPIO as GPIO
import time

# Common GPIO pins on Jetson Orin Nano (BOARD mode)
common_pins = [7, 11, 12, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26, 29, 31, 32, 33, 35, 36, 37, 38, 40]

print("Testing common GPIO pins on Jetson Orin Nano...")
print("=" * 60)

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

working_pins = []

for pin in common_pins:
    try:
        GPIO.setup(pin, GPIO.OUT)
        print(f"Pin {pin:2d}: ✓ Setup successful - testing...")
        
        # Test toggling the pin
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(pin, GPIO.LOW)
        time.sleep(0.1)
        
        working_pins.append(pin)
        print(f"         ✓ Toggle test passed!")
        
    except Exception as e:
        print(f"Pin {pin:2d}: ✗ Failed - {str(e)[:60]}")
    finally:
        try:
            GPIO.cleanup(pin)
        except:
            pass

print("\n" + "=" * 60)
print(f"\nWorking GPIO pins: {working_pins}")
print(f"Total working pins: {len(working_pins)}")

if len(working_pins) >= 2:
    print(f"\n✓ You can use pins {working_pins[0]} and {working_pins[1]} for your LEDs!")
elif len(working_pins) == 1:
    print(f"\n⚠  Only {working_pins[0]} is working. Need to configure more pins.")
else:
    print("\n✗ No working GPIO pins found. Need to configure pinmux.")

GPIO.cleanup()

