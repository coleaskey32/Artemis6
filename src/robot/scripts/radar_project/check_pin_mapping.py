#!/usr/bin/env python3
import Jetson.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

# Try to get the pin mapping
try:
    pin_data = GPIO.gpio_pin_data
    print("Checking pin mappings...")
    
    for pin in [7, 31]:
        try:
            info = pin_data.get_gpio_info(pin)
            print(f"\nPin {pin}:")
            print(f"  {info}")
        except Exception as e:
            print(f"\nPin {pin}: Error - {e}")
            
    # Try to check channels
    print("\n\nTrying to get channel info...")
    channels = pin_data.get_data()
    print(f"Total channels: {len(channels)}")
    
    for pin in [7, 31]:
        if pin < len(channels):
            print(f"Pin {pin}: {channels[pin]}")
        else:
            print(f"Pin {pin}: Out of range")
            
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()

