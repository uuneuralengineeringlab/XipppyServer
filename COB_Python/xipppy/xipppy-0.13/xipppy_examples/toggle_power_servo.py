"""
This script enables experiment with toggling power servo of an implant on
and off. Follow the prompts after running this script to toggle.
"""
from xipppy.transceiver import ensure_implant_booted
from xipppy import xipppy_open, transceiver_power_servo_enable


def main():
    with xipppy_open():
        ensure_implant_booted()
    servo_enable = False
    print("Press ctrl+c to to quit.")
    while True:
        with xipppy_open():
            transceiver_power_servo_enable(servo_enable)
            if servo_enable:
                input('Power servo is on. (Press enter to toggle)')
            else:
                input('Power servo is off. (Press enter to toggle)')
        servo_enable = not servo_enable


if __name__ == '__main__':
    main()
