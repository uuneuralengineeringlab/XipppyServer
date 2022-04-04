"""
This example demonstrates how to maintain a specific IR LED light level and
recording ADC range. This script would have to run continuously to monitor,
and maintain the light level. The reason is that a MIRA implant cannot
persist its internal state whenever it loses power. This script monitors the
light output level and resets the desired light level whenever power loss is
detected.

Running this script effectively increases the operating depth of the implant.
The trade-off is more power consumption. However, the user will notice a
hysteresis effect with depth. The effective depth is increased, but if the link
is lost, the depth at the default light level will need to be reached in order
to form a link again, at which point the IR level is immediately boosted.
"""
import argparse

from xipppy import xipppy_open
from xipppy.mira import MiraImplantCmdMode
from xipppy.transceiver import (
    transceiver_get_power_state, POWER_STATE_R3, ensure_implant_booted,
    transceiver_status, transceiver_get_ir_led_level, TransceiverStatus
)

# MIRA implant IR LED output level can be set from 0 (off) to 15 (maximum)
# the higher the IR output level the deeper the implant can be to get
# measurements from it. However, it consumes more power thus reducing the
# range at which it can be powered.
DEFAULT_IR_LED_TARGET_LEVEL = 15

# Use the following table to select ADC ranges.
# Value          uV/bit         Range uV
# --------------------------------------------
# 0            0.125          +/- 256
# 1            0.25           +/- 512
# 2            0.5            +/- 1024
# 3            1.0            +/- 2048
# 4            2.0            +/- 4096
# 5 (default)  4.0            +/- 8192
# 6            8.0            +/- 16384
DEFAULT_ADC_TARGET_LEVEL = 5


def maintain_ir_and_adc_level_forever(adc: int, ir: int):
    print('Recording ADC setpoint: {}'.format(adc))
    print('IR LED setpoint: {}'.format(ir))
    print('Pass --help flag to learn how to override these values.')
    with xipppy_open():
        print('Searching for implant.')
        implant_booted = ensure_implant_booted()
        if implant_booted:
            print('Ctrl+C to quit.')
            while True:
                # Check if we've caught the implant rebooting. If we have,
                # send the target light level as soon as it's done.
                if not is_implant_booted():
                    print('Waiting for implant to boot.')
                    block_until_booted()
                    set_ir_and_adc_to_target(target_ir=ir, target_adc=adc)
                # Handle the case where the implant rebooted, but we missed it.
                level = transceiver_get_ir_led_level()
                if was_ir_led_level_reset(level, target_ir=ir):
                    print('IR level was reset to {}'.format(level))
                    set_ir_and_adc_to_target(target_ir=ir, target_adc=adc)
                    adc_res = (transceiver_status(0)[
                        TransceiverStatus.MICS_CONF_LED_PROM] & 0x00F0) >> 4
                    print("adc resolution= {}".format(adc_res))
        else:
            print(
                "Failed to boot implant. Ensure it is in range of the "
                "transceiver."
            )


def was_ir_led_level_reset(level, target_ir):
    return level != target_ir


def set_ir_and_adc_to_target(target_ir, target_adc):
    print("Setting IR to target {}".format(target_ir))
    with MiraImplantCmdMode() as implant:
        implant.set_implant_ir_led_output(
            target_ir
        )
        implant.set_implant_adc_res_range(target_adc)


def block_until_booted():
    while True:
        if is_implant_booted():
            break


def is_implant_booted():
    state = transceiver_get_power_state()
    return state == POWER_STATE_R3['servo_enable']


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        '--adc', type=int, default=DEFAULT_ADC_TARGET_LEVEL,
        help=(
            'Use the following table to select ADC ranges.\n'
            'Value          uV/bit         Range uV\n'
            '--------------------------------------------\n'
            '0            0.125          +/- 256\n'
            '1            0.25           +/- 512\n'
            '2            0.5            +/- 1024\n'
            '3            1.0            +/- 2048\n'
            '4            2.0            +/- 4096\n'
            '5 (default)  4.0            +/- 8192\n'
            '6            8.0            +/- 16384\n'
        )
    )
    parser.add_argument(
        '--ir', type=int, default=DEFAULT_IR_LED_TARGET_LEVEL,
        help=(
            'Set the IR LED light level coming from the implant. Higher\n'
            'values consume more power but increase depth. This is a value\n'
            'in the range 0 to 15. But, it is not recommended to use values\n'
            '0-3 because they are not useful in practice. The implant default\n'
            'is 8.'
        )
    )
    args = parser.parse_args()
    maintain_ir_and_adc_level_forever(adc=args.adc, ir=args.ir)
