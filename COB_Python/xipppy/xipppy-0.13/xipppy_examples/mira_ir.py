"""
This example will cycle through all of the IR LED output levels in an
infinite loop. A change occurs ever 3 seconds. This example does not start
nipexec; but, it will open xipppy and prepare the implant to receive
commands. Upon exiting with ctrl+c it will restore the transceiver to normal
operation (exiting command mode) and close xipppy.
"""
import time
from itertools import count

from xipppy.transceiver import ensure_implant_booted, TransceiverStatus
from xipppy import xipppy_open
from xipppy.mira import MiraImplantCmdMode


def cycle_mira_ir_forever():
    with xipppy_open():
        implant_booted = ensure_implant_booted()
    if implant_booted:
        # Below we show two examples of putting the implant in command mode,
        # one where we explicitly open xipppy before we put the implant into
        # command mode and one where we do not. MiraImplantCmdMode will reuse
        # an already open xipppy context, if one is not already open it
        # creates its own.
        with xipppy_open():
            with MiraImplantCmdMode() as implant:
                implant.set_implant_ir_led_output(3)
                implant.set_implant_ir_led_output(implant.max_ir_led_output)

        # Cycle through all IR LED output levels forever, once every 3 seconds.
        with MiraImplantCmdMode() as implant:
            print('Ctrl+C to quit.')
            for i in count():
                if i < 3:
                    # Skip low light levels because they are problematic to
                    # get any data even in open air.
                    continue
                implant.set_implant_ir_led_output(
                    divmod(i + 1, implant.max_ir_led_output + 1)[1]
                )
                time.sleep(3)
                led_level = (implant.get_status(TransceiverStatus.MICS_CONF_LED_PROM) & 0x000F)
                print('\rIR LED: {}'.format(led_level), end='')


if __name__ == '__main__':
    cycle_mira_ir_forever()
