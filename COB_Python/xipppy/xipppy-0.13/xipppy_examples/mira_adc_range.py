"""
This example will cycle through all of the MIRA implant recording ADC
levels in an infinite loop. A change occurs ever 3 seconds. This example
does not start nipexec; but, it will open xipppy and prepare the implant
to receive commands. Upon exiting with ctrl+c it will restore the
transceiver to normal operation (exiting command mode) and close xipppy.
"""
import time
from itertools import count

from xipppy.transceiver import ensure_implant_booted, TransceiverStatus
from xipppy import xipppy_open
from xipppy import MiraImplantCmdMode


def cycle_mira_adc_range_forever():
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
                # This puts the implant recording ADCs into the lowest range
                # which has the highest resolution and lowest dynamic range.
                implant.set_implant_adc_res_range(0)
                # This puts the implant recording ADCs into the highest range
                # which has the coarsest resolution and the highest dynamic
                # range.
                implant.set_implant_adc_res_range(implant.max_adc_res)

        # Cycle through all ADC resolutions forever, once every second.
        with MiraImplantCmdMode() as implant:
            print('Ctrl+C to quit.')
            for i in count():
                implant.set_implant_adc_res_range(
                    divmod(i + 1, implant.max_adc_res + 1)[1]
                )
                time.sleep(1)
                adc_res = (implant.get_status(TransceiverStatus.MICS_CONF_LED_PROM)&0x00F0) >> 4
                print("\radc resolution= {}".format(adc_res), end='')


if __name__ == '__main__':
    cycle_mira_adc_range_forever()
