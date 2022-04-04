import logging
import time

from xipppy import (
    transceiver_get_power_state, POWER_STATE_R3,
    transceiver_set_implant_servo_dac,
    DEFAULT_SERVO_DAC_LEVEL,
    transceiver_power_servo_enable,
    transceiver_get_implant_voltage,
    transceiver_enable
)

logger = logging.getLogger(__name__)


def ensure_implant_booted():
    """
    This example shows how to ensure that a MIRA implant is booted. This
    function will return False if the implant could not be booted in the
    retry limit. True otherwise. An exception is not thrown if the implant is
    not booted. A xipppy_open context must be entered before calling this
    function.
    """
    for i in range(10):
        try:
            power_st = transceiver_get_power_state()
            if power_st == POWER_STATE_R3['servo_enable']:
                logger.debug("Implant is booted.")
                transceiver_set_implant_servo_dac(DEFAULT_SERVO_DAC_LEVEL)
                transceiver_power_servo_enable(True)
                for i in range(20):
                    imp_v = transceiver_get_implant_voltage()
                    if imp_v <= 3.5:
                        return True
                    else:
                        logger.debug(
                            'Waiting for implant voltage to settle (imp_v={})'.format(
                                imp_v)
                        )
                    time.sleep(0.33)
            elif power_st == POWER_STATE_R3['idle']:
                logger.debug('Implant is down, booting.')
                transceiver_enable(0, True)
            else:
                logger.debug('Implant is booting.')
        except IndexError:
            pass
        time.sleep(0.5)
    return False
