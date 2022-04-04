"""
This script shows how to get implant and transceiver info as well as
how to update MIRA implant model and serial number. This example
does not start nipexec; but, it will open xipppy and prepare the implant
to receive commands.
"""

import xipppy
import xipppy.transceiver as transceiver


def implant_info(implant):
    implant_fw_version = implant.get_fw_version() & 0x00FF
    implant_hw_version = implant.get_fw_version() >> 8 & 0x00FF
    print("Implant fw version: ", implant_fw_version)
    print("Implant hw version: ", implant_hw_version)
    print("Implant model number: ", implant.get_r_number())
    print("Implant serial number: ", implant.get_serial_number())


def transceiver_info():
    print("Transceiver model number:",
          transceiver.transceiver_status(0)
          [transceiver.TransceiverStatus.RNUM])
    print("Transceiver Serial number:",
          transceiver.transceiver_status(0)
          [transceiver.TransceiverStatus.SERIAL_NUM])
    print("Transceiver ir-level: ", transceiver.transceiver_get_ir_led_level(0))


if __name__ == '__main__':
    with xipppy.xipppy_open():
        transceiver_info()
        transceiver.ensure_implant_booted()
        with xipppy.MiraImplantCmdMode() as implant:
            implant_info(implant)
            print("Update firmware config....")
            implant.update_fw_config(1445, 1884)
            implant_info(implant)
