#!/usr/bin/env python3
"""
Starts the CAN interface and a server to listen for Deka udp commands.

"""

import subprocess
import time
import feedbackdecode as fd


def power_iso_port():
    """
    Bit of a hack that should only be needed by the black nomads.

    nipexec will disable and power down the isolated port so we need to
    enable it in that case.
    """
    # This is a pretty lame hack but it will work for now.  I would
    # rather soft link that single package into this virtualenv.
    import sys
    sys.path.append('/usr/local/lib/python3.4/dist-packages')
    from nomad_utils import fpga
    from nomad_utils.fpga import constants as constants
    pwr = fpga.GetRegister(constants.NIP_REG_PWR) & constants.NIP_PWR_ON_MASK

    fpga.SetRegister(constants.NIP_REG_PWR,  pwr | constants.NIP_PWR_ON_SET |
                      constants.NIP_PWR_ON_ISO_5V)
    fpga.SetRegister(constants.NIP_REG_AUX1, constants.NIP_AUX1_ISO_SET &
                      ~constants.NIP_AUX1_ISO_ENABLE)


def startup(delay=10):
    """
    load the CAN module and start the CAN interface.

    Args:
        delay - This needs to wait until nipc is loaded so as a hack way to
           do this I just wait for ten seconds, by default.
    """
    time.sleep(delay)
    #try:
    #    power_iso_port()
    #except AttributeError:
    #    print('Warning: Unable to modify FPGA registers.')

    try:
        subprocess.check_call(['modprobe', 'nomad-spi-xilinx'])
    except:
        print("warning: modprobe of nomad-spi-xilinx failed")
    time.sleep(1.0)
    # how long do we wait for the can0 to comeup?  Perhaps we
    # just probe?  5 seems like plenty,  probably too much.
    now = time.time()
    timeout = now + 5
    while now < timeout:
        try:
            subprocess.check_call(['ip', 'link', 'set', 'can0', 'type', 'can',
                                   'bitrate', '1000000'])
            time.sleep(1.0)
            subprocess.check_call(['ip', 'link', 'set', 'can0', 'up'])
        except subprocess.CalledProcessError as e:
            print("Warning: failed to set CAN settings")
        try:
            subprocess.check_call(['ifconfig', 'can0'], stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL)
            break
        except:
            print('Warning: could not find CAN interface')
        time.sleep(0.5)
        now = time.time()
        
    #now = time.time()
    #timeout = now + 5
    #while now < timeout:
    #    try:
    #        subprocess.check_call(['ifup', 'wlan0'])
    #        time.sleep(1.0)
    #    except subprocess.CalledProcessError as e:
    #        print("Warning: failed to start wlan0")
    #    try:
    #        subprocess.check_call(['ifconfig', 'wlan0'], stdout=subprocess.DEVNULL,
    #                              stderr=subprocess.DEVNULL)
    #        break
    #    except:
    #        print('Warning: could not find wlan0')
    #    time.sleep(0.5)
    #    now = time.time()


if __name__ == '__main__':
   
    startup()
    deka = fd.DekaControl()
      
