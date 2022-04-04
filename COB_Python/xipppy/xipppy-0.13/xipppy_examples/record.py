#!/usr/bin/env python3
"""
Start, stop, or pause FileSave on the processor.
"""

import sys
import time
import xipppy as xp

def get_electrodes(type = 'micro'):
    """
    Get a list of electrodes of the given type, verbosely.
    """
    return xp.list_elec('micro')


def show_subscriptions(elec_list):
    """
    Print the number of subscriptions to each stream type for each electrode in
    the argument.
    """
    counts = dict()
    for elec in elec_list:
        for s in xp.get_fe_streams(elec):
            if not s in counts:
                if xp.signal_save(elec, s):
                    counts.update({s: 1})
            else:
                if xp.signal_save(elec, s):
                    counts[s] = counts[s] + 1;
    print('subscription counts ', counts)


def subscribe_all_streams(elec_list):
    """
    For all the streams of all the electrodes, set them to be saved.  Note that
    they do not have to be selected via xipppy.signal_etc functions.
    """
    for elec in elec_list:
        for s in xp.get_fe_streams(elec):
            xp.signal_save_set(elec, s, True)


def unsubscribe_all_streams(elec_list):
    for elec in elec_list:
        for s in xp.get_fe_streams(elec):
            xp.signal_save_set(elec, s, False)


def subscribe_some(elec_list):
    """
    Subscribe to only some streams
    """
    for elec in elec_list:
        xp.signal_save_set(elec, "raw", True)
        xp.signal_save_set(elec, "hi-res", True)
        #xp.signal_save_set(elec, "stim", True)
        xp.signal_save_set(elec, "spk", True)
        xp.signal_save_set(elec, "lfp", True)

def send_trial(command):
    """
    Send the given command to the processor to start/stop recording.
    Note operator ID is 1, which is less than 128, indicating that this command
    is being sent to a processor and not an operator.
    """
    print(xp.trial(1, command, 'xipppy'))


if __name__ == '__main__':
    with xp.xipppy_open(True):

        # Without argument, print the stream subscriptions.  Otherwise "start"
        # or "stop" as command-line arguments will start or stop the Nomad
        # recording, respectively.  "clear" will clear the signal selection.
        try:
            if sys.argv[1] == 'start':
                elec_list = get_electrodes()
                if not elec_list:
                    sys.exit('No electrodes')

                # Uncomment one of the following to deselect everything for
                # saving, enable everything for saving, or enable some for
                # saving.  Or whatever combinations you need.
                #
                # unsubscribe_all_streams(elec_list)
                # subscribe_all_streams(elec_list)
                subscribe_some(elec_list)
                send_trial('recording')

            elif sys.argv[1] == 'stop':
                send_trial('stopped')

            elif sys.argv[1] == 'pause' or sys.argv[1] == 'resume':
                send_trial('paused');

            elif sys.argv[1] == 'clear':
                elec_list = get_electrodes()
                if not elec_list:
                    sys.exit('No electrodes')
                unsubscribe_all_streams(elec_list)

        except IndexError:
            elec_list = get_electrodes()
            show_subscriptions(elec_list)
