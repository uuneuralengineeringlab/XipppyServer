import can4python as can
import threading
import time
import json
import os


DEFAULT_KCD_FILE = '/usr/local/share/deka_server/deka_can.kcd'
# DEFAULT_KCD_FILE = r'C:\Users\Administrator\Code\COB\COB_Python\PyDEKA\deka\deka_can.kcd'
DEBUG_KCD_FILE = 'kcd/deka_can.kcd'


class DekaThread(threading.Thread):

    def __init__(self, bus):
        super().__init__()
        self.bus = bus


SENSOR_FRAME_IDS = [0x4AA, 0x4AB, 0x241, 0x341, 0x4C2]
CONTROL_FRAME_IDS = [0x210, 0x211, 0x212, 0x213]


HAND_MODE_MAP = {}


# DFC_MODE_MAP = {
#     'chan2_1': 'thumb_pitch_up',
#     'chan2_2': 'thumb_pitch_down',
#     'chan2_3': 'thumb_yaw_right',
#     'chan2_4': 'thumb_yaw_left',
#     'chan3_1': 'wrist_pronate',
#     'chan3_2': 'wrist_supinate',
#     'chan3_3': 'wrist_ext',
#     'chan3_4': 'wrist_flex',
#     'chan4_1': 'index_flex',
#     'chan4_2': 'index_extend',
#     'chan4_3': 'mrp_flex',
#     'chan4_4': 'mrp_ext',
# }


DFC_MODE_MAP = {
    'chan2_1': 'thumb_pitch_up',
    'chan2_2': 'thumb_pitch_down',
    'chan2_3': 'thumb_yaw_right',
    'chan2_4': 'thumb_yaw_left',
    'chan3_1': 'index_flex',
    'chan3_2': 'index_extend',
    'chan3_3': 'mrp_flex',
    'chan3_4': 'mrp_extend',
    'chan4_1': 'tmpa',
    'chan4_2': 'tmpb',
    'chan4_3': 'tmpc',
    'chan4_4': 'tmpd'
}


HAND_MODE_MAP = {
    'chan1_1': 'hand_open',
    'chan1_2': 'hand_close',
    'chan1_3': 'toggle_grip_next',
    'chan1_4': 'toggle_grip_prev',
    'chan2_1': 'wrist_pronate',
    'chan2_2': 'wrist_supinate',
    'chan2_3': 'wrist_ext',
    'chan2_4': 'wrist_flex',
    'chan3_1': 'mode_select'
}


class KCDUtils(object):
    """
    static class for getting data out of our KCD files

    This obviously doesn't really work at all
    """
    # @staticmethod
    # def get_dfc_signals(filename=DEFAULT_KCD_FILE):
    #     # config = can.FilehandlerKcd.read(filename)
    #     signalnames = []
    #     for frame_id in DFC_FRAME_IDS:
    #         f = bus.config.framedefinitions[frame_id]
    #         for s in f.signaldefinitions:
    #             signalsnames.append(signalsnames)
    #     return signalnames

    # @staticmethod
    # def get_sensor_signals(filename=DEFAULT_KCD_FILE):
    #     # config = can.FilehandlerKcd.read(filename)
    #     signalnames = []
    #     for frame_id in SENSOR_FRAME_IDS:
    #         f = bus.config.framedefinitions[frame_id]
    #         for s in f.signaldefinitions:
    #             signalsnames.append(signalsnames)
    #     return signalnames


class DekaSim(object):
    def __init__(self):
        self.thread = threading.Thread(target=self.run)
        if os.exists(DEFAULT_KCD_FILE):
            self.bus = can.CanBus.from_kcd_file(DEFAULT_KCD_FILE, 'can0',
                ego_node_ids=['1'], timeout=1.0)
        else:
            print("Warning: using fallback KCD file")
            self.bus = can.CanBus.from_kcd_file(DEBUG_KCD_FILE, 'can0',
                ego_node_ids=['1'], timeout=1.0)

    def run(self):
        while True:
            try:
                self.bus.send_signals(sync=0)
                r = self.bus.recv_next_signals()
            except can.CanTimeoutException:
                pass
            time.sleep(1)


class DFCData(object):
    """
    Class that automatically holds signal based data as defined by the
    DFC_MODE_MAP.
    """
    def __init__(self):
        self.signal_map = DFC_MODE_MAP
        for v in DFC_MODE_MAP.values():
            # build this out of the existing DFC_MODE_MAP
            self.__setattr__(v, None)

    def set_from_sensor(self, sensor_data):
        pass

    def get_signals(self, signalnames=[]):
        """
        Get dict of signals.

        list of attrs to send.
        """
        if len(signalnames) == 0:
            # default, return all the signals
            signalnames = list(self.signal_map.keys())

        signals = {}
        for n in signalnames:
            field_name = self.signal_map[n]
            value = self.__getattribute__(field_name)

            if value is None:
                raise KeyError('signal {} not set'.format(field_name))
            signals[n] = value
        return signals

    def get_signals_by_attr(self, fieldnames=[]):
        """
        Get dict of signals

        list of attrs to send.
        """
        # what?
        if len(fieldnames):
            signalnames = list(self.signal_map.keys())

        signals = {}
        for n in signalnames:
            field_name = self.signal_map[n]
            value = self.__getattribute__(field_name)
            if value is None:
                raise KeyError('signal {} not set'.format(field_name))
            signals[n] = value
        return signals

    def set_data(self, d):
        for signal, fieldname in self.signal_map.items():
            # print(signal)
            # update a sensor if we got one
            if signal in d:
                # print(fieldname)
                self.__setattr__(fieldname, d[signal])


class SensorData(object):
    """
    Class that automatically holds data from sensor signals in KCD
    file.  This will be used to pull data from the sensors.
    """
    def __init__(self):
        bus = can.CanBus.from_kcd_file(DEFAULT_KCD_FILE, 'can0',
                                       ego_node_ids=['1'])
        for frame_id in SENSOR_FRAME_IDS:
            f = bus.config.framedefinitions[frame_id]
            for s in f.signaldefinitions:
                self.__setattr__(s.signalname, None)

    def set_data(self, data):
        for s in self.__dict__.keys():
            # update a sensor if we got one
            if s in data:
                self.__setattr__(s, data[s])

    def __repr__(self):
        for s, v in self.__dict__.items():
            print("{}: {}".format(s, v), end='')
        print()


class HandData():
    def __init__(self):
        raise NotImplementedError


class DekaField(object):
    """
    Descriptor model for fields so that I can enforce limits
    and maybe do some other special tasks.
    """
    FIELD_MAX = 1.0

    def __init__(self, v=0):
        self._value = 0

    def __get__(self, instance, instance_type):
        return self._value

    def __set__(self, instance, value):
        if not (-self.FIELD_MAX <= value <= self.FIELD_MAX):
            msg = "values must be between {} and {}".format(
                -self.FIELD_MAX, self.FIELD_MAX)
            raise ValueError(msg)
        self._value = value


class DekaData2(object):
    """
    Reimplimentation of DekaData.  This should be easy to
    load from a JSON file and have a standardize interface.

    Then, we may set this to signal outputs as the fields change.
    """
    # this class will input maxes of 100, which can then be scaled
    # against however the Deka arm itself is setup.
    #
    # this can be +/- INPUT_MAX
    INPUT_MAX = 100
    SCALE = 10

    FIELD_MAPPING = {
        "thumb_pitch": ("thumb_pitch_down", "thumb_pitch_up"),
        "thumb_yaw": ("thumb_yaw_left", "thumb_yaw_right"),
        "index_finger": ("index_flex", "index_extend"),
        "mrp": ("mrp_flex", "mrp_extend"),
        "wrist_ext": ("", ""),
        "wrist_pron": ("", "")
    }

    def __init__(self, **kwargs):
        self.thumb_pitch = DekaField()
        self.thumb_yaw = DekaField()
        self.index_finger = DekaField()
        self.mrp = DekaField()
        self.wrist_ext = DekaField()
        self.wrist_pron = DekaField()

        self.thumb_pitch = 0.0
        self.thumb_yaw = 0.0
        self.index_finger = 0.0
        self.mrp = 0.0
        self.wrist_ext = 0.0
        self.wrist_pron = 0.0

        self._from_kwargs(**kwargs)

    def _from_kwargs(self, **kwargs):
        """
        Set data from key words.
        """
        keys = self.__dict__.keys()
        for k, v in kwargs.items():
            if k in keys:
                self.__dict__[k] = v

    def _from_json(self, buf):
        """
        TODO: switch this to a @classmethod, factory type function.
        """
        if isinstance(buf, bytes):
            buf = buf.decode('utf-8')
        json_dict = json.loads(buf)
        keys = self.__dict__.keys
        for k, v in json_dict.items():
            if k in keys():
                self.__dict__[k] = v

    def signals(self):
        """
        Get all the signals to send to the CAN server.
        """
        rt_dict = {}

        for k, v in self.__dict__.items():
            rt_dict[self.FIELD_MAPPING[k][0]] = 0
            rt_dict[self.FIELD_MAPPING[k][1]] = 0
            if v > 0:
                rt_dict[self.FIELD_MAPPING[k][1]] = v * self.SCALE
            else:
                rt_dict[self.FIELD_MAPPING[k][0]] = abs(v) * self.SCALE

        return rt_dict

    def to_json(self):
        """
        To be send on network
        """
        return json.dumps(self.__dict__)


class DekaControl(object):

    def __init__(self, frame_ids=CONTROL_FRAME_IDS):
        # collection of signals to send to arm.
        self.frame_ids = CONTROL_FRAME_IDS

        self.signals = {}
        # list of signals to send when server runs
        self.bus = can.CanBus.from_kcd_file(DEFAULT_KCD_FILE, 'can0',
                                            ego_node_ids=['1'])
        # print(self.bus.config.get_descriptive_ascii_art())
        self.allowed_signals = ()
        self._set_allowed_signals()
        assert len(self.allowed_signals) > 0

        self.thread = threading.Thread(target=self.run)

        self.dfc_data = DFCData()
        # last data returned from sensor's
        self.sensor_data = SensorData()
        # map physical names to signal names, using either the DFC or HAND
        # mappings found above.  Used so that we can ask for more physical
        # quantities rather than chan1_1's and chan1_2's
        self.signal_map = {}
        self.set_mode('dfc')

    def set_signal_map(self, signal_map):

        # Note: signal map is reversed from the input.
        self.signal_map = dict((v, k) for k, v in signal_map.items())

    def _set_allowed_signals(self):
        """
        more code than I want in the __init__.  Plus I want the allowed_signals
        to be a tuple..

        Ha, I have no idea what this does anymore.
        """
        signals = []
        for frame_id in CONTROL_FRAME_IDS:
            frame = self.bus.config.framedefinitions[frame_id]
            for sig in frame.signaldefinitions:
                signals.append(sig.signalname)
                self.signals[sig.signalname] = 0.0
        # allowed signals is deprecated.
        self.allowed_signals = tuple(signals)

    def start(self):
        self.thread.start()

    def set_active_mode(self):
        """
        switch from standby to active?

        This will block until done.
        """
        self.set_signals(mode=1000)
        time.sleep(0.5)
        self.set_signals(mode=0)
        # self.signals.pop('start_mode')
        # self.signals['switch_mode'] = 0
        # time.sleep(0.1)
        # self.signals.pop('switch_mode')
        # self.signals['start_mode'] = 0
        # time.sleep(0.01)
        # self.signals.pop('start_mode')

    def set_mode(self, mode):
        """
        switch from between dfc and hand.  This isn't really impliemented
        but the name of the signals going out will change.
        """
        if mode == 'dfc' or mode == 'hand':
            self.mode = mode
        else:
            raise ValueError('allowed mode: "hand" or "dfc"')
        # set signal map
        if self.mode == 'dfc':
            self.signal_map = dict((v, k) for k, v in DFC_MODE_MAP.items())
        else:
            self.signal_map = dict((v, k) for k, v in HAND_MODE_MAP.items())

    def set_signals_by_signame(self, **kwargs):
        """
        Set signals by signal name such as chan1_1, chan1_1's, not more
        physical names thumb_pitch_up, thumb_pitch_down.
        """
        # config = self.bus.config
        for k in kwargs:
            if k not in self.signals:
                raise KeyError('invalid signal: {}'.format(k))
            self.signals[k] = kwargs[k]

    def set_signals(self, **kwargs):
        """
        Normal way to set signals, using thumb_pitch_up, thumb_pitch_down, etc
        for DFC or hand_open, hand_close for hand control mode
        """
        for k in kwargs:
            if k not in self.signal_map:
                raise KeyError('invalid signal: {}'.format(k))
            self.signals[self.signal_map[k]] = kwargs[k]

    def reset_signals(self):
        for k in self.signals:
            self.signals[k] = 0

    def run(self):
        while True:
            # block loop, waiting for CAN frame from ARM
            r = self.bus.recv_next_signals()
            if 'sync' in r:
                # if we get a sync frame, get all the
                self.bus.send_signals(self.signals)


def example1():

    d = DekaControl()
    d.start()

    # d.set_signals_by_signame(chan2_2=500)
    while True:
        d.set_signals_by_signame(chan3_1=500)
        d.set_signals_by_signame(chan3_2=0)
        d.set_signals_by_signame(chan3_3=500)
        d.set_signals_by_signame(chan3_4=0)
        time.sleep(2)
        d.set_signals_by_signame(chan3_1=0)
        d.set_signals_by_signame(chan3_2=500)
        d.set_signals_by_signame(chan3_3=0)
        d.set_signals_by_signame(chan3_4=500)
        time.sleep(2)


def example2():

    d = DekaControl()
    d.start()

    while True:
        d.set_signals(index_extend=500)
        d.set_signals(index_flex=0)
        # d.set_signals_by_signame(chan3_3=500)
        # d.set_signals_by_signame(chan3_4=0)
        time.sleep(2)
        d.set_signals(index_flex=500)
        d.set_signals(index_extend=0)
        # d.set_signals_by_signame(chan3_3=0)
        # d.set_signals_by_signame(chan3_4=500)
        time.sleep(2)


def example3():
    d = DekaControl()
    d.start()

    toggle = 0

    while True:
        if toggle == 0:
            toggle = 1
        else:
            toggle = 0

        if toggle:
            dd = DekaData2(index_finger=50, thumb_pitch=50)
        else:
            dd = DekaData2(index_finger=-50, thumb_pitch=-50)
        d.set_signals(**dd.signals())

        time.sleep(2)


if __name__ == '__main__':
    example3()
