import threading
import time
import json
import numpy as np

DEFAULT_KCD_FILE = '/usr/local/share/deka_server/deka_can.kcd'
# DEFAULT_KCD_FILE = r'C:\Users\Administrator\Code\COB\COB_Python\PyDEKA\deka\deka_can.kcd'

DEBUG_KCD_FILE = 'kcd/deka_can.kcd'


class DekaThread(threading.Thread):

    def __init__(self, bus):
        super().__init__()
        self.bus = bus


SENSOR_FRAME_IDS = [0x4AA, 0x4AB, 0x241, 0x341, 0x4C2]
CONTROL_FRAME_IDS = [0x210, 0x211, 0x212, 0x213]


# The Utah
UTAH_MODE_MAP = {
    'chan1_1': 'thumb_pitch',
    'chan1_2': '',
    'chan1_3': '',
    'chan1_4': 'thumb_yaw',
    'chan2_1': 'index_finger',
    'chan2_2': '',
    'chan2_3': 'mrp',
    'chan2_4': '',
    'chan3_1': 'wrist_pron',
    'chan3_2': 'wrist_sup',
    'chan3_3': 'wrist_flex',
    'chan3_4': 'wrist_ext',
    'chan4_1': 'mode',
    'chan4_2': '',
    'chan4_3': '',
    'chan4_4': ''
}


HAND_MODE_MAP = {}


class HandData():
    def __init__(self):
        raise NotImplementedError


class DekaField(object):
    """
    Descriptor model for fields so that I can enforce limits
    and maybe do some other special tasks.
    """
    # FIELD_LIM = [-1.0, 1.0]

    def __init__(self, v=0):
        self._value = 0

    def __get__(self, instance, instance_type):
        return self._value

    def __set__(self, instance, value):
        if not (self.FIELD_LIM[0] <= value <= self.FIELD_LIM[1]):
            msg = "values must be between {} and {}".format(
                self.FIELD_LIM[0], self.FIELD_LIM[1])
            raise ValueError(msg)
        self._value = value


class DekaDataUtah(object):
    """
    Reimplimentation of DekaData.  This should be easy to
    load from a JSON file and have a standardize interface.

    Then, we may set this to signal outputs as the fields change.
    """
    # this class will input maxes of 100, which can then be scaled
    # against however the Deka arm itself is setup.
    #
    # this can be +/- INPUT_MAX
    # INPUT_MAX = 100
    SCALE = 1024

    FIELD_MAPPING = {
        "thumb_pitch": ("thumb_pitch", ),
        "thumb_yaw": ("thumb_yaw", ),
        "index_finger": ("index_finger", ),
        "mrp": ("mrp", ),
        "wrist_pron": ('wrist_sup', 'wrist_pron'),
        "wrist_flex": ('wrist_flex', 'wrist_ext'),
        # "mode": ("mode",),
    }

    def __init__(self, **kwargs):
        # self.thumb_pitch = DekaField()
        # self.thumb_yaw = DekaField()
        # self.index_finger = DekaField()
        # self.mrp = DekaField()
        # self.wrist_pron = DekaField()
        # self.wrist_flex = DekaField()

        self.thumb_pitch = 0
        self.thumb_yaw = 0
        self.index_finger = 0
        self.mrp = 0
        self.wrist_pron = 0
        self.wrist_flex = 0

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

    @staticmethod
    def tf_one_chan(v):
        """
        Linearly shift and scale data from range [-1, 1] to [0, 1024]

        v -> (v+ 1.) / 2 * 1024
        """
        tmp = v + 1.0
        tmp = tmp * 512
        return int(tmp)

    @staticmethod
    def tf_two_chan(v):
        """
        Linearly shift and scale data from range [-1, 1] to two channels

        v ->
           if positive:
               ((v+ 1.) / 2 * 1024, 0)

           if negative:
               (0, (v+ 1.) / 2 * 1024)
        """
        tf_val = int(np.abs(v) * 1024)
        if v > 0:
            return (tf_val, 0)
        else:
            return (0, tf_val)

    def signals(self):
        rt_dict = {}

        for k, v in self.__dict__.items():
            field_mapping = self.FIELD_MAPPING[k]
            if len(field_mapping) == 1:
                # only one mapping so we use the positive and negative values
                # in each signal.
                rt_dict[field_mapping[0]] = self.tf_one_chan(v)
            else:
                # if there are two mappings, there is a separate channel
                # for positive and negative.
                tup = self.tf_two_chan(v)

                rt_dict[field_mapping[0]] = tup[0]
                rt_dict[field_mapping[1]] = tup[1]

        return rt_dict

    def to_json(self):
        return json.dumps(self.__dict__)


def example():

    import pdb; pdb.set_trace()
    
    from deka import haptix_can
    # import haptix_can

    d = haptix_can.DekaControl()
    d.set_signal_map(UTAH_MODE_MAP)

    d.start()

    toggle = 0

    time.sleep(10)
    print('sending active')
    d.set_active_mode()

    while True:
        if toggle == 0:
            toggle = 1
        else:
            toggle = 0

        max_value = 1.0
        if toggle:
            dd = DekaDataUtah(index_finger=max_value, thumb_pitch=0,
                              thumb_yaw=max_value)
        else:
            dd = DekaDataUtah(index_finger=0, thumb_pitch=max_value,
                              thumb_yaw=0)

        d.set_signals(**dd.signals())

        time.sleep(2)


if __name__ == '__main__':
    example()
