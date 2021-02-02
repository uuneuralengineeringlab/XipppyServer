from deka import haptix_can
from deka.haptix_utah import DekaDataUtah, UTAH_MODE_MAP
import asyncio
import json
import time
import signal
import re

DEFAULT_PORT = 6263
use_mock = False


def group_json(buf):
    """
    create a list of all the json packets and perhaps allow for a little
    checking...
    """
    return re.findall('\{.*?\}', buf)


class DekaServer(object):
    """
    Listens for a limb data to arrive on a socket and send it to the arm.

    This holds the `DekaControl` server and updates its data which
    in turn get's updated to the limb, when sync commands arrive.
    """
    def __init__(self, addr='localhost', port=6263):
        if use_mock:
            self.dekactl = None
        else:
            self.dekactl = haptix_can.DekaControl()
            self.dekactl.set_signal_map(UTAH_MODE_MAP)
            # self.dekactl.set_active_mode()

            self.dekactl.start()
        self.port = port
        self.addr = addr

    @asyncio.coroutine
    def recv(self, reader, writer):
        """
        Main listener.

        Unpack JSON data.  Convert it with a data class to what the
        limb is expecting and ship it off to `DekaControl`
        """
        while True:
            data = yield from reader.read(1024)
            if not data:
                print("connection broken")
                break
            if isinstance(data, bytes):
                data = data.decode('utf-8')
            packets = group_json(data)
            if len(packets) > 0:
                packet = packets[-1]
            else:
                continue

            parsed = json.loads(packet)
            if 'mode' in parsed:
                self.dekactl.set_active_mode()

            d = DekaDataUtah(**parsed)

            if self.dekactl:
                self.dekactl.set_signals(**d.signals())

    def start(self):
        """
        Control of event loop.
        """
        loop = asyncio.get_event_loop()

        server_coro = asyncio.start_server(self.recv, self.addr,
                                           self.port, loop=loop)
        server = loop.run_until_complete(server_coro)
        # Serve requests until Ctrl+C is pressed

        print('Serving on {}'.format(server.sockets[0].getsockname()))
        try:
            loop.run_forever()
        except KeyboardInterrupt:
            pass

    def stop():
        """
        Stop event loop.  Not entirely sure this is doing what I think it's
        doing but it does work.
        """
        raise KeyboardInterrupt


class DekaClient(object):
    """
    Toy application that listens to for data and sends it along the network.
    """
    def __init__(self, addr='localhost', port=6263):
        # self.pos = haptix_can.DekaData2()
        self.port = port
        self.addr = addr

    @asyncio.coroutine
    def __call__(self, **kwargs):
        """
        Just style I guess. but kind of silly
        """
        yield from self.send(**kwargs)

    @asyncio.coroutine
    def send(self, **kwargs):
        """
        Open a connection to the server in this module and ship the
        requested data.
        """
        reader, writer = \
            yield from asyncio.open_connection('localhost', self.port)
        if 'mode' in kwargs.keys():
            # Special case, if we want to send a mode select.  This is needed
            # to start the arm after a reboot
            buf = """{"mode":0}"""
        else:
            # Normal case.  Use DekaDataUtah to validate the data.  This
            # will screen out bad data without really complianing so be
            # careful.
            d = DekaDataUtah(**kwargs)
            buf = d.to_json()
        writer.write(buf.encode('utf-8'))
        yield from writer.drain()
        writer.close()


def server(addr='192.168.42.1', port=6263):
    """
    entry for server
    """
    ds = DekaServer(addr, port)
    ds.start()


def client():
    """
    entry for client
    """
    c = DekaClient()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(c(mode=1))

    toggle = False
    running = True

    def handler(_1, _2):
        # why is so hard to remember nonlocal
        nonlocal running
        print("got it. Exiting")
        running = False

    signal.signal(signal.SIGINT, handler)

    while running:
        if toggle:
            loop.run_until_complete(c(index_finger=-1, mrp=-1,
                                      thumb_pitch=-1, thumb_yaw=-1,
                                      wrist_pron=-1, wrist_flex=-1))
            toggle = False
        else:
            loop.run_until_complete(c(index_finger=1, mrp=1.0,
                                      thumb_pitch=1, thumb_yaw=1,
                                      wrist_pron=1, wrist_flex=1))
            toggle = True

        time.sleep(2.0)


if __name__ == '__main__':
    # may as well let this get run from python -m ...
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument('-c', '--client', default=False, action='store_true')

    args = parser.parse_args()

    if args.client:
        client()
    else:
        server()
