"""canio.RawCanBus, the daemon's raw AF_CAN backend (plan C, step 1) -- no hardware needed.

A datagram socketpair stands in for the kernel's CAN socket: it keeps frame boundaries, and
non-blocking reads end in BlockingIOError the same way. The python-can comparisons use python-can's
own socketcan packer/dissector (build_can_frame / dissect_can_frame -- the bytes SocketcanBus.send
writes and the parsing its recv does), and are skipped where python-can is not installed.
"""
import errno
import os
import socket
import struct
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import canio                                                            # noqa: E402

try:
    import can
    from can.interfaces.socketcan.socketcan import build_can_frame, dissect_can_frame
except ImportError:                                                     # the Mac without python-can
    can = None

needs_python_can = pytest.mark.skipif(can is None, reason="python-can not installed")

STATUS_ID = 0x2900 | 105                 # a servo status frame from drive 105
STATUS = struct.pack(">hhhbB", 1234, -56, 789, 41, 0)


def _pair(**kw):
    a, b = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
    return canio.RawCanBus("can0", sock=a, **kw), b


def _frame(can_id, data, rtr=False):
    """struct can_frame bytes, built independently of canio."""
    return struct.pack("=IB3x8s", can_id, len(data), bytes(data))


@pytest.fixture(autouse=True)
def _clean_send_state():
    canio._send_errors.clear()
    canio._send_state.clear()
    canio.install_send_error_hook(None)
    yield
    canio._send_errors.clear()
    canio._send_state.clear()
    canio.install_send_error_hook(None)


# ---- receive ----------------------------------------------------------------------------------
@pytest.mark.parametrize("mode", ["poll", "exception", "select"])
def test_recv_decodes_a_status_frame(mode):
    bus, k = _pair(empty_check=mode)
    k.send(_frame(canio.CAN_EFF_FLAG | STATUS_ID, STATUS))
    m = bus.recv(timeout=0.0)
    assert m.is_extended_id and m.arbitration_id == STATUS_ID and m.data == STATUS
    st = canio.parse_status(m.data)
    assert st["pos"] == pytest.approx(123.4) and st["spd"] == -560.0
    assert st["cur"] == pytest.approx(7.89) and (st["temp"], st["err"]) == (41, 0)
    assert bus.recv(timeout=0.0) is None           # empty: None, not an exception
    bus.shutdown()
    bus.shutdown()                                 # idempotent, like the daemon's close path


@needs_python_can
def test_decode_matches_python_can():
    """The same frame bytes through python-can's dissector and through RawCanBus give the same
    id/flags/data, hence the same parse_status."""
    msgs = [can.Message(arbitration_id=STATUS_ID, data=STATUS, is_extended_id=True),
            can.Message(arbitration_id=0x2900 | 104, data=bytes([0xFF] * 8), is_extended_id=True),
            can.Message(arbitration_id=0x123, data=b"\x01\x02\x03", is_extended_id=False)]
    bus, k = _pair()
    for msg in msgs:
        raw = build_can_frame(msg)
        k.send(raw)
        got = bus.recv(timeout=0.0)
        can_id, dlc, _flags, data = dissect_can_frame(raw)
        assert got.is_extended_id == bool(can_id & canio.CAN_EFF_FLAG) == msg.is_extended_id
        assert got.arbitration_id == msg.arbitration_id
        assert bytes(got.data) == bytes(data) == bytes(msg.data)
        assert canio.parse_status(got.data) == canio.parse_status(msg.data)


@pytest.mark.parametrize("mode", ["poll", "exception", "select"])
def test_recv_skips_error_and_remote_frames_and_keeps_standard_ids(mode):
    bus, k = _pair(empty_check=mode)
    k.send(_frame(canio.CAN_ERR_FLAG | 0x4, bytes(8)))
    k.send(_frame(canio.CAN_EFF_FLAG | canio.CAN_RTR_FLAG | STATUS_ID, bytes(8)))
    k.send(_frame(0x7FF, b"\xAA"))
    m = bus.recv(timeout=0.0)
    assert (m.arbitration_id, m.is_extended_id, m.data) == (0x7FF, False, b"\xAA")
    assert bus.recv(timeout=0.0) is None


@pytest.mark.parametrize("mode", ["poll", "exception", "select"])
def test_recv_timeout_waits_then_gives_up(mode):
    bus, k = _pair(empty_check=mode)
    t = time.monotonic()
    assert bus.recv(timeout=0.05) is None
    assert 0.04 <= time.monotonic() - t < 0.5
    k.send(_frame(canio.CAN_EFF_FLAG | STATUS_ID, STATUS))
    assert bus.recv(timeout=0.05).arbitration_id == STATUS_ID


# ---- send -------------------------------------------------------------------------------------
def _sent_by_raw(fn, *args):
    bus, k = _pair()
    assert fn(bus, *args) is True
    return k.recv(64)


@needs_python_can
@pytest.mark.parametrize("fn,args", [
    (canio.set_current, (105, 0.0)),
    (canio.set_current, (104, -3.217)),
    (canio.set_pos, (106, -123.4567)),
    (canio.set_pos, (105, 1e9)),                   # clamped to int32
    (canio.force_control, (104, bytes(range(1, 9)))),
])
def test_encode_matches_python_can(fn, args):
    """The bytes RawCanBus writes == the bytes python-can's SocketcanBus.send writes for the
    can.Message canio builds for its python-can backend."""
    raw = _sent_by_raw(fn, *args)

    class Capture:                                  # the python-can path: bus.send(can.Message)
        channel = "can0"
        msg = None

        def send(self, msg):
            Capture.msg = msg
    assert fn(Capture(), *args) is True
    assert isinstance(Capture.msg, can.Message)
    assert raw == build_can_frame(Capture.msg)


def test_encode_layout_without_python_can():
    """Independent of python-can: SET_POS to drive 106 = extended id 0x406, 4 bytes, BE int32."""
    raw = _sent_by_raw(canio.set_pos, 106, 12.5)
    can_id, n, _pad, _res0, len8 = struct.unpack("=IBBBB", raw[:8])
    data = raw[8:]
    assert can_id == canio.CAN_EFF_FLAG | (canio.CAN_PACKET_SET_POS << 8) | 106
    assert n == 4 and len8 == 4
    assert data == struct.pack(">i", 125000) + bytes(4)


def test_force_control_still_refuses_a_short_payload():
    bus, _ = _pair()
    with pytest.raises(ValueError):
        canio.force_control(bus, 104, b"\x00" * 7)


# ---- a full TX queue is visible ---------------------------------------------------------------
class _FullSock:
    """A socket whose TX queue is full: send() fails like socketcan with txqueuelen exhausted.
    No real fd, so the buses built on it use empty_check="exception" (poll needs an fd)."""
    def __init__(self, err=errno.ENOBUFS):
        self.err = err
        self.sends = 0

    def setblocking(self, flag):
        pass

    def recv(self, n):
        raise BlockingIOError

    def send(self, b):
        self.sends += 1
        raise OSError(self.err, os.strerror(self.err))

    def fileno(self):
        return -1

    def close(self):
        pass


@pytest.mark.parametrize("err", [errno.ENOBUFS, errno.EAGAIN, errno.ENETDOWN])
def test_a_refused_frame_reaches_send_errors_and_the_hook(err):
    seen = []
    canio.install_send_error_hook(lambda ch, e: seen.append((ch, e)))
    bus = canio.RawCanBus("can1", sock=_FullSock(err), empty_check="exception")
    assert canio.set_current(bus, 105, 0.0) is False
    assert canio.send_errors() == {"can1": 1}
    assert seen and seen[0][0] == "can1" and isinstance(seen[0][1], OSError)
    assert seen[0][1].errno == err
    st = canio.send_stats()["can1"]
    assert st["errors"] == 1 and st["backoff"] is False


def test_a_full_queue_backs_off_exactly_as_before():
    sock = _FullSock()
    bus = canio.RawCanBus("can0", sock=sock, empty_check="exception")
    for _ in range(canio.CHANNEL_BACKOFF_AFTER + 20):
        canio.set_current(bus, 105, 0.0)
    st = canio.send_stats()["can0"]
    assert st["backoff"] is True
    assert st["errors"] == canio.CHANNEL_BACKOFF_AFTER + 1     # the first retry after the backoff
    assert st["skipped"] == 19
    assert sock.sends == canio.CHANNEL_BACKOFF_AFTER + 1


def test_short_write_is_an_error():
    class Short(_FullSock):
        def send(self, b):
            return 8
    bus = canio.RawCanBus("can0", sock=Short(), empty_check="exception")
    assert canio.set_pos(bus, 104, 0.0) is False
    assert canio.send_errors() == {"can0": 1}


# ---- the switch -------------------------------------------------------------------------------
def test_the_daemon_default_is_raw_and_callers_default_to_python_can():
    import inspect
    assert canio.CANIO_BACKEND == "raw"
    assert inspect.signature(canio.open_buses).parameters["backend"].default == "python-can"
    with pytest.raises(ValueError):
        canio.open_buses("socketcan", ["can0"], backend="bogus")


def test_mock_is_mock_whatever_the_backend():
    for backend in canio.BACKENDS:
        buses = canio.open_buses("socketcan", ["can0", "can1"], mock=True, backend=backend)
        assert all(isinstance(b, canio.MockBus) for b in buses.values())


def test_raw_without_af_can_falls_back_to_mock(monkeypatch):
    monkeypatch.setattr(canio, "RAW_CAN_AVAILABLE", False)
    buses = canio.open_buses("socketcan", ["can0"], backend="raw")
    assert isinstance(buses["can0"], canio.MockBus)


def test_raw_opens_raw_buses(monkeypatch):
    made = []

    class Fake:
        def __init__(self, ch):
            made.append(ch)
    monkeypatch.setattr(canio, "RAW_CAN_AVAILABLE", True)
    monkeypatch.setattr(canio, "RawCanBus", Fake)
    buses = canio.open_buses("socketcan", ["can0", "can1"], backend="raw")
    assert made == ["can0", "can1"] and all(isinstance(b, Fake) for b in buses.values())


@needs_python_can
def test_python_can_backend_still_works():
    """CANIO_BACKEND = 'python-can': can.Bus end to end, on python-can's virtual interface (raw
    asks for socketcan, so 'virtual' also exercises the fall-through to python-can)."""
    tx = canio.open_buses("virtual", ["vcan_plan_c"], backend="python-can")["vcan_plan_c"]
    rx = canio.open_buses("virtual", ["vcan_plan_c"], backend="raw")["vcan_plan_c"]
    class Named:                      # VirtualBus keeps its queue list in .channel; socketcan a str
        channel = "vcan_plan_c"
        send = tx.send
    try:
        assert isinstance(tx, can.BusABC) and isinstance(rx, can.BusABC)
        assert canio.set_pos(Named(), 106, -12.5) is True
        m = rx.recv(timeout=1.0)
        assert m.is_extended_id and m.arbitration_id == (canio.CAN_PACKET_SET_POS << 8) | 106
        assert struct.unpack(">i", bytes(m.data))[0] == -125000
    finally:
        tx.shutdown()
        rx.shutdown()


def test_daemon_carries_the_backend():
    import daemon as D
    assert D.RobotDaemon(mock=True).can_backend == "raw"
    assert D.RobotDaemon(mock=True, can_backend="python-can").can_backend == "python-can"


def test_poll_is_the_default_empty_check():
    """Measured on the Pi 3B (2026-11-20): the BlockingIOError that ends an 'exception' drain costs
    55-58 us, a poll(0) 5 us."""
    bus, _ = _pair()
    assert bus.empty_check == "poll"
    with pytest.raises(ValueError):
        _pair(empty_check="bogus")
