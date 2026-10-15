"""
Reads channel data from the FlySky IA10B receiver over i-BUS (a single-wire
serial protocol carrying all channels, read via the Pi's UART).

PROTOCOL CONFIRMATION NEEDED: the official FS-i6X manual documents
"i-BUS Setup" as a named feature of this exact receiver family (section
8.9) — this module assumes i-BUS on that basis. However, the seller
separately described the output as "SBUS" in a chat message, which
conflicts: SBUS is a different protocol (inverted signal levels, 100000
baud 8E2, Futaba-originated) that needs a small inverter circuit between
the receiver and the Pi's UART RX pin — i-BUS does NOT need one, it's a
plain non-inverted UART at 115200 baud. Wiring real SBUS straight into a
UART expecting i-BUS will not decode (and vice versa). CHECK THE PHYSICAL
RECEIVER'S PORT LABEL (FlySky receivers typically print "i-BUS" directly
next to the correct pin) before connecting it to the Pi.

i-BUS protocol: continuous 32-byte packets:
    byte 0:      0x20  (packet length)
    byte 1:      0x40  (command: channel data)
    bytes 2-29:  14 channels, little-endian uint16 each (~1000-2000 range,
                 same scale as RC PWM pulse widths in microseconds)
    bytes 30-31: checksum, little-endian uint16 = 0xFFFF - sum(bytes 0-29)

Requires pyserial:
    pip3 install pyserial
And the Pi's primary UART freed from the Linux serial console:
    sudo raspi-config  ->  Interface Options -> Serial Port
        "login shell over serial?" -> No
        "serial port hardware enabled?" -> Yes
"""

import threading
import time

IBUS_PACKET_LEN = 32
IBUS_NUM_CHANNELS = 14
IBUS_HEADER = bytes([0x20, 0x40])


def _ibus_checksum_ok(packet):
    checksum = sum(packet[:30])
    expected = 0xFFFF - checksum
    actual = packet[30] | (packet[31] << 8)
    return expected == actual


class IBusReceiver:
    """Continuously reads i-BUS packets from a serial port on its own
    thread, exposing the latest decoded channel values. Channel values
    are raw i-BUS units (~1000-2000, same scale as RC PWM microseconds) —
    mapping those to specific functions (mode, throttle, etc.) is
    remote_control.py's job, not this module's."""

    def __init__(self, serial_port, baudrate=115200):
        import serial  # pyserial — imported here so this module still loads without it installed

        self.ser = serial.Serial(serial_port, baudrate=baudrate, timeout=0.1)
        self.channels = [1500] * IBUS_NUM_CHANNELS  # neutral default until the first packet arrives
        self.last_update_at = None
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _loop(self):
        buf = bytearray()
        while self.running:
            chunk = self.ser.read(64)
            if not chunk:
                continue
            buf.extend(chunk)

            while len(buf) >= IBUS_PACKET_LEN:
                if bytes(buf[0:2]) != IBUS_HEADER:
                    buf.pop(0)  # resync: drop one byte and look again
                    continue

                packet = buf[:IBUS_PACKET_LEN]
                if _ibus_checksum_ok(packet):
                    channels = [
                        packet[2 + i * 2] | (packet[3 + i * 2] << 8)
                        for i in range(IBUS_NUM_CHANNELS)
                    ]
                    with self.lock:
                        self.channels = channels
                        self.last_update_at = time.time()
                    del buf[:IBUS_PACKET_LEN]
                else:
                    buf.pop(0)  # bad checksum — resync byte by byte rather than drop a whole packet blind

    def get_channels(self):
        """Returns the 14 latest channel values (raw ~1000-2000 units), or
        None if no valid packet has ever been received."""
        with self.lock:
            return list(self.channels) if self.last_update_at is not None else None

    def is_stale(self, max_age_sec):
        """True if no valid packet has arrived recently — caller should
        treat this as a lost-link failsafe, not trust the last values."""
        with self.lock:
            if self.last_update_at is None:
                return True
            return time.time() - self.last_update_at > max_age_sec

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.ser.close()
