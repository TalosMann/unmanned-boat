"""
Prints live i-BUS channel values from the FlySky IA10B receiver — no
motors, no servo, nothing powered beyond the receiver itself. Use this to
confirm which physical stick/switch maps to which channel number BEFORE
powering any ESC: move one control at a time on the transmitter and watch
which column changes here.

Run on the Pi:
    python3 test_rc_diagnostic.py
Ctrl+C to stop.
"""

import time

import config
from remote_input import IBusReceiver


def main():
    print(f"[rc-diag] Connecting to i-BUS receiver on {config.REMOTE_SERIAL_PORT}...")
    receiver = IBusReceiver(config.REMOTE_SERIAL_PORT).start()

    print("[rc-diag] Waiting for first packet...")
    while receiver.get_channels() is None:
        time.sleep(0.1)
    print("[rc-diag] Receiving. Move one stick/switch at a time and watch which "
          "column changes. Ctrl+C to stop.\n")

    try:
        while True:
            channels = receiver.get_channels()
            stale = receiver.is_stale(config.REMOTE_LINK_TIMEOUT_SEC)
            label = "STALE" if stale else " live"
            cols = "  ".join(f"CH{i + 1}={v:4d}" for i, v in enumerate(channels[:10]))
            print(f"\r[{label}] {cols}", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[rc-diag] Stopped.")
    finally:
        receiver.stop()


if __name__ == "__main__":
    main()
