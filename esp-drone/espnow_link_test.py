#!/usr/bin/env python3
"""Propeller-off test for the full-duplex cflib ESP-NOW link."""

import logging
import threading
import time

from cflib.crazyflie import Crazyflie

from espnow_driver import register_espnow_driver


URI = "espnow://COM9/115200"
TIMEOUT_SECONDS = 12.0


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    print("Registering ESP-NOW driver...", flush=True)
    register_espnow_driver()

    connected = threading.Event()
    failed = threading.Event()
    result = {"message": ""}

    cf = Crazyflie(rw_cache="./cache")

    def on_link_established(uri):
        print(f"First valid CRTP reply received from {uri}", flush=True)

    def on_connected(uri):
        print(f"CONNECTED: TOCs downloaded from {uri}", flush=True)
        connected.set()

    def on_failed(uri, message):
        result["message"] = message
        print(f"CONNECTION FAILED: {message}", flush=True)
        failed.set()

    def on_lost(uri, message):
        result["message"] = message
        print(f"CONNECTION LOST: {message}", flush=True)
        failed.set()

    cf.link_established.add_callback(on_link_established)
    cf.connected.add_callback(on_connected)
    cf.connection_failed.add_callback(on_failed)
    cf.connection_lost.add_callback(on_lost)

    print(f"Opening {URI}...", flush=True)
    cf.open_link(URI)

    deadline = time.monotonic() + TIMEOUT_SECONDS

    while (
        not connected.is_set()
        and not failed.is_set()
        and time.monotonic() < deadline
    ):
        time.sleep(0.05)

    if connected.is_set():
        print("PASS: full-duplex cflib communication works.", flush=True)
    elif failed.is_set():
        print(f"FAIL: {result['message']}", flush=True)
    else:
        print(
            "FAIL: no valid CRTP reply reached Python within "
            f"{TIMEOUT_SECONDS:.0f} seconds.",
            flush=True,
        )

    cf.close_link()


if __name__ == "__main__":
    main()