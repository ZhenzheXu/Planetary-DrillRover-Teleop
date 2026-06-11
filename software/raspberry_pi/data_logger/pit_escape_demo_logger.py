#!/usr/bin/env python3
from rover_scene_logger_common import record

record(
    scene="pit_escape_demo",
    port="/dev/serial/by-path/platform-xhci-hcd.0-usb-0:2:1.0-port0",
    baud=115200,
    duration=None,
)
