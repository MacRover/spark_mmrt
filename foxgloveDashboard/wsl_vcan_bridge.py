#!/usr/bin/env python3
"""
Bridge Linux SocketCAN (vcan/can) frames to Foxglove WebSocket as JSON.

Designed to pair with:
  foxgloveDashboard/test.ts

Default behavior:
  - Reads from SocketCAN interface: vcan0
  - Publishes on topic: /can0/received_frames
  - Serves Foxglove WebSocket on: 0.0.0.0:8765
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from dataclasses import asdict, dataclass
from typing import Any

import can
import foxglove


@dataclass
class CanFrameMessage:
    """JSON payload shape consumed by foxgloveDashboard/test.ts."""

    arbId: int
    data: list[int]
    channel: str
    dlc: int
    isExtendedId: bool
    isFd: bool
    isErrorFrame: bool
    timestampSec: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Bridge SocketCAN frames into Foxglove WebSocket",
    )
    parser.add_argument("--can-iface", default="vcan0", help="SocketCAN interface")
    parser.add_argument("--topic", default="/can0/received_frames", help="Foxglove topic")
    parser.add_argument("--host", default="0.0.0.0", help="WebSocket bind host")
    parser.add_argument("--port", type=int, default=8765, help="WebSocket bind port")
    parser.add_argument(
        "--receive-own",
        action="store_true",
        help="Receive frames sent by this process/socket (if supported)",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=100,
        help="Print one status line every N forwarded frames (0 disables)",
    )
    return parser.parse_args()


def build_payload(msg: can.Message, channel_name: str) -> dict[str, Any]:
    payload = CanFrameMessage(
        arbId=int(msg.arbitration_id),
        data=[int(b) & 0xFF for b in msg.data],
        channel=channel_name,
        dlc=int(msg.dlc),
        isExtendedId=bool(msg.is_extended_id),
        isFd=bool(getattr(msg, "is_fd", False)),
        isErrorFrame=bool(getattr(msg, "is_error_frame", False)),
        timestampSec=float(getattr(msg, "timestamp", time.time())),
    )
    return asdict(payload)


def main() -> int:
    args = parse_args()
    running = True

    def handle_signal(_sig: int, _frame: Any) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    foxglove.start_server(host=args.host, port=args.port, name="WSL SocketCAN Bridge")
    print(f"[bridge] Foxglove server listening at ws://{args.host}:{args.port}")
    print(f"[bridge] Reading SocketCAN interface: {args.can_iface}")
    print(f"[bridge] Publishing topic: {args.topic}")

    bus = can.Bus(
        interface="socketcan",
        channel=args.can_iface,
        receive_own_messages=args.receive_own,
    )

    forwarded = 0
    try:
        while running:
            msg = bus.recv(timeout=0.2)
            if msg is None:
                continue

            foxglove.log(args.topic, build_payload(msg, args.can_iface))
            forwarded += 1

            if args.print_every > 0 and forwarded % args.print_every == 0:
                print(f"[bridge] forwarded={forwarded} last_id=0x{msg.arbitration_id:X}")
    finally:
        bus.shutdown()
        print(f"[bridge] stopped, total forwarded={forwarded}")

    return 0


if __name__ == "__main__":
    sys.exit(main())

