#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
import signal
import threading
import time
from dataclasses import asdict, dataclass
from typing import Any, Optional

import can
from foxglove_websocket.server import FoxgloveServer

CAN_FRAME_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "arbId": {"type": "integer"},
        "data": {"type": "array", "items": {"type": "integer"}},
        "channel": {"type": "string"},
        "dlc": {"type": "integer"},
        "isExtendedId": {"type": "boolean"},
        "isFd": {"type": "boolean"},
        "isErrorFrame": {"type": "boolean"},
        "timestampSec": {"type": "number"},
    },
    "required": [
        "arbId",
        "data",
        "channel",
        "dlc",
        "isExtendedId",
        "isFd",
        "isErrorFrame",
        "timestampSec",
    ],
}


@dataclass
class CanFrameMessage:
    arbId: int
    data: list[int]
    channel: str
    dlc: int
    isExtendedId: bool
    isFd: bool
    isErrorFrame: bool
    timestampSec: float


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Windows PCAN -> Foxglove WebSocket (JSON)")
    p.add_argument("--can-iface", default="PCAN_USBBUS1", help="PCAN channel e.g. PCAN_USBBUS1")
    p.add_argument("--bitrate", type=int, default=1000000, help="CAN bitrate")
    p.add_argument("--topic", default="/can0/received_frames", help="Foxglove topic")
    p.add_argument("--host", default="0.0.0.0", help="WebSocket bind host")
    p.add_argument("--port", type=int, default=8765, help="WebSocket bind port")
    p.add_argument("--print-every", type=int, default=100, help="Print every N frames (0 disables)")
    return p.parse_args()


def build_payload(msg: can.Message, channel_name: str) -> dict[str, Any]:
    return asdict(
        CanFrameMessage(
            arbId=int(msg.arbitration_id),
            data=[int(b) & 0xFF for b in msg.data],
            channel=channel_name,
            dlc=int(msg.dlc),
            isExtendedId=bool(msg.is_extended_id),
            isFd=bool(getattr(msg, "is_fd", False)),
            isErrorFrame=bool(getattr(msg, "is_error_frame", False)),
            timestampSec=float(getattr(msg, "timestamp", time.time())),
        )
    )


async def main_async() -> int:
    args = parse_args()
    loop = asyncio.get_running_loop()

    stop = asyncio.Event()

    def _sigint(_sig=None, _frame=None):
        stop.set()

    signal.signal(signal.SIGINT, _sigint)

    # Create Foxglove server INSIDE running loop, but start it synchronously (0.1.4 API)
    server = FoxgloveServer(host=args.host, port=args.port, name="Windows PCAN Bridge")
    server.start()

    chan_id = await server.add_channel(
        {
            "topic": args.topic,
            "encoding": "json",
            "schemaName": "CanFrameMessage",
            "schema": json.dumps(CAN_FRAME_JSON_SCHEMA),
            "schemaEncoding": "jsonschema",
        }
    )

    print(f"[bridge] Foxglove WS: ws://{args.host}:{args.port}")
    print(f"[bridge] Reading: {args.can_iface} @ {args.bitrate} bps")
    print(f"[bridge] Topic: {args.topic}")

    # Open CAN bus
    bus: can.BusABC = can.Bus(interface="pcan", channel=args.can_iface, bitrate=args.bitrate)

    forwarded = 0

    def can_thread():
        nonlocal forwarded
        try:
            while not stop.is_set():
                msg = bus.recv(timeout=0.2)
                if msg is None:
                    continue

                payload = build_payload(msg, args.can_iface)
                data = json.dumps(payload).encode("utf-8")
                ts = time.time_ns()

                # send_message is async in this foxglove-websocket version
                asyncio.run_coroutine_threadsafe(
                    server.send_message(chan_id, ts, data),
                    loop,
                )

                forwarded += 1
                if args.print_every > 0 and forwarded % args.print_every == 0:
                    print(f"[bridge] forwarded={forwarded} last_id=0x{msg.arbitration_id:X}")
        finally:
            try:
                bus.shutdown()
            except Exception:
                pass

    t = threading.Thread(target=can_thread, daemon=True)
    t.start()

    # Keep loop alive until Ctrl+C
    await stop.wait()

    print(f"[bridge] stopping, total forwarded={forwarded}")
    server.close()
    await server.wait_closed()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main_async()))
