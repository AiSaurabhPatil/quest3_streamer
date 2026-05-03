from __future__ import annotations

import os
import pickle
import socket
import struct
from typing import Any


HEADER_SIZE = 4
MAX_MESSAGE_BYTES = 1024 * 1024 * 1024


class IPCClosedError(RuntimeError):
    """Raised when the recorder IPC channel closes before a full message arrives."""


def send_message(channel: int | socket.socket, message: dict[str, Any]) -> None:
    payload = pickle.dumps(message, protocol=pickle.HIGHEST_PROTOCOL)
    if len(payload) > MAX_MESSAGE_BYTES:
        raise ValueError(f"Recorder IPC message is too large: {len(payload)} bytes")
    _write_all(channel, struct.pack("!I", len(payload)))
    _write_all(channel, payload)


def receive_message(channel: int | socket.socket) -> dict[str, Any]:
    header = _read_exact(channel, HEADER_SIZE)
    payload_size = struct.unpack("!I", header)[0]
    if payload_size > MAX_MESSAGE_BYTES:
        raise ValueError(f"Recorder IPC message is too large: {payload_size} bytes")
    payload = _read_exact(channel, payload_size)
    message = pickle.loads(payload)
    if not isinstance(message, dict):
        raise TypeError(f"Recorder IPC expected a dict message, got {type(message).__name__}")
    return message


def _write_all(channel: int | socket.socket, payload: bytes) -> None:
    if isinstance(channel, int):
        view = memoryview(payload)
        while view:
            written = os.write(channel, view)
            view = view[written:]
        return
    channel.sendall(payload)


def _read_exact(channel: int | socket.socket, byte_count: int) -> bytes:
    chunks = []
    bytes_remaining = byte_count
    while bytes_remaining > 0:
        if isinstance(channel, int):
            chunk = os.read(channel, bytes_remaining)
        else:
            chunk = channel.recv(bytes_remaining)
        if not chunk:
            raise IPCClosedError("Recorder IPC channel closed")
        chunks.append(chunk)
        bytes_remaining -= len(chunk)
    return b"".join(chunks)
