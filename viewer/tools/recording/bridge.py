#!/usr/bin/env python3
"""Local-only UDP -> WebSocket truth-state bridge with optional JSONL recording."""
import argparse
import asyncio
import json
from pathlib import Path
import websockets
from websockets.protocol import State


def validate(frame):
    import math
    if not isinstance(frame, dict):
        raise ValueError('Expected a state object')
    if frame.get('schema') != 'microagu.state.v1':
        raise ValueError('Unknown state schema')
    for name, size in [('state', 26), ('brake_mm', 2), ('joint_rotvec_rad', 3), ('joint_rate_rad_s', 3)]:
        values = frame.get(name, [])
        if len(values) != size or not all(isinstance(v, (float, int)) and math.isfinite(v) for v in values):
            raise ValueError(f'Invalid {name}')
    for name in ['time_s', 'rpm', 'voltage_v', 'throttle']:
        if not isinstance(frame.get(name), (float, int)) or not math.isfinite(frame[name]):
            raise ValueError(f'Invalid {name}')
    if not isinstance(frame.get('released'), bool):
        raise ValueError('Invalid release state')
    return frame


class Receiver(asyncio.DatagramProtocol):
    def __init__(self, clients, record):
        self.clients, self.record = clients, record
        self.last_time = None
        self.received = 0

    def datagram_received(self, data, address):
        try:
            frame = validate(json.loads(data))
            if frame['time_s'] == self.last_time:
                return
            if self.last_time is not None and frame['time_s'] < self.last_time:
                # Keep recordings parseable: a restarted run needs a new file.
                if self.record:
                    print('Simulation restarted: recording stopped; restart bridge for a new recording.', flush=True)
                    self.record.close()
                    self.record = None
            self.last_time = frame['time_s']
            message = json.dumps(frame, allow_nan=False)
        except (ValueError, TypeError, KeyError):
            return
        self.received += 1
        if self.record:
            self.record.write(message + '\n')
            if self.received % 30 == 0:
                self.record.flush()
        for queue in tuple(self.clients):
            if queue.full():
                queue.get_nowait()
            queue.put_nowait(message)


async def serve(args):
    clients = set()
    record = Path(args.record).open('x') if args.record else None
    receiver = Receiver(clients, record)
    transport, _ = await asyncio.get_running_loop().create_datagram_endpoint(
        lambda: receiver, local_addr=('127.0.0.1', args.udp))

    async def client(ws):
        queue = asyncio.Queue(maxsize=1)
        clients.add(queue)
        try:
            while ws.state is State.OPEN:
                try:
                    message = await asyncio.wait_for(queue.get(), timeout=1)
                except asyncio.TimeoutError:
                    continue
                await asyncio.wait_for(ws.send(message), timeout=2)
        except (websockets.ConnectionClosed, asyncio.TimeoutError):
            pass
        finally:
            clients.discard(queue)

    try:
        # Loopback binding and origin allow-list prevent arbitrary websites reading local flight state.
        async with websockets.serve(client, '127.0.0.1', args.port,
                                    origins=[None, 'http://127.0.0.1:5173', 'http://localhost:5173',
                                             'http://127.0.0.1:4173', 'http://localhost:4173'],
                                    max_size=16384):
            print(f'UDP {args.udp} -> ws://127.0.0.1:{args.port}', flush=True)
            await asyncio.Future()
    finally:
        transport.close()
        if receiver.record:
            receiver.record.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--udp', type=int, default=9010)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--record', help='New JSONL file; existing files are never overwritten')
    try:
        asyncio.run(serve(parser.parse_args()))
    except KeyboardInterrupt:
        pass
