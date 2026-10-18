#!/usr/bin/env python3
"""Exercise actual backend CLI -> UDP bridge -> WebSocket with lockstep packets.

Run with --model-dir /path/to/micro-agu/simulation.
Uses isolated ephemeral ports and temporary recordings; does not run autopilot.
"""
import argparse
import asyncio
import json
from pathlib import Path
import queue
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import websockets


def port(kind):
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def ready(process, marker):
    lines = queue.Queue()
    def read():
        for line in process.stdout:
            lines.put(line)
    threading.Thread(target=read, daemon=True).start()
    while True:
        line = lines.get(timeout=45)
        if marker in line:
            return


async def exercise(model_dir):
    udp, ws, native = port(socket.SOCK_DGRAM), port(socket.SOCK_STREAM), port(socket.SOCK_DGRAM)
    children = []
    with tempfile.TemporaryDirectory() as tmp:
        recording = Path(tmp)/'run.jsonl'
        try:
            bridge = subprocess.Popen([sys.executable, str(Path(__file__).with_name('bridge.py')),
                                       '--udp', str(udp), '--port', str(ws), '--record', str(recording)],
                                      stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            children.append(bridge)
            ready(bridge, 'UDP')
            backend = subprocess.Popen([sys.executable, '-u', '-m', 'microagu_sim.compat.json_backend',
                                        '--port', str(native), '--viewer-port', str(udp)],
                                       stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True)
            children.append(backend)
            ready(backend, 'Listening')
            # Initialization trim is enough to test transport without injecting a control maneuver.
            async with websockets.connect(f'ws://127.0.0.1:{ws}', origin='http://127.0.0.1:5173') as viewer:
                observed = []
                async def receive():
                    while True:
                        observed.append(json.loads(await viewer.recv()))
                task = asyncio.create_task(receive())
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sitl:
                    sitl.setblocking(False)
                    loop = asyncio.get_running_loop()
                    for i in range(400):
                        packet = struct.pack('<HHI16H', 18458, 400, i, *([1500]*16))
                        await loop.sock_sendto(sitl, packet, ('127.0.0.1', native))
                        reply, _ = await asyncio.wait_for(loop.sock_recvfrom(sitl, 4096), 3)
                        assert abs(json.loads(reply)['timestamp']-(i+1)/400) < 1e-8
                    # Let the independent bridge/client consume the final datagrams.
                    await asyncio.sleep(.1)
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
                assert len(observed) >= 2, len(observed)
                assert all(x['schema'] == 'microagu.state.v1' for x in observed)
                assert all(len(x['state']) == 26 and len(x['brake_mm']) == 2 for x in observed)
                assert observed[-1]['time_s'] > .9
                assert all(not x['released'] for x in observed)
                print(f'PASS: 400 lockstep responses, {len(observed)} live frames; pose and actuation exported.')
            bridge.send_signal(2)
            bridge.wait(timeout=5)
            rows = [json.loads(line) for line in recording.read_text().splitlines()]
            assert 25 <= len(rows) <= 31, len(rows)
            assert all(b['time_s'] > a['time_s'] for a,b in zip(rows, rows[1:]))
            print(f'PASS: {len(rows)} recorded frames at bounded simulation rate.')
        finally:
            for child in children:
                if child.poll() is None:
                    child.terminate()
                    child.wait(timeout=5)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model-dir', type=Path, required=True)
    asyncio.run(exercise(p.parse_args().model_dir))
