#!/usr/bin/env python3
"""Record full viewer state while running the unchanged native AUTO test.

Run from the ArduPilot checkout using its autotest Python environment.
Arguments after -- are forwarded to compare_autotest.py. BUILDLOGS and
PG_CONTROLLER_CONFIG retain their usual meanings. This observer does not
modify commands, controller settings, or test assertions. With --turbulence,
repeatable gust velocity drives the physics and matching wind/airspeed sensors.
"""
import argparse
import json
import os
from pathlib import Path
import runpy
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--model-dir', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--turbulence', action='store_true')
parser.add_argument('--wind-ned', type=float, nargs=3, default=[0., 0., 0.], metavar=('N', 'E', 'D'), help='Steady earth-frame air velocity in m/s, applied after release')
args, forwarded = parser.parse_known_args()
if forwarded[:1] == ['--']:
    forwarded = forwarded[1:]
sys.path.insert(0, str(args.model_dir.resolve()))
from microagu_sim.compat.json_backend import Backend
from turbulence import SETTINGS, respond_with_wind

original = Backend.respond
last_time = -float('inf')
args.output.parent.mkdir(parents=True, exist_ok=True)
with args.output.open('x') as recording:
    def observe(self, packet):
        global last_time
        response = respond_with_wind(self, packet, original, args.wind_ned, args.turbulence) if args.turbulence or any(args.wind_ned) else original(self, packet)
        if self.time >= last_time + 1 / 30:
            state = self.viewer_state()
            state['wind_ned_m_s'] = self._viewer_wind.tolist() if args.turbulence or any(args.wind_ned) else [0., 0., 0.]
            if args.turbulence:
                state['turbulence'] = 'moderate synthetic gusts'
            recording.write(json.dumps(state, allow_nan=False) + '\n')
            last_time = self.time
        return response

    Backend.respond = observe
    study=Path(__file__).resolve().parents[3]/'research/archive/sitl-studies'
    os.environ['MICROAGU_SIM_ROOT']=str(args.model_dir.resolve())
    sys.path.insert(0,str(study))
    sys.argv = [str(study / 'compare_autotest.py'), *forwarded]
    try:
        runpy.run_path(sys.argv[0], run_name='__main__')
    finally:
        Backend.respond = original
        if args.turbulence:
            args.output.with_suffix('.turbulence.json').write_text(json.dumps(dict(SETTINGS, mean_wind_ned_m_s=args.wind_ned), indent=2)+'\n')

        args.output.with_suffix('.wind.json').write_text(json.dumps(dict(mean_wind_ned_m_s=args.wind_ned, gusts=args.turbulence), indent=2)+'\n')
