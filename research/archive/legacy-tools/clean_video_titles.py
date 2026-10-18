"""Remove the old title band from published captures without rerunning physics.

Usage: python research/archive/legacy-tools/clean_video_titles.py RAW_STUDY_ROOT
Uses original browser captures, preserving all telemetry and motion. The current
viewer exporter omits these titles itself; this migrates existing recordings.
"""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import argparse
import subprocess
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('raw', type=Path)
a = p.parse_args()
assets = Path(__file__).resolve().parents[1] / 'docs/assets/viewer'
jobs = [('viewer-showcase-turbulence/browser-capture-controls.webm', 'reversing-turn-controls'),
        *[(f'viewer-wind-comparison/{c}/browser-capture.webm', f'auto-{c}-profile') for c in ('calm', 'gust', 'crosswind')]]
def encode(job):
    source, name = job
    source = a.raw / source
    base = assets / (name + '-plain')
    common = ['ffmpeg', '-y', '-v', 'error', '-i', str(source), '-filter_complex', '[0:v]split=2[top][bottom];[top]crop=iw:40:0:0[t];[bottom]crop=iw:ih-88:0:88[b];[t][b]vstack=inputs=2[out]', '-map', '[out]', '-an']
    subprocess.run([*common, '-c:v', 'libx264', '-crf', '22', '-preset', 'fast', '-threads', '4', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', '-r', '30', str(base.with_suffix('.mp4'))], check=True)
    subprocess.run([*common, '-c:v', 'libvpx-vp9', '-crf', '34', '-b:v', '0', '-cpu-used', '4', '-threads', '4', '-r', '30', str(base.with_suffix('.webm'))], check=True)
    subprocess.run(['ffmpeg', '-y', '-v', 'error', '-ss', '12', '-i', str(base.with_suffix('.mp4')), '-frames:v', '1', '-q:v', '2', '-update', '1', str(base.with_suffix('.jpg'))], check=True)
    print(name + ': complete', flush=True)
with ThreadPoolExecutor(max_workers=2) as pool:
    list(pool.map(encode, jobs))
