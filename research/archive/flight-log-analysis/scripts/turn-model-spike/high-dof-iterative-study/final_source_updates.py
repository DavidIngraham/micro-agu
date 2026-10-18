from pathlib import Path
R=Path(__file__).parent
s=(R/'soft_mode_check.py').read_text().split("cal={r['flight']")[0]
s=s.replace('float(np.angle(s[k],deg=True))','float(np.angle(s[kr],deg=True))').replace('float(co[k])','float(co[kr])').replace('soft-mode-phase-check','soft-roll-ridge-phase-check').replace('pitch/yaw spectral ridge','roll spectral ridge').replace('each signal high-band pitch/yaw ridge','each signal high-band roll ridge')
(R/'soft_roll_phase_check.py').write_text(s)
p=R/'tl_matched_diagnostic.py';s=p.read_text().replace('fig.tight_layout();fig.savefig','fig.tight_layout(rect=(0,0,1,.97));fig.savefig');p.write_text(s)
p=R/'plot_retained.py';s=p.read_text();line=next(l for l in s.splitlines() if l.startswith('rows=json.loads'));s=s.replace(line,"rows=json.loads((R/'atlas-results.json').read_text());v2=json.loads((R/'candidate-v2-atlas-results.json').read_text());rows+=v2+json.loads((R/'candidate-v3-atlas-results.json').read_text())");p.write_text(s)
for fname in ['plot_final_spectra.py','sustained_turn_crosscheck.py']:
 p=R/fname;s=p.read_text();s=s.replace("R/(f'preload_6.0-{day}-{start}.npz' if di==0 else f'candidate_v2-{day}-{start}.npz')","R/f'candidate_v3-{day}-{start}.npz'").replace("R/(f'preload_6.0-{day}-{start}.npz' if flight=='hood_river' else f'candidate_v2-{day}-{start}.npz')","R/f'candidate_v3-{day}-{start}.npz'");p.write_text(s)
