"""Predeclared timing sensitivity cases around the coarse-grid candidate."""
import json
from native import HERE, run

cases = []
for h in [.75, 1.25, 1.5]:
    for b in [36, 54, 68]:
        cases.append(dict(id=f"fine-{h}-{b}", kind="landing", height=30,
                          flare_height=h, flare_brake=b))
for bias in [-.5, -.25, 0, .25, .5]:
    for sink in [-.3, 0, .3]:
        cases.append(dict(id=f"bias-{bias}-{sink}", kind="landing", height=30,
                          flare_height=1, flare_brake=54,
                          height_bias=bias, sink_perturbation=sink))
(HERE/"refine-protocol.json").write_text(json.dumps(cases, indent=2)+"\n")
results = []
for spec in cases:
    result = run(spec)
    (HERE/"native-runs"/(spec["id"]+".json")).write_text(json.dumps(result))
    results.append({k:v for k,v in result.items() if k != "samples"})
    print(spec["id"], result["metrics"]["contact"], flush=True)
(HERE/"refine-summary.json").write_text(json.dumps(results, indent=2)+"\n")
