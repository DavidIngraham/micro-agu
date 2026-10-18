"""Export a reviewable default-preserving reference filter, without truth injection."""
import difflib
import importlib.util
import json
from pathlib import Path
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('base',HERE.parent/'lqi-balanced-2026-10-17/campaign.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
manifest=json.loads((HERE/'original/manifest.json').read_text())
original={Path(r['path']):(HERE/'original'/r['backup']).read_text() for r in manifest if r['backup'] in ['2','3','4']}
cpp,header,frontend=b.governor(original[b.FILES[4]],original[b.FILES[3]],original[b.FILES[2]])
header=b.replace(header,'    AP_Float _height_tc;','    AP_Float _height_tc;\n    AP_Float _height_rate_tc;\n    float _smoothed_climb;')
cpp=b.replace(cpp,'    AP_GROUPEND','''    // @Param: HRTC
    // @DisplayName: LQI climb reference smoothing
    // @Description: Time constant for climb reference smoothing before altitude integration and equilibrium feedforward. Zero preserves the unfiltered reference.
    // @Units: s
    // @Range: 0 3
    // @User: Advanced
    AP_GROUPINFO("HRTC", 4, AP_CustomControl_LQI, _height_rate_tc, 0.0f),
    AP_GROUPEND''')
cpp=b.replace(cpp,'    return isfinite(_height_up.get())',
    '    return isfinite(_height_rate_tc.get()) && _height_rate_tc >= 0 && _height_rate_tc <= 3 &&\n        isfinite(_height_up.get())')
cpp=b.replace(cpp,'    _height_reference = 0;','    _height_reference = 0;\n    _smoothed_climb = 0;')
cpp=b.replace(cpp,'    const float governed_climb = constrain_float(climb, -_height_down.get(), _height_up.get());',
    '''    const float requested_climb = constrain_float(climb, -_height_down.get(), _height_up.get());
    _smoothed_climb += DT/(_height_rate_tc+DT)*(requested_climb-_smoothed_climb);
    const float governed_climb = _smoothed_climb;''')
patch=''
for p,text in zip([b.FILES[4],b.FILES[3],b.FILES[2]],[cpp,header,frontend]):
    rel=str(p.relative_to(b.AP))
    patch+=''.join(difflib.unified_diff(original[p].splitlines(True),text.splitlines(True),fromfile='a/'+rel,tofile='b/'+rel))
(HERE/'reference-filter.patch').write_text(patch)
print('Exported optional CP3_HRTC filter plus existing configurable reference limits; not applied.')
