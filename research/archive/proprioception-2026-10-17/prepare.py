"""AI-assisted, reversible SITL-only sensing experiment. Never deploy this patch."""
import hashlib
import json
from pathlib import Path
import shutil
import sys
import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
AP = ROOT.parent/'ardupilot-browser-9dof'
VIEW = ROOT/'viewer'
FILES = [AP/'libraries/SITL/SIM_JSON.h', AP/'libraries/SITL/SIM_JSON.cpp',
         AP/'libraries/AP_CustomControl/AP_CustomControl_LQI.cpp',
         AP/'libraries/AP_CustomControl/AP_CustomControl_LQI_gains.h',
         VIEW/'packages/sim-browser/src/physics/physics.ts', VIEW/'tests/integration/stability-case.ts',
         VIEW/'generated/ardupilot/arduplane.js', VIEW/'generated/ardupilot/arduplane.wasm',
         VIEW/'assets/runtime/build.json', VIEW/'assets/runtime/RUST-NOTICES.txt']


def snapshot():
    dest = HERE/'original'
    if dest.exists():
        raise RuntimeError('Snapshot already exists; use restore or inspect before preparing again')
    dest.mkdir()
    manifest = []
    for i, p in enumerate(FILES):
        shutil.copyfile(p, dest/str(i))
        manifest.append(dict(path=str(p), backup=str(i), sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
    (dest/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')


def replace(path, old, new):
    text = path.read_text()
    assert text.count(old) == 1, (path, old[:60])
    path.write_text(text.replace(old, new))


def arrays_header(design):
    z = np.load(design/'design.npz')
    state = z['state']
    rp, rc = Rotation.from_quat(state[3:7]), Rotation.from_quat(state[16:20])
    measured = list(z['measured'])+[2, 3, 4]
    h = np.eye(16)[measured]
    noise = np.diag((np.r_[[np.deg2rad(1)]*2, [.2]*3, [np.deg2rad(1)]*3, .3,
                           [np.deg2rad(.5)]*3]/z['XS'][measured])**2)
    p = solve_discrete_are(z['A'].T, h.T, np.eye(16)*1e-5, noise)
    gain = np.linalg.solve(h@p@h.T+noise, h@p).T
    poles = np.linalg.eigvals((np.eye(16)-gain@h)@z['A'])
    assert max(abs(poles)) < 1
    arrays = dict(PROP_TRIM=(rp.inv()*rc).as_quat()[[3, 0, 1, 2]],
                  CANOPY_RATES=rc.inv().apply(state[23:26]), L_PROP=gain)
    def value(a):
        return '{'+', '.join(value(x) for x in a)+'}' if np.ndim(a) else f'{a:.9e}f'
    extra = '\nnamespace MicroAGU_LQI {\n'
    for name, a in arrays.items():
        extra += 'static constexpr float '+name+''.join(f'[{d}]' for d in a.shape)+' = '+value(a)+';\n'
    extra += '}\n'
    return extra


def prepare(design):
    snapshot()
    header = AP/'libraries/SITL/SIM_JSON.h'
    replace(header, 'private:\n', '''// Research-only simulated proprioceptor; removed after the campaign.
    static Quaternion research_relative;
    static Vector3f research_rate;
    static float research_mode;
    static uint32_t research_received_ms;

private:
''')
    replace(header, '} keytable[36] {', '} keytable[39] {')
    replace(header, '        { "battery", "current", &state.bat_amp, DATA_FLOAT, false },', '''        { "battery", "current", &state.bat_amp, DATA_FLOAT, false },
        { "", "microagu_relative", &research_relative, QUATERNION, false },
        { "", "microagu_rate", &research_rate, DATA_VECTOR3F, false },
        { "", "microagu_mode", &research_mode, DATA_FLOAT, false },''')
    cpp = AP/'libraries/SITL/SIM_JSON.cpp'
    replace(cpp, 'using namespace SITL;', '''using namespace SITL;
Quaternion JSON::research_relative;
Vector3f JSON::research_rate;
float JSON::research_mode;
uint32_t JSON::research_received_ms;
''')
    replace(cpp, '    const uint64_t received_bitmask = parse_sensors((const char *)(p1+1));', '''    research_mode = 0;
    const uint64_t received_bitmask = parse_sensors((const char *)(p1+1));
    const uint64_t research_mask = (1ULL << 36) | (1ULL << 37) | (1ULL << 38);
    if ((received_bitmask & research_mask) == research_mask) {
        research_received_ms = AP_HAL::millis();
    } else {
        research_mode = 0;
    }''')
    backend = AP/'libraries/AP_CustomControl/AP_CustomControl_LQI.cpp'
    replace(backend, '#include "AP_CustomControl_LQI_gains.h"', '''#include "AP_CustomControl_LQI_gains.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SIM_JSON.h>
#endif''')
    replace(backend, '    const float climb = in.reference.vertical', '''#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && AP_SIM_JSON_ENABLED
    // AI-assisted research: sensor or ideal feedback, payload EKF remains active.
    const unsigned proprio_mode = unsigned(SITL::JSON::research_mode);
    float proprio_observer[6] = {_state[2], _state[3], _state[4], _state[11], _state[12], _state[13]};
    if (proprio_mode > 0 && proprio_mode <= 3 &&
        in.time_ms-SITL::JSON::research_received_ms < 100) {
        const Quaternion relative = SITL::JSON::research_relative;
        if (!isfinite(relative.length_squared()) || fabsf(relative.length_squared()-1) > 0.01f) {
            return false;
        }
        const Quaternion trim_relative(PROP_TRIM[0], PROP_TRIM[1], PROP_TRIM[2], PROP_TRIM[3]);
        Vector3f angle;
        (trim_relative.inverse()*relative).to_axis_angle(angle);
        if (proprio_mode == 3) {
            const uint8_t observed[12] = {0, 1, 5, 6, 7, 8, 9, 10, 15, 2, 3, 4};
            for (uint8_t i = 0; i < 16; i++) {
                _state[i] = predicted[i];
                for (uint8_t j = 0; j < 12; j++) {
                    const float measurement = j < 9 ? y[j] : angle[j-9]/XS[2+j-9];
                    _state[i] += L_PROP[i][j]*(measurement-predicted[observed[j]]);
                }
            }
        } else {
            for (uint8_t j = 0; j < 3; j++) {
                _state[2+j] = angle[j]/XS[2+j];
            }
            if (proprio_mode == 2) {
                Matrix3f relative_rotation;
                relative.rotation_matrix(relative_rotation);
                const Vector3f canopy_rates = relative_rotation.transposed() *
                    (in.state.body_rate_rads+SITL::JSON::research_rate);
                for (uint8_t j = 0; j < 3; j++) {
                    _state[11+j] = (canopy_rates[j]-CANOPY_RATES[j])/XS[11+j];
                }
            }
        }
    }
#endif
    const float climb = in.reference.vertical''')
    replace(backend, '        printf("LQID,%lu", (unsigned long)in.time_ms);', '''#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && AP_SIM_JSON_ENABLED
        printf("LQIP,%lu,%u", (unsigned long)in.time_ms, proprio_mode);
        for (const float value : proprio_observer) { printf(",%.7g", (double)value); }
        for (const uint8_t j : {2, 3, 4, 11, 12, 13}) { printf(",%.7g", (double)_state[j]); }
        printf("\\n");
#endif
        printf("LQID,%lu", (unsigned long)in.time_ms);''')
    gains = AP/'libraries/AP_CustomControl/AP_CustomControl_LQI_gains.h'
    gains.write_text((design/'AP_CustomControl_LQI_gains.h').read_text()+arrays_header(design))
    physics = VIEW/'packages/sim-browser/src/physics/physics.ts'
    replace(physics, 'export class Physics {', '''export class Physics {
  researchSensor?: (state: number[], time: number) => Record<string, unknown>;
''')
    replace(physics, '      timestamp: this.time,', '      ...this.researchSensor?.(this.state, this.time),\n      timestamp: this.time,')
    harness = VIEW/'tests/integration/stability-case.ts'
    replace(harness, 'const config: PhysicsConfig = JSON.parse(', '''const sensor = await import("../../../research/archive/proprioception-2026-10-17/sensor.ts");
const config: PhysicsConfig = JSON.parse(''')
    replace(harness, 'const samples: unknown[] = [];', '''sim.physics; // sensor is attached after start creates the physics instance
const samples: unknown[] = [];''')
    replace(harness, '  await sim.upload(exampleMission(), () => {});', '''  sim.physics.researchSensor = sensor.makeSensor(process.env.PROPRIO_CASE ?? "none", spec.id);
  await sim.upload(exampleMission(), () => {});''')
    # No pre-launch access to physics (getter may not yet be initialized).
    replace(harness, 'sim.physics; // sensor is attached after start creates the physics instance\n', '')
    for p in FILES[:6]:
        shutil.copyfile(p, HERE/(p.name+'.experiment.txt'))


def restore():
    manifest = json.loads((HERE/'original/manifest.json').read_text())
    # Do not overwrite an unrelated concurrent edit to any patched source file.
    for row in manifest[:6]:
        p = Path(row['path'])
        current = hashlib.sha256(p.read_bytes()).hexdigest()
        allowed = {row['sha256']}
        candidates = [HERE/(p.name+'.experiment.txt')]
        candidates += list(HERE.glob(f'*-design/runtime/{p.name}'))
        allowed.update(hashlib.sha256(q.read_bytes()).hexdigest() for q in candidates if q.exists())
        if current not in allowed:
            raise RuntimeError(f'Refusing to overwrite an unrecognized concurrent edit: {p}')
    for row in manifest:
        p = Path(row['path'])
        shutil.copyfile(HERE/'original'/row['backup'], p)
        assert hashlib.sha256(p.read_bytes()).hexdigest() == row['sha256']
    (HERE/'restored.json').write_text(json.dumps(dict(verified_files=len(manifest)), indent=2)+'\n')


if __name__ == '__main__':
    restore() if sys.argv[1] == 'restore' else prepare(Path(sys.argv[1]))
