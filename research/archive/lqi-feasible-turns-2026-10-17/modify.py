"""AI-assisted research ablations: feasible reference map and channel anti-windup."""
import json
from pathlib import Path
HERE=Path(__file__).resolve().parent
def transform(cpp,hdr):
    nodes=[r for r in json.loads((HERE/'trims.json').read_text()) if r['feasible'] and r['residual']<1e-6 and abs(r['rate'])<=40]
    rates=sorted({r['rate'] for r in nodes})
    groups=[sorted([n for n in nodes if n['rate']==r],key=lambda n:n['speed']) for r in rates]
    assert all(g[0]['speed']==3.8 and g[-1]['speed']>=4.2 for g in groups)
    values=[];offsets=[0]
    def arr(a):return '{'+','.join(f'{v:.9e}f' for v in a)+'}'
    for g in groups:
        values += ['{'+f'{n["speed"]:.3f}f,'+arr(n['x'])+','+arr(n['u'])+'}' for n in g];offsets.append(len(values))
    table='''
// AI-assisted research table: constrained nonlinear steady turns, as-built model.
struct TurnTrim { float speed; float x[16]; float u[3]; };
static constexpr float trim_rates[] = '''+arr(rates)+''';
static constexpr uint8_t trim_offsets[] = {'''+','.join(map(str,offsets))+'''};
static constexpr TurnTrim trim_points[] = {'''+','.join(values)+'''};
static float turn_trim(float rate, float speed, float *x, float *u)
{
    constexpr uint8_t count = sizeof(trim_rates)/sizeof(trim_rates[0]);
    rate = constrain_float(rate, trim_rates[0], trim_rates[count-1]);
    uint8_t lo=0;
    while (lo+2 < count && rate > trim_rates[lo+1]) { lo++; }
    const float mix=(rate-trim_rates[lo])/(trim_rates[lo+1]-trim_rates[lo]);
    const float maximum=(1-mix)*trim_points[trim_offsets[lo+1]-1].speed+mix*trim_points[trim_offsets[lo+2]-1].speed;
    speed=constrain_float(speed, 3.8f, maximum);
    const float speed_fraction=(speed-3.8f)/(maximum-3.8f);
    for (uint8_t j=0; j<16; j++) { x[j]=0; }
    for (uint8_t j=0; j<3; j++) { u[j]=0; }
    for (uint8_t side=0; side<2; side++) {
        uint8_t a=trim_offsets[lo+side];
        const uint8_t end=trim_offsets[lo+side+1];
        const float row_speed=3.8f+speed_fraction*(trim_points[end-1].speed-3.8f);
        while (a+2<end && row_speed>trim_points[a+1].speed) { a++; }
        const float f=constrain_float((row_speed-trim_points[a].speed)/(trim_points[a+1].speed-trim_points[a].speed),0,1);
        const float w=side ? mix : 1-mix;
        for (uint8_t j=0; j<16; j++) { x[j]+=w*((1-f)*trim_points[a].x[j]+f*trim_points[a+1].x[j]); }
        for (uint8_t j=0; j<3; j++) { u[j]+=w*((1-f)*trim_points[a].u[j]+f*trim_points[a+1].u[j]); }
    }
    return speed;
}
'''
    cpp=cpp.replace('using namespace MicroAGU_LQI;','using namespace MicroAGU_LQI;\n'+table)
    hdr=hdr.replace('    AP_Float _turn_max;','    AP_Float _turn_max;\n    AP_Int8 _trim_map;\n    AP_Int8 _antiwindup_mode;\n    AP_Int8 _observer_trim;')
    cpp=cpp.replace('    AP_GROUPEND','''    // @Param: TMAP
    // @DisplayName: Research feasible turn trim map
    // @Description: Use constrained nonlinear trim references and feasible speed limits. Research model only.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("TMAP", 6, AP_CustomControl_LQI, _trim_map, 0),
    // @Param: AWMODE
    // @DisplayName: Research anti-windup allocation
    // @Description: Select full inverse or dominant-channel back calculation for research comparisons.
    // @Values: 0:Full inverse,1:Dominant channel
    // @User: Advanced
    AP_GROUPINFO("AWMODE", 7, AP_CustomControl_LQI, _antiwindup_mode, 0),
    // @Param: OBSTRIM
    // @DisplayName: Research observer equilibrium correction
    // @Description: Recenter the fixed observer dynamics on the nonlinear turn equilibrium.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("OBSTRIM", 8, AP_CustomControl_LQI, _observer_trim, 0),
    AP_GROUPEND''')
    cpp=cpp.replace('    float predicted[16]{};','''    float observer_x[16]{}, observer_u[3]{};
    if (_observer_trim) {
        float r,p,y; in.state.attitude_body_to_ned.to_euler(r,p,y);
        const float turn=(in.state.body_rate_rads.y*sinf(r)+in.state.body_rate_rads.z*cosf(r))/MAX(cosf(p),0.3f);
        turn_trim(degrees(turn),in.state.true_airspeed_mps,observer_x,observer_u);
    }
    float predicted[16]{};''')
    cpp=cpp.replace('predicted[i] += A[i][j]*_state[j];','predicted[i] += A[i][j]*(_state[j]-(_observer_trim ? observer_x[j] : 0));')
    cpp=cpp.replace('B[i][j]*(_applied[j]-TRIM[j])/US[j]','B[i][j]*(_applied[j]-(_observer_trim ? observer_u[j] : TRIM[j]))/US[j]')
    cpp=cpp.replace('    float y[9];\n    measurements(in, y);\n    const uint8_t indices[9] = {0, 1, 5, 6, 7, 8, 9, 10, 15};\n    for (uint8_t i = 0;', '    if (_observer_trim) { for (uint8_t i=0; i<16; i++) { predicted[i]+=observer_x[i]; } }\n    float y[9];\n    measurements(in, y);\n    const uint8_t indices[9] = {0, 1, 5, 6, 7, 8, 9, 10, 15};\n    for (uint8_t i = 0;')
    cpp=cpp.replace('const float speed_target =','float speed_target =')
    cpp=cpp.replace('    const float errors[3] = {','''    float trim_x[16]{}, trim_u[3]{};
    if (_trim_map) { speed_target=turn_trim(degrees(_turn_reference),speed_target,trim_x,trim_u); }
    const float errors[3] = {''')
    cpp=cpp.replace('        _integral[j] = constrain_float','''        if (_antiwindup_mode) {
            // Preserve turn integration when only common brake clips. Each
            // correction uses the dominant actuator and its actual integral gain.
            const uint8_t actuator[3]={0,2,1};
            const uint8_t i=actuator[j];
            correction=(_requested[i]-_applied[i])/(US[i]*K[i][16+j]);
        }
        _integral[j] = constrain_float''')
    cpp=cpp.replace('    for (uint8_t i = 0; i < 3; i++) {\n        float feedback = 0;', '''    if (_trim_map) {
        for (uint8_t j=0; j<16; j++) { target_state[j]=trim_x[j]+XREF[j][0]*references[0]+XREF[j][3]*references[3]; }
    }
    for (uint8_t i = 0; i < 3; i++) {
        float feedback = 0;''')
    cpp=cpp.replace('        _requested[i] = TRIM[i]+US[i]*(feedforward-feedback)+_matching[i];','''        if (_trim_map) { feedforward=(trim_u[i]-TRIM[i])/US[i]+UREF[i][3]*references[3]; }
        _requested[i] = TRIM[i]+US[i]*(feedforward-feedback)+_matching[i];''')
    cpp=cpp.replace('    _held = in.previous.applied;', '''    if (_trim_map) {
        float x[16],u[3];
        turn_trim(0,in.reference.equivalent_airspeed_mps*in.state.eas_to_tas,x,u);
        const uint8_t hidden[7]={2,3,4,11,12,13,14};
        for (uint8_t j=0; j<7; j++) { _state[hidden[j]]=x[hidden[j]]; }
    }
    _held = in.previous.applied;''')
    cpp=cpp.replace('        _requested[j] = _applied[j];','''        if (_trim_map) {
            float x[16],u[3];
            turn_trim(0,in.reference.equivalent_airspeed_mps*in.state.eas_to_tas,x,u);
            feedback=0;
            for (uint8_t k=0; k<16; k++) { feedback+=K[j][k]*(_state[k]-x[k]); }
            _matching[j]=_applied[j]-u[j]+US[j]*feedback;
        }
        _requested[j] = _applied[j];''')
    return cpp,hdr
