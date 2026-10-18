"""Summarize the bounded landing-aim iteration and audit native logs."""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil
from scipy.spatial.transform import Rotation, Slerp

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    summary = {}
    datasets = {}
    provenance = {}
    for name in ['screen-aim0', 'screen-sink14', 'heldout-baseline', 'heldout-candidate']:
        rows = json.loads((args.root/name/'results.json').read_text())
        datasets[name] = rows
        good = [r for r in rows if r['completed_contact']]
        errors = np.array([r['landing']['radial_error_m'] for r in good])
        sinks = np.array([r['landing']['contact']['velocity'][2] for r in good])
        speeds = [np.linalg.norm(r['landing']['contact']['velocity'][:2]) for r in good]
        summary[name] = dict(attempted=len(rows), completed=len(good),
                            within_5m=int(sum(errors <= 5)), within_10m=int(sum(errors <= 10)),
                            median_m=float(np.median(errors)), p90_m=float(np.percentile(errors,90)),
                            worst_m=float(max(errors)), median_sink_m_s=float(np.median(sinks)),
                            p90_sink_m_s=float(np.percentile(sinks,90)), median_groundspeed_m_s=float(np.median(speeds)))
        provenance[name] = json.loads((args.root/name/'manifest.json').read_text())
    audits=[]
    for name in ['heldout-baseline', 'heldout-candidate']:
        expected_sink=.25 if name.endswith('baseline') else 1.4
        for row in datasets[name]:
            directory=args.root/name/('moderate-%02d' % row['trial'])
            log=mavutil.mavlink_connection(str(directory/'flight.BIN'))
            params={}; active=0; max_common=0
            while True:
                msg=log.recv_match(type=['PARM','PGFL'])
                if msg is None: break
                if msg.get_type()=='PARM': params[msg.Name]=msg.Value
                elif msg.Active:
                    active+=1;max_common=max(max_common,msg.Common)
                    assert 0<=msg.Common<=60.001 and 0<=msg.Left<=100 and 0<=msg.Right<=100
            assert active and abs(params['TECS_LAND_SINK']-expected_sink)<1e-5
            assert params['LAND_FLARE_AIM']==50
            audits.append(dict(suite=name,trial=row['trial'],flare_samples=active,max_common=max_common,
                               sink_parameter=params['TECS_LAND_SINK']))
    baseline=datasets['heldout-baseline'];candidate=datasets['heldout-candidate']
    assert len(baseline)==len(candidate)==12
    for a,b in zip(baseline,candidate):
        ka={k:v for k,v in a['config'].items() if k not in ['flare_sink_m_s','record_viewer']}
        kb={k:v for k,v in b['config'].items() if k not in ['flare_sink_m_s','record_viewer']}
        assert ka==kb
    summary['paired_improved']=sum(b['landing']['radial_error_m']<a['landing']['radial_error_m'] for a,b in zip(baseline,candidate))
    summary['audits']=audits
    fig,ax=plt.subplots(1,3,figsize=(15,4.8))
    for rows,label,color in [(baseline,'Original sink assumption: 0.25 m/s','#c66b4e'),(candidate,'Updated sink assumption: 1.4 m/s','#228d8a')]:
        land=[r['landing'] for r in rows if r['completed_contact']]
        cross=[r['cross_track_error_m'] for r in land];along=[r['along_track_error_m'] for r in land]
        radial=np.sort([r['radial_error_m'] for r in land])
        ax[0].scatter(cross,along,label=label,color=color)
        ax[1].step(np.r_[0,radial,22],np.r_[0,np.arange(1,len(radial)+1)/len(rows),len(radial)/len(rows)],where='post',color=color,label=label)
        ax[2].scatter([r['radial_error_m'] for r in land],[r['contact']['velocity'][2] for r in land],color=color)
    for r in [5,10,20]:ax[0].add_patch(plt.Circle((0,0),r,fill=False,color='.8'))
    ax[0].scatter(0,0,marker='+',color='black',s=100);ax[0].set_aspect('equal',adjustable='datalim')
    ax[0].set(xlabel='Cross-track error (m)',ylabel='Along-track error (m; negative = short)',title='First ground-plane crossing')
    ax[1].set(xlabel='Distance from target (m)',ylabel='Fraction of all attempted runs',title='12 fresh paired conditions',ylim=(0,1.05))
    ax[1].axvline(5,color='.7',ls='--');ax[1].legend(fontsize=8,loc='lower right')
    ax[2].set(xlabel='Distance from target (m)',ylabel='Sink speed (m/s)',title='Accuracy and vertical speed')
    fig.tight_layout();fig.savefig(args.output/'landing-iteration.png',dpi=170);plt.close(fig)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    (args.output/'experiment.json').write_text(json.dumps(provenance,indent=2))
    # Show a representative successful example: closest to median miss among <=5 m.
    good=[r for r in candidate if r['completed_contact'] and r['landing']['radial_error_m']<=5]
    chosen=min(good,key=lambda r:abs(r['landing']['radial_error_m']-summary['heldout-candidate']['median_m']))
    directory=args.root/'heldout-candidate'/('moderate-%02d' % chosen['trial'])
    frames=json.loads((directory/'viewer.json').read_text())
    end=chosen['landing']['contact']['time']
    before=max((f for f in frames if f['time_s']<=end),key=lambda f:f['time_s'])
    after=min((f for f in frames if f['time_s']>=end),key=lambda f:f['time_s'],default=None)
    if after and after['time_s']>before['time_s']:
        ratio=(end-before['time_s'])/(after['time_s']-before['time_s'])
        state=np.array(before['state'])+ratio*(np.array(after['state'])-before['state'])
        for offset in (3,16):
            state[offset:offset+4]=Slerp([0,1],Rotation.from_quat([before['state'][offset:offset+4],after['state'][offset:offset+4]]))([ratio]).as_quat()[0]
        endpoint=before | {'time_s':end,'state':state.tolist()}
        for key in ['rpm','throttle']:
            endpoint[key]=before[key]+ratio*(after[key]-before[key])
        endpoint['brake_mm']=(np.array(before['brake_mm'])+ratio*(np.array(after['brake_mm'])-before['brake_mm'])).tolist()
        frames=[f for f in frames if end-22<=f['time_s']<end]+[endpoint]
    else:
        frames=[f for f in frames if end-22<=f['time_s']<=end]
    assert abs(frames[-1]['time_s']-end)<.051

    replay=dict(frames=frames,target_ned_m=chosen['landing']['target_ned_m'],result=chosen['landing'],
                selected_trial=chosen['trial'],selection='Closest to candidate median miss among successful <=5 m trials',
                source_sha256=hashlib.sha256((directory/'viewer.json').read_bytes()).hexdigest())
    (args.output/'replay.json').write_text(json.dumps(replay,separators=(',',':')))
    print(json.dumps({k:v for k,v in summary.items() if k!='audits'},indent=2))
    print('Selected replay trial',chosen['trial'])

if __name__=='__main__':
    main()
