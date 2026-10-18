from run import *
import shutil,contextlib,io
sys.path.insert(0,str(HERE.parent/'scheduled-state-feedback-2026-10-15'))
import experiment as ex
ORIGINAL=ex.HERE
sys.path.insert(0,str(HERE.parents[1]/'research/archive/viewer-studies'))
def task(job):
 case,variant,resolution=job
 dest=HERE/'traces'/f"closed-{case['name']}-{variant}-{resolution}";dest.mkdir(parents=True,exist_ok=True)
 for n in ['grid.npz','gains.npz']:shutil.copyfile(ORIGINAL/n,dest/n)
 ex.HERE=dest;diags=[]
 def factory(flight,profile):
  s,_=make(flight,variant,profile);step=s.step
  def advance(state,rpm,command,voltage=22.2,wind=(0,0,0),dt=.04):
   s.substeps=resolution
   diags.append(dynamics.tension_diagnostic(s,state,rpm,command,voltage,wind))
   return step(state,rpm,command,voltage,wind,dt)
  s.step=advance
  return s
 ex.setup=lambda _:factory
 with contextlib.redirect_stdout(io.StringIO()):row=ex.run_case(case,'baseline')
 d=np.array(diags)
 row.update(variant=variant,substeps=resolution,load_quantiles=np.quantile(d[:,0]/d[:,1],[0,.05,.5,.95,1]).tolist(),clipped_fraction=float(np.mean((d[:,2]<=.25)|(d[:,2]>=2))),closure_max=float(max(abs(d[:,6]))),constraint_max=float(max(d[:,7])),denominator_min=float(min(d[:,4])))
 np.savez_compressed(dest/'load.npz',diag=d)
 return row
if __name__=='__main__':
 rows=[]
 jobs=[(c,v,16) for c in ex.scenarios() for v in VARIANTS]
 with concurrent.futures.ProcessPoolExecutor(max_workers=4) as pool:
  for row in pool.map(task,jobs):
   rows.append(row);print(len(rows),row['case'],row['variant'],row['height_rms'],row['heading_rms'],flush=True)
   (HERE/'closed-results.json').write_text(json.dumps(rows,indent=2)+'\n')
