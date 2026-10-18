"""Napkin mass model: cached CAD anchors, explicit approximate centroids, measured AUW closure."""
from pathlib import Path
import json,numpy as np,xml.etree.ElementTree as ET
O=Path(__file__).parent;CAD=Path('/mnt/c/Users/davin/OneDrive/Documentos/Personal Projects/microagu/analysis/cad');tree=ET.parse(CAD/'microAGU/swXmlContents_COMPINSTANCETREE');ns={'s':'http://www.solidworks.com/sw2003/schema'};models={e.attrib['id']:e for e in tree.findall('.//s:swModel',ns)};anchors=[]
def walk(mid,T=np.eye(4),path=''):
 for e in models[mid].findall('s:swReference',ns):
  if e.get('swSuppressed')=='YES':continue
  A=np.array(list(map(float,e.get('swTransform').split()))).reshape(4,4).T;G=T@A;name=e.get('swName');anchors.append(dict(path=path+'/'+name,origin_m=G[:3,3].tolist(),note='Component origin, not center of mass'));child=e.get('swModelRef')
  if child in models:walk(child,G,path+'/'+name)
walk('2');(O/'napkin-cad-anchors.json').write_text(json.dumps(anchors,indent=2))
# CAD frame: +x toward front assumed from motor aft of core, +z up; body FRD=[x,-y,-z].
B=np.diag([1,-1,-1]);cases=[]
for label,canopy,avionics,wire,upper_fraction,lead_z in [('nominal',.190,.030,.050,.55,-.008),('low_cg',.140,.022,.040,.35,-.020),('high_cg',.240,.030,.070,.75,.005)]:
 rows=[]
 def add(name,m,c,size):rows.append(dict(name=name,mass_kg=m,center_cad_m=list(c),box_dimensions_cad_m=list(size)))
 add('Battery left',.310,[.044,-.0195,.044],[.055,.038,.070]);add('Battery right',.310,[.044,.0195,.044],[.055,.038,.070])
 add('FFTY motor (box approximation)',.112,[-.017,0,.125],[.034,.0375,.0375]);add('Servo left',.032,[.049,-.025,.100],[.033,.015,.0345]);add('Servo right',.032,[.049,.025,.100],[.033,.015,.0345])
 add('GPS',.008,[.069,0,.1366],[.020,.020,.0124]);add('Avionics incl USB allowance',avionics,[.044,0,.110],[.054,.036,.013]);add('ESC provisional',.0073,[.0455,-.018,.098],[.021,.035,.007]);add('9x4.5 prop allowance',.012,[-.04,0,.125],[.008,.229,.015]);add('Wires connectors receiver hardware allowance',wire,[.044,0,.100],[.08,.07,.07])
 residual=1.385-canopy-sum(r['mass_kg'] for r in rows);assert residual>0
 add('Unweighed upper structure allocation',residual*upper_fraction,[.038,0,.126],[.10,.11,.050]);add('Unweighed lower structure allocation',residual*(1-upper_fraction),[.044,0,.038],[.09,.08,.076])
 for ballast in [False,True]:
  rr=list(rows)
  if ballast:rr=rr+[dict(name='Lead',mass_kg=.373,center_cad_m=[.044,0,lead_z],box_dimensions_cad_m=[.060,.050,.011])]
  mass=sum(r['mass_kg'] for r in rr);cg=sum(r['mass_kg']*np.array(r['center_cad_m']) for r in rr)/mass;I=np.zeros((3,3))
  for r in rr:
   m=r['mass_kg'];size=np.array(r['box_dimensions_cad_m']);d=np.array(r['center_cad_m'])-cg;I+=np.diag(m/12*(np.sum(size**2)-size**2))+m*((d@d)*np.eye(3)-np.outer(d,d))
  anchors_cad={'hinge':[.044,0,.150],'thrust':[0,0,.125],'imu':[.0375,0,.110]};arms={k:(B@(np.array(v)-cg)).tolist() for k,v in anchors_cad.items()};Ib=B@I@B.T
  assert abs(mass+canopy-(1.758 if ballast else 1.385))<1e-9;assert np.linalg.eigvalsh(I).min()>0
  cases.append(dict(case=label,with_lead=ballast,canopy_mass_kg=canopy,payload_mass_kg=mass,AUW_kg=mass+canopy,CG_cad_m=cg.tolist(),inertia_body_kg_m2=Ib.tolist(),arms_body_m=arms,unweighed_structure_mass_kg=residual,components=rr))
result=dict(scope='Approximate model, not native SolidWorks edit. AUW constrained; correlated sensitivity scenarios, not confidence intervals.',frame='CAD+x forward assumed, CAD+z up; body FRD=[CADx,-CADy,-CADz]. Datum CAD assembly origin.',placement_notes=['Cached component origins are not centroids. Box centroids use CAD anchors and build-photo layout.','Cached second servo/prop placements appear displaced; use symmetric installed servos and prop on motor shaft rather than literal cached origins.','Batteries approximated side-by-side in bottom box; dimensions from source pack size, centroid allowance.','Hinge at CADz=.150 is a top-spreader approximation, not extracted attachment hole.','Residual mass covers unweighed printed structure; mass closure does not validate spatial allocation.','Canopy mass range140–240g assumed; not inferred from logs.','Lead center -20 to+5mm CADz assumed at payload bottom.','Box inertia approximates component mass distribution; shroud distributed inertia not separately resolved.'],cases=cases)
(O/'napkin-payload-model.json').write_text(json.dumps(result,indent=2));print(json.dumps([{k:r[k] for k in ['case','with_lead','CG_cad_m','inertia_body_kg_m2','arms_body_m']} for r in cases],indent=2))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(1,2,figsize=(11,5))
for ax,lead in zip(axes,[False,True]):
 r=next(r for r in cases if r['case']=='nominal' and r['with_lead']==lead)
 for c in r['components']:
  x,y,z=c['center_cad_m'];ax.scatter(x*1000,z*1000,s=c['mass_kg']*900,label=c['name'])
 for q in cases:
  if q['with_lead']==lead:ax.plot(q['CG_cad_m'][0]*1000,q['CG_cad_m'][2]*1000,'kx')
 ax.plot(44,150,'r^',label='Assumed suspension');ax.axhline(125,color='gray',ls='--',label='Thrust line');ax.set(xlabel='CAD x (mm)',ylabel='CAD z (mm)',title='With lead' if lead else 'Without lead');ax.grid(alpha=.2)
axes[1].legend(fontsize=6,loc='center left',bbox_to_anchor=(1,0.5));fig.suptitle('Napkin mass distribution — marker area scales with mass; crosses are CG scenarios');fig.tight_layout();fig.savefig(O/'napkin-payload-model.png',dpi=150)
