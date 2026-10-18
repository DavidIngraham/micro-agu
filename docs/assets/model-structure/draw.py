"""Generate editable vector diagrams for the controller-refinement article."""
from pathlib import Path
from html import escape
ROOT=Path(__file__).parent
def start(title,desc,height):
 return [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1200 {height}" role="img" aria-labelledby="title desc"><title id="title">{escape(title)}</title><desc id="desc">{escape(desc)}</desc>',
 '<defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto-start-reverse"><path d="M0 0L8 4L0 8Z" fill="context-stroke"/></marker></defs>',
 '<style>text{font-family:Arial,sans-serif;fill:#213c4a;font-size:19px}.title{font-size:29px;font-weight:bold}.head{font-size:20px;font-weight:bold}.small{font-size:17px}.box{fill:#fff;stroke:#9bb2bd;stroke-width:2}.link{fill:none;stroke:#536f80;stroke-width:2.5;marker-end:url(#arrow)}.force{fill:none;stroke:#c56a2e;stroke-width:3;marker-end:url(#arrow)}.joint{fill:none;stroke:#237f78;stroke-width:3;marker-end:url(#arrow)}.ghost{fill:none;stroke:#9bb2bd;stroke-width:2;stroke-dasharray:6 5}</style>',
 f'<rect width="1200" height="{height}" fill="#f5f8fa"/>']
def text(s,x,y,lines,cls='',gap=27):
 for i,line in enumerate(lines):s.append(f'<text x="{x}" y="{y+i*gap}" class="{cls}">{escape(line)}</text>')
def box(s,x,y,w,h,title,lines):
 s.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="10" class="box"/>');text(s,x+18,y+32,[title],'head');text(s,x+18,y+62,lines,'small',24)
def path(s,d,cls='link'):s.append(f'<path d="{d}" class="{cls}"/>')
def circle(s,x,y,r=6):s.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="#213c4a"/>')
def save(s,name):ROOT.joinpath(name).write_text('\n'.join(s+['</svg>'])+'\n')

s=start('How one simulation step is assembled','Commands become brake travel and rotor loads. Aerodynamic, gravity and joint loads enter a constrained two-body solve. Integration updates both bodies; payload measurements close the controller loop.',850)
text(s,32,45,['One aircraft model, two ways to drive it'],'title')
text(s,32,79,['Recorded commands for replay, or live commands from a controller / native ArduPilot SITL.'])
box(s,30,125,290,120,'1 · Commands & actuators',['Throttle + left/right PWM or pull','Asymmetric mapping, limits, rate','Slack / preload → effective pull'])
box(s,410,125,350,125,'2a · Canopy aerodynamics',['Five regions on one rigid canopy','Local airflow + rotation + brakes','Sum panel forces and moments'])
box(s,410,280,350,125,'2b · Payload & propulsion',['Payload drag + thrust at motor','Voltage → RPM lag → prop map','Shaft torque + rotor momentum'])
box(s,410,435,350,105,'2c · Coupling & gravity',['Weights of both bodies','Relative-rotation spring/damper'])
path(s,'M320 175H410');path(s,'M350 175V330H410')
text(s,30,325,['Environment / configuration'],'head');text(s,30,358,['Wind → relative airflow','Voltage → motor speed','Mass, inertia, geometry','Mission / softer-roll profile'])
path(s,'M300 385H370V220H410');path(s,'M370 385H410');path(s,'M300 420H350V490H410')
box(s,840,235,330,240,'3 · Coupled acceleration solve',['Payload inertia','Canopy + air added inertia','Point-joint reaction force J','Equal attachment acceleration','→ both bodies’ linear and','   angular accelerations'])
path(s,'M760 188H800V280H840');path(s,'M760 343H840');path(s,'M760 488H800V430H840')
box(s,840,565,330,145,'4 · Integrate & retain states',['RK4 substeps; normalize attitudes','Position, velocity, quaternion,','angular velocity for each body','Plus rotor RPM and brake memory'])
path(s,'M1005 475V565')
box(s,410,595,350,115,'5 · Payload observations',['IMU location and body frame','Attitude / rates / navigation','JSON sensor interface in SITL'])
path(s,'M840 650H760')
box(s,30,595,290,115,'Controller or log analysis',['Standalone feedback / analysis','or ArduPilot estimator,','navigation and control loops'])
path(s,'M410 650H320');path(s,'M150 595V465H15V185H30')
path(s,'M1005 710V775H785V110H580V125');path(s,'M785 375H760');path(s,'M785 515H760')
text(s,345,804,['Updated motion feeds the next airflow, joint and rotor-load calculation.'],'small')
save(s,'simulation-blocks.svg')

s=start('Modeled aircraft and free-body diagrams','A rigid canopy and rigid payload share one equivalent point attachment. Separate body diagrams show external forces, opposite joint reactions and opposite restoring torques. Added air inertia belongs to the canopy mass matrix.',900)
text(s,32,45,['Two bodies, one equivalent attachment'],'title')
text(s,32,78,['Schematic only · force directions are illustrative · moments and offsets are three-dimensional'])
s.append('<path d="M590 110V805" stroke="#c4d1d8" stroke-width="2"/>')
text(s,35,130,['Modeled system'],'head')
s.append('<path d="M90 270Q280 95 485 260L465 290Q280 180 110 300Z" fill="#b9dfd4" stroke="#237f78" stroke-width="3"/>')
for x,y in [(150,255),(225,225),(310,225),(395,247)]:path(s,f'M{x} {y}L280 490','ghost')
circle(s,280,230);text(s,330,190,['Rigid canopy','5 aerodynamic panels'],'small')
circle(s,280,490,9);text(s,45,445,['Equivalent point joint'],'head');text(s,45,473,['Positions coincide; rotations may differ'],'small')
path(s,'M280 490V560','ghost')
s.append('<rect x="222" y="560" width="115" height="105" rx="15" fill="#d2e3ef" stroke="#426e89" stroke-width="3"/>')
circle(s,280,612);text(s,350,595,['Rigid payload','Separate attitude','and angular rate'],'small')
s.append('<path d="M222 595H195M195 562V630" stroke="#426e89" stroke-width="5"/>');path(s,'M195 595H125','ghost')
text(s,45,685,['Motor / propeller offset from payload CG'],'small')
text(s,35,745,['Lines drawn here explain the geometry.','The solver uses one point constraint plus','an effective rotational spring and damper,','not individually flexible suspension lines.'],'small',25)

text(s,625,130,['Canopy free body'],'head')
s.append('<path d="M720 230Q810 170 910 230" fill="none" stroke="#237f78" stroke-width="12"/>');circle(s,815,215)
path(s,'M815 215V155','force');text(s,840,171,['Aerodynamic lift'],'small')
path(s,'M815 215H655','force');text(s,630,198,['Drag'],'small')
path(s,'M850 220V290','force');text(s,875,275,['Canopy','weight'],'small',22)
path(s,'M815 220V340','ghost');circle(s,815,340,7)
path(s,'M815 340L785 405','joint');text(s,830,393,['−J at attachment'],'small')
path(s,'M920 220C970 245 960 305 915 310','force');text(s,985,245,['Aero moment'],'small')
text(s,980,280,['Includes panel','moment arms'],'small')
path(s,'M735 310C710 345 735 375 765 360','joint');text(s,630,339,['−Mⱼ'],'small')
text(s,625,438,['Added air inertia enters the canopy mass matrix;','it is not an extra applied lift or weight arrow.'],'small',24)

text(s,625,505,['Payload free body'],'head')
circle(s,815,540,7);path(s,'M815 540L845 475','joint');text(s,860,520,['+J at attachment'],'small')
path(s,'M815 540V625','ghost')
s.append('<rect x="765" y="585" width="100" height="80" rx="12" fill="#d2e3ef" stroke="#426e89" stroke-width="3"/>');circle(s,815,625)
path(s,'M815 625V740','force');text(s,835,735,['Payload weight'],'small')
path(s,'M815 625H655','force');text(s,630,590,['Payload drag'],'small')
path(s,'M780 600H1010','force');text(s,950,580,['Thrust at motor'],'small')
path(s,'M765 620C735 635 710 605 735 570','joint');text(s,630,560,['+Mⱼ'],'small')
text(s,930,650,['Motor offset → pitch moment','Shaft reaction torque','Gyroscopic + spin-up reaction'],'small',25)
text(s,625,793,['J: joint reaction on payload · Mⱼ: restoring/damping torque'],'small')
text(s,32,853,['Internal joint forces and torques cancel for the combined aircraft; their CG moment arms still matter.'],'small')
save(s,'modeled-system-fbd.svg')
