from pathlib import Path
import base64,json,numpy as np,cairosvg
from html import escape
P=Path(__file__).resolve().parent
D=json.loads((P/'reconstruction-landmarks.json').read_text())
def photo(file,x,y,w,h):
 data=base64.b64encode((P/file).read_bytes()).decode();return f'<image x="{x}" y="{y}" width="{w}" height="{h}" href="data:image/jpeg;base64,{data}"/>'
def text(x,y,t,size=23,bold=False):return f'<text x="{x}" y="{y}" font-size="{size}" font-weight="{"bold" if bold else "normal"}">{escape(t)}</text>'
def line(a,b,c='#0f8ca8',w=3,dash=''):return f'<line x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}" stroke="{c}" stroke-width="{w}" stroke-dasharray="{dash}"/>'
def dot(a,c):return f'<circle cx="{a[0]}" cy="{a[1]}" r="6" fill="{c}" stroke="#142233" stroke-width="1.5"/>'
def start(w,h):return [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}"><rect width="100%" height="100%" fill="#f8fafc"/><g font-family="DejaVu Sans,sans-serif" fill="#142233">']
def save(s,name):
 s.append('</g></svg>');(P/(name+'.svg')).write_text(''.join(s));cairosvg.svg2png(url=str(P/(name+'.svg')),write_to=str(P/(name+'.png')))
s=start(1700,1080)
s += [text(25,44,'Reconstruction anchored to the documented 2.4 m canopy span',30,True),text(25,80,'Working interpretation: 2.4 m is the flat span; trace the inflated mid-chord arch to set scale.',20),photo('hobbyking-v2.jpg',20,110,1130,828)]
arc=np.array(D['arc_points_px'])+[20,110]
s.append('<polyline points="'+' '.join(f'{x},{y}' for x,y in arc)+'" fill="none" stroke="#7dff78" stroke-width="4"/>')
s.append(line(arc[0],arc[-1],'#00eaff',3,'9 6'))
for a in [arc[0],arc[-1]]:s.append(dot(a,'#00eaff'))
lo=np.array(D['stock_holes_px'])+[20,110];up=np.array(D['upper_junctions_px'])+[20,110]
s += [line(lo[0],lo[1],'#ff7979',5),line(up[0],up[1],'#ffe66d',4)]
for a in lo:s.append(dot(a,'#ff7979'))
for a in up:s.append(dot(a,'#ffe66d'))
s.append(line(arc[7],up.mean(0),'#ffe66d',3,'8 6'))
for y,t,b in [(145,'DOCUMENTED',True),(180,'Nominal canopy span: 2.4 m',False),(250,'PHOTO-DERIVED, CONDITIONAL',True),(290,'Green arch trace: 939 px',False),(325,'Scale: 2.56 mm / pixel',False),(375,'Cyan: projected span ≈ 1.98 m',False),(425,'Yellow: upper junctions ≈ 266 mm',False),(475,'Red: stock bar holes ≈ 184 mm',True),(525,'Crown to upper junctions ≈ 1.29 m',False),(560,'Crown to bar midpoint ≈ 1.44 m',False),(640,'MODEL / YOUR CAD',True),(680,'Your riser centers: ~130–132 mm',True),(720,'Model effective canopy span: 647 mm',False),(760,'Model CG–joint offset: 850 mm',False),(825,'Crown is not the canopy CG.',False),(860,'Effective span still needs line loads.',False)]:s.append(text(1170,y,t,21,b))
s += [text(25,980,'Interpretation check: if 2.4 m meant projected span, the same stock holes would be ≈222 mm apart.',21),text(25,1014,'The separate manual kit-photo estimate (~174–184 mm) favors the flat-span interpretation, but does not prove it.',20),text(25,1048,'Original photo preserved. Perspective and manual tracing remain uncertain. Source: HobbyKing V2 product photo and manual.',18)]
save(s,'span-anchored-reconstruction')
# Second figure: exact CAD edges, cached assembly fasteners, and independent manual photo.
s=start(1700,1100);s+=[text(25,44,'Spreader comparison: riser attachments, not overall shroud-support width',29,True),text(25,80,'microAGU: slot-centered straps ~130–132 mm apart  |  HobbyKing: estimated ~175–200 mm',22,True)]
# CAD top-view in its native Y/Z coordinates, 3 px/mm.
edges=np.load(P/'SpreaderBarSpar-edges.npz')['edges'];ox,oy,sc=440,365,3
for pts in edges:
 xy=np.column_stack([ox+pts[:,1]*sc,oy-pts[:,2]*sc]);s.append('<polyline points="'+' '.join(f'{x:.2f},{y:.2f}' for x,y in xy)+'" fill="none" stroke="#456171" stroke-width="0.7"/>')
a=(ox-66*sc,oy);b=(ox+66*sc,oy)
s += [dot(a,'#fb7185'),dot(b,'#fb7185'),line((a[0],470),(b[0],470),'#e14666',3),line(a,(a[0],480),'#e14666',1),line(b,(b[0],480),'#e14666',1),text(290,505,'132 mm CAD slot-center spacing',23,True),text(80,550,'Curved outer extensions support the shroud.',21),text(80,580,'They do not define the load-bearing riser spacing.',21)]
s += [text(40,135,'YOUR SPREADER — direct STEP projection',23,True),text(950,135,'HOBBYKING — manual kit-layout photo',23,True)]
# Original PDF render, clipped via native SVG viewport (no resynthesis).
s.append('<svg x="930" y="155" width="700" height="510" viewBox="210 295 370 260">'+photo('manual-5.png',0,0,893,1263)+'</svg>')
# scaled landmarks in viewport coordinates
conv=lambda x,y:(930+(x-210)*700/370,155+(y-295)*510/260)
# viewport has meet scaling: width scale 1.8919, height actual 491.89 with 9.05 px margin
conv=lambda x,y:(930+(x-210)*700/370,164.054+(y-295)*700/370)
for aa,bb,col in [((256,319),(257,496),'#22c55e'),((379,399),(507,399),'#ee4964')]:
 a,b=conv(*aa),conv(*bb);s += [line(a,b,col,3),dot(a,col),dot(b,col)]
s += [text(950,700,'Propeller: 254 mm / ~177 px',21),text(950,735,'Flybar side spacing: ~128 px → ~184 mm',21),text(950,770,'Ring-based foreshortening check → ~174 mm',20),text(950,805,'Treat as an estimate, not a dimensioned drawing.',20)]
# same-scale widths
s += [text(40,690,'SAME-SCALE COMPARISON',24,True)]
for y,width,col,label in [(750,132,'#c24169','microAGU: ~130–132 mm'),(830,185,'#187b9e','HobbyKing: ~185 mm (working estimate)')]:
 a=(75,y);b=(75+width*3,y);s += [line(a,b,col,8),dot(a,col),dot(b,col),text(75,y+38,label,21,True)]
s += [text(40,960,'Your riser spacing is roughly 30% narrower than stock; your straps ride the bolts at the slot centers.',23,True),text(40,1000,'The simulator uses 148.6 mm: about 13–14% too wide compared with the actual strap centers.',22),text(40,1040,'Sources: saved microAGU assembly transforms; SpreaderBarSpar.STEP; HobbyKing manual printed pp. 3 and 14.',19),text(40,1070,'Slot openings span Y = 60–72 mm and −72 to −60 mm; centers ±66 mm. User confirms straps near ±65 mm.',18)]
save(s,'spreader-comparison')
