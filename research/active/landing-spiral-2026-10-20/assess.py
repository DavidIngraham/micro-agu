import json,math,tarfile,hashlib
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
rows=[]
for name in ["calm","crosswind"]:
 d=json.loads((HERE/"runs"/(name+"-result.json")).read_text())
 assert d["failure"] is None and d["finalLanding"]["phase"]=="landed"
 samples=d["samples"]
 assert set(x["mode"] for x in samples)=={10}
 spiral=[x for x in samples if x["telemetry"]["mission"]==2]
 approach=[x for x in samples if x["telemetry"]["mission"]==3]
 angles=[math.atan2(x["state"][1]-40,x["state"][0]+280) for x in spiral]
 rotation=sum(math.atan2(math.sin(b-a),math.cos(b-a)) for a,b in zip(angles,angles[1:]))
 exit=spiral[-1]; final=approach[-1]; contact=d["contact"]["incoming"]
 row={"case":name,"spiral_start_alt_m":spiral[0]["telemetry"]["alt"],"spiral_exit_alt_m":exit["telemetry"]["alt"],
 "descending_orbit_deg":math.degrees(rotation),"exit_course_deg":math.degrees(exit["telemetry"]["course"]),
 "final_course_deg":math.degrees(final["telemetry"]["course"]),"final_cross_track_m":final["state"][1],
 "sink_at_contact_m_s":contact["state"][9],"spot_error_m":math.hypot(*contact["state"][:2])}
 assert row["spiral_start_alt_m"]>55 and abs(row["spiral_exit_alt_m"]-30)<2,row
 assert row["descending_orbit_deg"]>180,row
 assert abs(row["exit_course_deg"])<12 and abs(row["final_course_deg"])<3,row
 assert abs(row["final_cross_track_m"])<2,row
 assert row["sink_at_contact_m_s"]<1.5,row
 rows.append(row)
(HERE/"summary.json").write_text(json.dumps(rows,indent=2)+"\n")
with tarfile.open(HERE/"evidence.tar.gz","w:gz") as archive:
 for name in ["calm","crosswind"]:
  path=HERE/"runs"/(name+"-result.json");archive.add(path,arcname=path.name)
provenance={"files":{f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in ["viewer/apps/simulator/missions/default.ts","viewer/apps/simulator/missions/preview.ts","viewer/generated/ardupilot/arduplane.wasm","aircraft/microagu/configurations/browser-physics.json","aircraft/microagu/controllers/lqi-gains.bin"]},"evidence_sha256":hashlib.sha256((HERE/"evidence.tar.gz").read_bytes()).hexdigest()}
(HERE/"provenance.json").write_text(json.dumps(provenance,indent=2)+"\n")
print(json.dumps(rows,indent=2))
