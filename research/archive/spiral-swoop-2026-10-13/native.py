"""Native AUTO descending spiral followed by AP_Landing flare.

Research mission only: no firmware changes or replacement actuator commands.
PG_LANDING_CONFIG selects geometry, flare and weather. Raw results use BUILDLOGS.
"""
import sys,json,math,runpy
from pathlib import Path
import numpy as np
sys.path.insert(0,'/home/david/repos/ardupilot-high-dof/Tools/simulation/paraglider')
import landing_autotest as landing
from pymavlink import mavutil
harness=landing.harness
config=landing.CONFIG


def experiment(self):
    harness.airborne(self,altitude=config.get('entry_alt_m',35))
    backend=self._pg_backend[0]
    self.set_parameters({
        'PG_FLARE_PCT':config.get('flare_pct',75),
        'PG_FLARE_RATE':config.get('flare_rate',50),
        'PG_FLARE_RES':config.get('flare_reserve',10),
        'LAND_FLARE_ALT':config.get('flare_alt_m',4),
        'LAND_FLARE_SEC':0,'LAND_DISARMDELAY':0,'TECS_LAND_THR':-1,
        'LAND_FLARE_AIM':config.get('flare_aim_pct',50),
        'TECS_LAND_SINK':config.get('flare_sink_m_s',1.4),
        'WP_RADIUS':config.get('wp_radius_m',5),'WP_MAX_RADIUS':0,
        'PG_TURN_RMAX':config.get('turn_rate_limit_dps',25),
        'NAVL1_PERIOD':config.get('turn_l1_period',10),
    })
    north,east=backend.state[:2]
    east-=config.get('cross_track_error_m',0)
    radius=config.get('radius_m',7)
    if config.get('route_type')=='loiter_to_alt':
        corner=np.array([north+12,east+radius])
        target=np.r_[corner+[radius,config.get('final_distance_m',38)],0.]
        route=[(mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,*corner,config['exit_alt_m']),
               (mavutil.mavlink.MAV_CMD_NAV_LAND,*target[:2],0.)]
        mission=self.create_simple_relhome_mission(route)
        mission[1].param2=radius
        mission[1].param4=1 # Native tangent-exit track, not track from orbit center.
    else:
        turn_angle=config.get('spiral_degrees',450)
        count=int(turn_angle/45)
        route=[(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,north+6,east,config['entry_alt_m'])]
        origin=np.array([north+6,east])
        for i,angle in enumerate(np.linspace(45,turn_angle,count)):
            theta=np.deg2rad(angle)
            point=origin+radius*np.array([np.sin(theta),1-np.cos(theta)])
            altitude=config['entry_alt_m']+(config['exit_alt_m']-config['entry_alt_m'])*(i+1)/count
            route.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,*point,altitude))
        theta=np.deg2rad(turn_angle)
        corner=point
        target=np.r_[point+config.get('final_distance_m',30)*np.array([np.cos(theta),np.sin(theta)]),0.]
        route.append((mavutil.mavlink.MAV_CMD_NAV_LAND,*target[:2],0.))
        mission=self.create_simple_relhome_mission(route)
    self.check_mission_upload_download(mission)
    self.change_mode('AUTO')
    start=backend.time;samples=[];guidance_restored=False;events=[]
    while backend.time-start<140:
        msg=self.assert_receive_message('GLOBAL_POSITION_INT')
        samples.append([backend.time,-backend.state[2],*backend.state[7:10],*backend.command])
        # Optional terminal schedule uses native estimator heading, never truth feedback.
        current=self.mav.messages.get('MISSION_CURRENT')
        if current is not None and current.seq>=len(mission)-1 and not guidance_restored:
            self.set_parameter('NAVL1_PERIOD',config.get('final_l1_period',10))
            self.set_parameter('PG_TURN_RMAX',25)
            guidance_restored=True
            events.append(dict(time=backend.time,event='Final approach guidance',sequence=current.seq))
        if backend.contact is not None:break
        if not math.isfinite(msg.relative_alt):raise RuntimeError('Nonfinite altitude')
    summary=dict(case=config.get('name','development'),contact=backend.contact,config=config,
                 duration=backend.time-start,target_ned_m=target.tolist(),corner_ned_m=np.r_[corner,-config.get('exit_alt_m',15)].tolist(),
                 mission_land_lat_lon=[mission[-1].x*1e-7,mission[-1].y*1e-7],
                 route=route,events=events,controller='Native AUTO / L1 / PG turn controller / parafoil TECS / AP_Landing scheduled flare')
    if backend.contact:
        error=np.array(backend.contact['position'])-target
        summary.update(radial_error_m=float(np.linalg.norm(error[:2])),cross_track_error_m=float(error[0]),along_track_error_m=float(error[1]))
    Path(self.buildlogs_path('landing.json')).write_text(json.dumps(summary,indent=2))
    Path(self.buildlogs_path('viewer.json')).write_text(json.dumps(backend.viewer_frames))
    np.savetxt(self.buildlogs_path('landing.csv'),samples,delimiter=',')
    if backend.contact is None:raise RuntimeError('No ground-plane crossing')
    self.disarm_vehicle(force=True)

harness.original=experiment
harness.mission.__name__='ParagliderSpiralStudy'
harness.mission.__doc__=__doc__
if __name__=='__main__':runpy.run_path(str(harness.ROOT/'Tools/autotest/autotest.py'),run_name='__main__')