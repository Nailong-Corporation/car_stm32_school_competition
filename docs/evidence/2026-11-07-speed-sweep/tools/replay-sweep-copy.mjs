import { createServer } from 'vite';
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
const scheme = Number(process.argv[2] ?? 1);
const out = process.argv[3] ?? '/tmp/stm32-replay';
mkdirSync(out, {recursive:true});
const server = await createServer({server:{middlewareMode:true},logLevel:'error',optimizeDeps:{noDiscovery:true,include:[]}});
try {
 const {LineFollowModel}=await server.ssrLoadModule('/src/sim/line_follow.ts');
 const {VehicleSimulator}=await server.ssrLoadModule('/src/sim/physics.ts');
 const {parseRouteTableC}=await server.ssrLoadModule('/src/sim/route_parser.ts');
 const {LineState,RouteState,RouteFailure}=await server.ssrLoadModule('/src/sim/types.ts');
 const tablePath=process.env.SIM_ROUTE_TABLE ?? '/Users/zhendong/Documents/PlatformIO/Projects/car_stm32/src/route_table.c';
 const source=readFileSync(tablePath,'utf8');
 const steps=parseRouteTableC(source).schemes.find(s=>s.id===scheme).steps;
 const f=new LineFollowModel(steps),s=new VehicleSimulator(f);
 // One complete lap, so reaching the configured final STOP is observable.
 f.route.repeat_count=1;
 s.requestStart(); let prev='',rows=[],events=[];
 for(let i=0;i<240000;i++) {
  s.step();const r=f.route,o=f.observation;
  const row={ms:s.sim_time_ms,index:r.index,state:RouteState[r.state],count:r.junctions_seen,x:s.pose.x,y:s.pose.y,h:s.pose.heading,d:f.odometry.distance_cm,mask:o.line_mask,pos:o.position,width:o.white_count,regions:o.region_count,center:r.approach_center_count,reject:r.candidate_reject,smooth:r.smooth_edge_sides,lt:f.left_target_cm_s,rt:f.right_target_cm_s,lv:s.actual_left_speed,rv:s.actual_right_speed,lm:f.odometry.left_speed_cm_s,rm:f.odometry.right_speed_cm_s,lp:f.command.left_pwm,rp:f.command.right_pwm,entry:r.entry_heading,actionStart:r.action_start_heading,phase:r.phase_ms,recoveries:r.navigation_recoveries,shortTracking:r.short_link_tracking,shortActive:r.short_link_active,shortForward:r.short_link_forward_cm,shortLateral:r.short_link_lateral_cm,rewind:r.turn_rewind_active,warning:r.navigation_warning,failure:r.failure,line:LineState[f.state],pose:r.pose_mode,abn:f.abnormal,ccross:f.component_cross_latched,lrel:f.last_reliable_ms,pcr:r.pose_cross_cm,pcrb:r.pose_cross_bar_cm,pgap:r.pose_gap_bar_cm,epnd:f.exit_heading_pending,crossl:f.cross_track_lost?1:0,rcov:r.component_recovery_active,poss:f.pose,ox:f.odometry.x_cm,oy:f.odometry.y_cm,oh:f.odometry.heading_rad,wp:`${f.route.waypoint_x_cm[r.index]?.toFixed(1)},${f.route.waypoint_y_cm[r.index]?.toFixed(1)}`,wpp:`${f.route.waypoint_x_cm[r.index-1]?.toFixed(1)},${f.route.waypoint_y_cm[r.index-1]?.toFixed(1)}`,seg:f.route.pose_seg_heading_rad,bear:f.route.pose_bearing_rad,tlate:f.route.pose_trigger_late_cm,ex:(f.route.start_x_cm+f.odometry.x_cm*Math.cos(f.route.start_heading_rad)-f.odometry.y_cm*Math.sin(f.route.start_heading_rad)),ey:(f.route.start_y_cm+f.odometry.x_cm*Math.sin(f.route.start_heading_rad)+f.odometry.y_cm*Math.cos(f.route.start_heading_rad))};
  rows.push(row);
  const k=[r.index,r.state,r.junctions_seen].join(',');
  if(k!==prev){events.push(row);console.log(JSON.stringify(row));prev=k;}
  if(f.state===LineState.LINE_STOPPED || r.state===RouteState.ROUTE_FINISHED)break;
 }
 const keys=Object.keys(rows[0]);writeFileSync(`${out}/scheme${scheme}.csv`,keys.join(',')+'\n'+rows.map(r=>keys.map(k=>r[k]).join(',')).join('\n'));
 writeFileSync(`${out}/scheme${scheme}.json`,JSON.stringify({steps,events,final:rows.at(-1)},null,2));
} finally{await server.close();}
