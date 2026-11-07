import { createServer } from '/Users/zhendong/Documents/stm32car_simulator/node_modules/vite/dist/node/index.js';
import { writeFileSync } from 'node:fs';
const root = '/Users/zhendong/Documents/stm32car_simulator';
const server = await createServer({root, cacheDir:new URL('./vite-cache',import.meta.url).pathname,server:{middlewareMode:true}, optimizeDeps:{noDiscovery:true,include:[]}, logLevel:'error'});
try {
 const {LineFollowModel}=await server.ssrLoadModule('/src/sim/line_follow.ts');
 const {VehicleSimulator}=await server.ssrLoadModule('/src/sim/physics.ts');
 const {parseRouteTableC,DEFAULT_ROUTE_TABLE_C}=await server.ssrLoadModule('/src/sim/route_parser.ts');
 const {RouteState,LineState}=await server.ssrLoadModule('/src/sim/types.ts');
 const parsed=parseRouteTableC(DEFAULT_ROUTE_TABLE_C);
 const steps=parsed.schemes.find(s=>s.id===1).steps;
 const f=new LineFollowModel(steps),s=new VehicleSimulator(f);
 s.requestStart();let prev='',rows=[];
 for(let i=0;i<12000;i++) {
  s.step(); const r=f.route,o=f.observation;
  const row={ms:s.sim_time_ms,x:s.pose.x,y:s.pose.y,h:s.pose.heading,index:r.index,count:r.junctions_seen,state:RouteState[r.state],raw:s.raw_sensor_mask,mask:o.line_mask,pos:o.position,width:o.white_count,regions:o.region_count,kind:o.kind,reject:r.candidate_reject,smooth:r.smooth_edge_sides,center:r.have_approach_center,center_ms:r.approach_center_ms,center_d:r.approach_center_cm,d:f.odometry.distance_cm,candidate:r.candidate_count,evidence:r.candidate_kind,passing:r.passing_junction,exit:r.exit_count,recoveries:r.navigation_recoveries,warning:r.navigation_warning,lt:f.left_target_cm_s,rt:f.right_target_cm_s,lm:f.odometry.left_speed_cm_s,rm:f.odometry.right_speed_cm_s,pending:f.exit_heading_pending,axis:f.exit_heading_rad,entry_h:r.entry_heading,target:f.route.turnTargetRad(),action:r.currentAction(),axis_origin:f.exit_heading_origin_cm,yaw:f.turn_yaw_rate_rad_s,pyaw:f.odometry.heading_rad};
  rows.push(row);
  const key=[f.state,r.state,r.index,r.junctions_seen].join(',');
  if(prev!==key){console.log(JSON.stringify({...row,line:LineState[f.state]}));prev=key;}
  if(f.state===LineState.LINE_STOPPED) break;
 }
 const keys=Object.keys(rows[0]);writeFileSync(new URL(process.argv.includes('--after') ? './after-trace.csv' : './baseline-trace.csv',import.meta.url),keys.join(',')+'\n'+rows.map(r=>keys.map(k=>r[k]).join(',')).join('\n')+'\n');
}finally{await server.close();}
