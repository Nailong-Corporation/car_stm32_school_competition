import {createServer} from '/Users/zhendong/Documents/stm32car_simulator/node_modules/vite/dist/node/index.js';
const root='/Users/zhendong/Documents/stm32car_simulator';
const server=await createServer({root,cacheDir:new URL('./history-cache',import.meta.url).pathname,server:{middlewareMode:true},optimizeDeps:{noDiscovery:true,include:[]},logLevel:'error',plugins:[{name:'history',enforce:'pre',transform(code,id){
 if(!id.endsWith('/sim/route.ts'))return;
 const before='const history = (this.state === RouteState.ROUTE_CANDIDATE && d >= this.entry_cm) || recent_center;';
 if(!code.includes(before)) throw new Error('history pattern missing');
 return code.replace(before,'const history = (this.state === RouteState.ROUTE_CANDIDATE && d >= this.entry_cm) || recent_center || (o.white_count >= 4 && (o.line_mask & 0x18) !== 0);');
}}]});
try{
 const {LineFollowModel}=await server.ssrLoadModule('/src/sim/line_follow.ts');
 const {VehicleSimulator}=await server.ssrLoadModule('/src/sim/physics.ts');
 const {parseRouteTableC,DEFAULT_ROUTE_TABLE_C}=await server.ssrLoadModule('/src/sim/route_parser.ts');
 const {RouteState:R,LineState:L}=await server.ssrLoadModule('/src/sim/types.ts');
 const f=new LineFollowModel(parseRouteTableC(DEFAULT_ROUTE_TABLE_C).activeSteps),sim=new VehicleSimulator(f);
 sim.requestStart();let last='';
 for(let i=0;i<12000;i++){sim.step();const r=f.route;const key=[r.state,r.index,r.junctions_seen].join(',');
  if(key!==last){console.log(JSON.stringify({ms:sim.sim_time_ms,state:R[r.state],index:r.index,count:r.junctions_seen,x:sim.pose.x,y:sim.pose.y,h:sim.pose.heading,warning:r.navigation_warning}));last=key;}
  if(f.state===L.LINE_STOPPED||r.index>=8)break;
 }
}finally{await server.close();}
