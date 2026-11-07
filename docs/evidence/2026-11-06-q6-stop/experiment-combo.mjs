import {createServer} from '/Users/zhendong/Documents/stm32car_simulator/node_modules/vite/dist/node/index.js';
const root='/Users/zhendong/Documents/stm32car_simulator';
const server=await createServer({root,cacheDir:new URL('./combo-cache',import.meta.url).pathname,server:{middlewareMode:true},optimizeDeps:{noDiscovery:true,include:[]},logLevel:'error',plugins:[{name:'combo',enforce:'pre',transform(code,id){
 if(id.endsWith('/sim/route.ts')){
  const a='const history = (this.state === RouteState.ROUTE_CANDIDATE && d >= this.entry_cm) || recent_center;';
  if(!code.includes(a)) throw Error('route history');
  code=code.replace(a,'const history = (this.state === RouteState.ROUTE_CANDIDATE && d >= this.entry_cm) || recent_center || (o.white_count === 4 && (o.line_mask & 0x18) !== 0);');
 }
 if(id.endsWith('/sim/line_follow.ts')){
  const a='this.exit_heading_rad = previous_exit_axis;';
  if(!code.includes(a)) throw Error('axis');
  code=code.replace(a,'this.exit_heading_rad = Math.abs(previous_exit_axis - Math.round(previous_exit_axis / (Math.PI / 2)) * (Math.PI / 2)) <= 0.5 ? Math.round(previous_exit_axis / (Math.PI / 2)) * (Math.PI / 2) : previous_exit_axis;');
 }
 return code;
}}]});
try{const {LineFollowModel}=await server.ssrLoadModule('/src/sim/line_follow.ts');const {VehicleSimulator}=await server.ssrLoadModule('/src/sim/physics.ts');const {parseRouteTableC,DEFAULT_ROUTE_TABLE_C}=await server.ssrLoadModule('/src/sim/route_parser.ts');const {RouteState:R,LineState:L}=await server.ssrLoadModule('/src/sim/types.ts');const f=new LineFollowModel(parseRouteTableC(DEFAULT_ROUTE_TABLE_C).activeSteps),s=new VehicleSimulator(f);s.requestStart();let last='';for(let i=0;i<15000;i++){s.step();const r=f.route,k=[r.state,r.index,r.junctions_seen].join(',');if(k!==last){console.log(JSON.stringify({ms:s.sim_time_ms,state:R[r.state],index:r.index,count:r.junctions_seen,x:s.pose.x,y:s.pose.y,h:s.pose.heading,mask:f.observation.line_mask,pos:f.observation.position,width:f.observation.white_count,warning:r.navigation_warning}));last=k;}if(f.state===L.LINE_STOPPED||r.index>=9)break;}}finally{await server.close();}
