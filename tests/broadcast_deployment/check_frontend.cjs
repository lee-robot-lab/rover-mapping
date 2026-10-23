// ================================================================
// check_frontend.cjs
// 설명: 실기 화면에서 조작 버튼 숨김·실제 속도 기반 평활화·Canvas 실행을 확인한다.
// 사용법: node tests/broadcast_deployment/check_frontend.cjs
// ================================================================
const fs=require('fs'),vm=require('vm'),nodes=new Map();
const context=new Proxy({},{get:()=>()=>{}});
function node(id){if(!nodes.has(id))nodes.set(id,{textContent:'',style:{},classList:{toggle(){}},clientWidth:1000,clientHeight:500,getContext:()=>context});return nodes.get(id)}
const meta={live:true,simulation_mode:process.argv.includes('--simulation'),phase:'transfer',extent:[0,0,4,8.12],start:[2.659,5.573,-1.33],cell:.04};
const state={pose:{x:2.659,y:5.573,yaw:-1.33,v:.1,w:0,speed_m_s:.12,seq:1},prediction:null,
  run:1,active:true,stale:false,phase:'transfer',elapsed_s:1,age_s:.1,accepted:1,rejected:0,bad_packets:0,
  tracks:[[],[]],distances:[0,0],explored:[[],[]]};
const sandbox={console,document:{getElementById:node,querySelector:()=>node('controls')},devicePixelRatio:1,
  Image:class{constructor(){this.complete=true;this.naturalWidth=100;this.width=100;this.height=200}},
  window:{addEventListener(){}},setInterval(){},fetch:async url=>({ok:true,json:async()=>url==='/api/meta'?meta:state})};
vm.createContext(sandbox);const html=fs.readFileSync('experiments/broadcast_demo/index.html','utf8');
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],sandbox);
setTimeout(()=>{
 if(node('controls').style.display!=='none')throw Error('live controls exposed');
 if((node('run-summary').style.display==='none')===meta.simulation_mode)throw Error('distance summary visibility does not match real/simulation mode');
 for(const id of ['prediction-legend','teleop-legend'])if((node(id).style.display==='none')===meta.simulation_mode)throw Error('real display legend visible: '+id);
 if((node('display-footer').style.display==='none')===meta.simulation_mode)throw Error('footer visibility does not match real/simulation mode');
 if(node('speed').textContent!=='0.120')throw Error('display must use measured speed rather than command v');
 state.pose.seq=2;state.pose.speed_m_s=.14;vm.runInContext('update()',sandbox);
 if(node('speed').textContent!=='0.127')throw Error('speed smoothing failed');
 if(state.pose.v!==.1||state.pose.speed_m_s!==.14)throw Error('smoothing altered input');
 state.active=false;state.operator={active:false};state.teleop_tracks=[[[3.6,1.7,0,0],[3.6,1.6,.1,1]],[[3.5,1.5,0,2]]];state.teleop_distance_m=.1;state.prediction={source:'teleop_model',slip:.1,costs:[1],candidates:[[[3.6,1.7],[3.6,1.6]]],selected:[[3.6,1.7],[3.6,1.6]]};vm.runInContext('update()',sandbox);
 if(node('run').textContent!=='TELEOP'||node('prediction').textContent!=='TELEOP MODEL')throw Error('manual prediction label missing');
 if((node('run').style.display==='none')===meta.simulation_mode||(node('prediction').style.display==='none')===meta.simulation_mode)throw Error('real TELEOP labels visible');
 if(node('teleopdistance').textContent!=='0.10 m')throw Error('manual distance missing');
 state.obstacles=[{x:2.458,y:.85,diameter_m:.1}];state.map_match=false;vm.runInContext('update()',sandbox);
 if(node('status').textContent!=='MAP MISMATCH')throw Error('map mismatch label missing');
 console.log('Live frontend + measured speed + TELEOP model/tracks + obstacle/map status OK');
},100);
