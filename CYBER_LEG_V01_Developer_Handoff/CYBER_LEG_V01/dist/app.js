import {installMotorConsole} from './motor-console.js';
import {WalkingDummy} from './dummy.js';
import * as THREE from './three.module.min.js';
import {rad,clamp,terrain,makeStones,wrapStone,stoneProfile,surfaceHeight,supportSlope,feasiblePose} from './motion.js';
import {GaitEngine,ACTIVITY_LABELS} from './gait.js';
import {EnergyMeter,DEVICES} from './energy.js';
const $=id=>document.getElementById(id);
const state={activity:'walk',stairsDir:'stairsUp',chart:'ankle',targetHip:0,targetKnee:0,actualHip:null,actualKnee:null,clamped:false,ready:false,running:false,mode:'manual',time:0,phase:0,distance:0,knee:0,kneeVelocity:0,pitch:0,roll:0,height:0,hip:0,hipVelocity:0,correctionPitch:0,correctionRoll:0,extensions:[20,20,20],limited:false};
let dummy;
let scene,camera,renderer,root,shank,foot,cad,meshes=[],actuators=[],anchors=[],contactVertices=[],axisGroup,terrainMesh;
let kneePivot=new THREE.Vector3(-7.4,387.77,.831),anklePivot=new THREE.Vector3(.435,-16.962,.831);
let stones=makeStones(crypto.getRandomValues(new Uint32Array(1))[0]),stoneMeshes=[],filteredPitch=0,filteredRoll=0;
const history=[],samples=[];let lastSample=0,lastTerrain=-1,baseGround=94.582;
const orbit={theta:1.05,phi:1.27,radius:2750,target:new THREE.Vector3(25,820,-60)};
const clock=new THREE.Clock();let drag=null,joyDrag=false,pointerDown=null;
let ankleLimits=null;
// Gait rig. u = knee->hip in the thigh frame (matches dummy.js hipLocal). The
// activity engine (gait.js) moves the pelvis; the joint angles are what the motors receive.
const HIP_LOCAL=[-22,395];let rig=null,engine=null,gp=null;
const energy=new EnergyMeter(),jointHistory=[];
let treadmillParts=[],floorMesh,stairGroup,stairMeshes=[],chairGroup,obstacleMeshes=[],lastStats=-1;
let bridgeToken=null,bridgeBusy=false,bridgeArmed=false,bridgeArmPending=false,bridgeGeneration=0,bridgeQueue=Promise.resolve();
function applyMotorLimits(data){
 const rotary=data.console?.rotary;if(!rotary)return;
 for(const name of ['knee','hip']){
  const r=rotary[name];if(!r)continue;
  $(name).min=String(r.limits_deg[0]);$(name).max=String(r.limits_deg[1]);
  $(name+'MinLabel').textContent=r.limits_deg[0].toFixed(1)+'°';$(name+'MaxLabel').textContent=r.limits_deg[1].toFixed(1)+'°';
  $(name).value=clamp(+$(name).value,r.limits_deg[0],r.limits_deg[1]);
  state[name]=clamp(state[name],r.limits_deg[0],r.limits_deg[1]);
 }
}
const onlineConsole=installMotorConsole({post:bridgePost,onLimits(data){applyMotorLimits(data);if(state.ready&&!state.running)solvePose(0);},onReference(target){
 if(target==='all'||target==='knee'){state.knee=0;$('knee').value=0;}
 if(target==='all'||target==='hip'){state.hip=0;$('hip').value=0;}
 if(target==='all'||target==='ankle'){state.pitch=state.roll=state.height=0;$('height').value=0;state.extensions=[20,20,20];}
 if(state.ready)solvePose(0);
}});
function bridgeDisplay(data){
 applyMotorLimits(data);onlineConsole.render({...data,simulator_running:state.running});
 const rot=data.console?.rotary;state.actualKnee=rot?.knee?.stale===false?rot.knee.actual_angle_deg:null;state.actualHip=rot?.hip?.stale===false?rot.hip.actual_angle_deg:null;
 energy.ingest(data.console,performance.now()/1000);
 const ankle=data.ankle;
 ankleLimits=ankle?.enabled?(ankle.limits_mm||[[0,39],[0,39],[0,39]]):null;
 $('ankleStatus').textContent=!ankle?.enabled?'Ankle control disabled':ankle.error?`Ankle: ${ankle.error}`:ankle.arming?'Ankle connecting and checking nodes 1/2/3...':ankle.armed?'Ankle armed: A1/A2/A3 = nodes 1/2/3':'Ankle disarmed';
 $('ankleFeedback').textContent=ankle?.relative_positions_mm?ankle.relative_positions_mm.map((p,i)=>`A${i+1}: ${p.toFixed(2)} mm from zero`).join(' / '):'Ankle tilt and height move all three actuators together.';
 const kneeEnabled=data.knee_enabled!==false;
 $('motorArm').textContent='Arm '+(data.outputs||['knee','hip','ankle']).join(' + ');
 $('motorZero').disabled=!kneeEnabled||!bridgeToken||data.armed||bridgeArmPending||state.running;
 bridgeArmed=!!data.armed;$('motorArm').disabled=!bridgeToken||bridgeArmed||bridgeArmPending||!state.ready;$('motorDisarm').disabled=!bridgeToken||(!bridgeArmed&&!bridgeArmPending);
 const gains=data.knee_enabled!==false&&data.gains?` \u00b7 knee P/D ${data.gains.kp_pos}/${data.gains.kd_pos}`:'';
 $('motorStatus').textContent=(data.error?`Motor: ${data.error}`:bridgeArmed?'Motor armed':'Motor disarmed')+gains;
 const hip=data.hip,hf=hip?.feedback;
 $('hipStatus').textContent=!hip?.enabled?'Hip output disabled':hip.error?`Hip: ${hip.error}`:hip.armed?'Hip armed · CAN ID 2':'Hip disarmed · CAN ID 2';
 if(hip?.enabled&&hip.gains)$('hipStatus').textContent+=` · P/D ${hip.gains.kp_pos}/${hip.gains.kd_pos}`;
 const hipDegrees=value=>(value-hip.offset_rad-(hip.reference_rad||0))/hip.sign/rad+hip.reference_angle_deg;
 const hipCommand=hip?.armed&&Number.isFinite(hip.setpoint_rad)?` · command ${hipDegrees(hip.setpoint_rad).toFixed(1)}°`:'';
 $('hipFeedback').textContent=hf?`Hip actual ${hipDegrees(hf.position_rad).toFixed(1)}°${hipCommand} · torque ${hf.torque_nm.toFixed(2)} Nm · fault ${hf.fault}`:'Hip range: -30° to +30°; neutral hip (0°) = motor zero.';
 $('hipZero').disabled=!hip?.enabled||!bridgeToken||data.armed||bridgeArmPending||state.running;
 const f=data.feedback;
 $('motorFeedback').textContent=!kneeEnabled?'Knee output disabled':f?`Actual ${((f.position_rad-data.offset_rad-(data.reference_rad||0))/data.sign/rad).toFixed(1)}° · ${(f.velocity_rad_s/rad).toFixed(1)}°/s · fault ${f.fault}${bridgeArmed?` · command ${(data.setpoint_rad/rad).toFixed(1)}°`:''}`:'Stop holds; reset disarms.';
}
function currentBridgeTarget(){
 return {angle_deg:state.knee,hip_angle_deg:state.hip,
  hip_velocity_deg_s:state.running&&state.mode==='auto'?state.hipVelocity:0,
  extensions_mm:[...state.extensions],
  velocity_deg_s:state.running&&state.mode==='auto'?state.kneeVelocity:0,
  running:state.running,mode:state.mode};
}
function bridgePost(route,payload={}){
 const operation=bridgeQueue.catch(()=>{}).then(async()=>{
  const response=await fetch(`/api/${route}`,{method:'POST',headers:{'Content-Type':'application/json','X-Bridge-Token':bridgeToken},body:JSON.stringify(route==='update'?currentBridgeTarget():payload)});
  const data=await response.json();if(!response.ok)throw Error(data.error||'Bridge request failed');bridgeDisplay(data);return data;
 });bridgeQueue=operation;return operation;
}
async function bridgeDisarm(){bridgeGeneration++;if(!bridgeToken)return;try{await bridgePost('disarm');}catch(e){$('motorStatus').textContent=`Bridge: ${e.message}`;}}
async function bridgeTick(){
 if(!bridgeToken||bridgeBusy||!state.ready)return;bridgeBusy=true;
 try{await bridgePost('update',{angle_deg:state.knee,hip_angle_deg:state.hip,hip_velocity_deg_s:state.running&&state.mode==='auto'?state.hipVelocity:0,extensions_mm:[...state.extensions],velocity_deg_s:state.running&&state.mode==='auto'?state.kneeVelocity:0,running:state.running,mode:state.mode});}catch(e){bridgeArmed=false;$('motorArm').disabled=true;$('motorDisarm').disabled=true;$('motorStatus').textContent=`Bridge disconnected: ${e.message}`;onlineConsole.disconnect();bridgeToken=null;}
 finally{bridgeBusy=false;}
}
async function bridgeConnect(){
 try{const r=await fetch('/api/session',{cache:'no-store'});if(!r.ok)return;const data=await r.json();bridgeToken=data.token;bridgeDisplay(data);await bridgeTick();}
 catch(e){$('motorStatus').textContent='Bridge not connected';}
}
for(let i=0;i<3;i++)$('meters').insertAdjacentHTML('beforeend',`<div class="meterrow"><span>A${i+1}</span><div class="track"><i class="fill" id="bar${i}"></i></div><output id="ext${i}">0.0</output></div>`);
function error(e){$('loading').hidden=false;$('loadTitle').textContent='Simulator could not start';$('loadDetail').textContent=e.message||String(e);$('retry').hidden=false;document.querySelector('.spinner').style.display='none';state.running=false;syncControls();console.error(e);}
async function fetchAsset(path,type){const r=await fetch(path,{signal:AbortSignal.timeout(45000)});if(!r.ok)throw Error(`${path}: HTTP ${r.status}. Please retry.`);return type==='json'?r.json():r.arrayBuffer();}
function center(p){return new THREE.Vector3(...p.min).add(new THREE.Vector3(...p.max)).multiplyScalar(.5);}
function color(p){
 if(p.name==='LOW_POLY_COVER')return 0x536574;
 if(p.name==='ROTOR_LEVERAGE')return 0xc3c9cd;
 if(p.name==='MAIN_BASE_HOLDER')return 0x718596;
 if(p.name==='PISTON')return 0xd6e0e4;
 if(/MOUNT/.test(p.name))return 0xec984f;
 if(p.path.includes('500N'))return p.solid<5?0xb2bdc5:0x6e7c86;
 if(p.name==='base plate')return 0x98a7b0;
 if(/heel|rubber/.test(p.name))return 0x242d32;
 if(p.name==='ankle')return 0x8497a3;
 if(p.path.includes('Battery'))return p.name.includes('Connector')?0xdcb64c:0x42515a;
 if(p.name==='jetson')return 0x468271;
 return 0x9da9b0;
}
function addToRig(mesh,parent,origin){parent.add(mesh);mesh.position.copy(origin).multiplyScalar(-1);}
function buildRig(){
 if(dummy)dummy.dispose();
 if(root)scene.remove(root);
 root=new THREE.Group();scene.add(root);shank=new THREE.Group();root.add(shank);foot=new THREE.Group();shank.add(foot);
 foot.position.copy(anklePivot).sub(kneePivot);
 const bodyRoot=new THREE.Group();root.add(bodyRoot);
 actuators=[];anchors=[];contactVertices=[];
 // Each named actuator instance contains 25 body solids and 3 piston solids.
 for(let a=0;a<3;a++){
  const first=1+a*28,top=center(cad.parts[first+3]),bottom=center(cad.parts[first+27]);
  const barrel=new THREE.Group(),piston=new THREE.Group();shank.add(barrel,piston);
  const direction=bottom.clone().sub(top),length=direction.length();
  actuators.push({barrel,piston,top,bottom,direction:direction.normalize(),length,first});
  anchors.push({top:top.toArray(),bottom:bottom.toArray(),length});
 }
 for(const mesh of meshes){
  const p=mesh.userData.part;
  if(p.id>=1&&p.id<=84){const a=actuators[Math.floor((p.id-1)/28)];const sliding=p.name==='PISTON'||(p.name==='500N Linear Actuator'&&[1,2,6].includes(p.solid));addToRig(mesh,sliding?a.piston:a.barrel,sliding?a.bottom:a.top);}
  else if(p.id===87)addToRig(mesh,bodyRoot,kneePivot);
  else if(p.id===86||p.path.includes('completed foot assembly')){
   addToRig(mesh,foot,anklePivot);
   const v=mesh.geometry.attributes.position.array;
   // Actual CAD sole vertices supply floor contact, not a proxy foot box.
   if(['heel final','base plate','CARRAIGE BOLT_M8x20-CAR-ZP'].includes(p.name))for(let i=0;i<v.length;i+=3)if(v[i+1]<-75)contactVertices.push(new THREE.Vector3(v[i],v[i+1],v[i+2]).sub(anklePivot));
  }else addToRig(mesh,shank,kneePivot);
 }
 // Keep a distributed sole sample for interactive ground clearance checks.
 const cells=new Map();for(const v of contactVertices){const key=`${Math.round(v.x/5)},${Math.round(v.z/5)}`;if(!cells.has(key)||cells.get(key).y>v.y)cells.set(key,v);}
 contactVertices=[...cells.values()];
 rig={u:HIP_LOCAL,w:[anklePivot.x-kneePivot.x,anklePivot.y-kneePivot.y],contact:contactVertices.map(v=>v.toArray()),anchors,pivot:anklePivot.toArray(),limits:null,z:anklePivot.z,key:`${kneePivot.y}|${anklePivot.y}`};
 engine=new GaitEngine(rig);applyGaitParams(false);
 axisGroup=new THREE.Group();root.add(axisGroup);
 // Same rotary motor type as the knee; illustrative housing at the hip.
 const hipMotor=new THREE.Mesh(new THREE.CylinderGeometry(48,48,65,32),new THREE.MeshStandardMaterial({color:0x52c8bd,metalness:.65,roughness:.3}));
 hipMotor.rotation.x=Math.PI/2;hipMotor.position.set(-22,395,0);root.add(hipMotor);
 const hipAxis=new THREE.ArrowHelper(new THREE.Vector3(0,0,1),new THREE.Vector3(-22,395,-70),140,0x52c8bd,12,7);axisGroup.add(hipAxis);
 const kneeAxis=new THREE.ArrowHelper(new THREE.Vector3(0,0,1),new THREE.Vector3(0,0,-70),140,0xff903d,12,7);axisGroup.add(kneeAxis);
 const ankleAxes=new THREE.AxesHelper(65);ankleAxes.position.copy(anklePivot).sub(kneePivot);axisGroup.add(ankleAxes);
 axisGroup.visible=$('joints').checked;
 $('kp').value=kneePivot.y.toFixed(2);$('ap').value=anklePivot.y.toFixed(3);
 dummy=new WalkingDummy(scene,root,kneePivot,{upper:Math.hypot(...HIP_LOCAL),lower:Math.hypot(...rig.w),contact:rig.contact});dummy.setOpacity(+$('dummyOpacity').value/100);dummy.setVisible($('dummy').checked);
 reset();
}
function fitView(){const show=$('dummy').checked;orbit.radius=show?2750:1050;orbit.target.set(25,show?820:290,show?-60:0);cameraUpdate();}
function cameraUpdate(){camera.position.set(orbit.target.x+orbit.radius*Math.sin(orbit.phi)*Math.cos(orbit.theta),orbit.target.y+orbit.radius*Math.cos(orbit.phi),orbit.target.z+orbit.radius*Math.sin(orbit.phi)*Math.sin(orbit.theta));camera.lookAt(orbit.target);}
function resize(){const r=$('viewport').getBoundingClientRect();renderer.setSize(r.width,r.height,false);camera.aspect=r.width/r.height;camera.updateProjectionMatrix();drawChart();}
function initScene(){
 scene=new THREE.Scene();scene.fog=new THREE.Fog(0x141b22,3000,6000);
 camera=new THREE.PerspectiveCamera(38,1,1,6000);
 try{renderer=new THREE.WebGLRenderer({canvas:$('scene'),antialias:true,alpha:true,powerPreference:'high-performance'});}catch(e){throw Error('WebGL 2 is unavailable. Enable browser hardware acceleration and reload.');}
 renderer.setPixelRatio(Math.min(devicePixelRatio||1,1.75));renderer.setClearColor(0x131b22,0);renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.55;
 scene.add(new THREE.HemisphereLight(0xddeeff,0x34322e,2.4));
 for(const [pos,intensity] of [[[200,800,500],3], [[-400,450,-350],2],[[500,200,-100],1]]){const l=new THREE.DirectionalLight(0xffffff,intensity);l.position.set(...pos);scene.add(l);}
 const deck=new THREE.Mesh(new THREE.BoxGeometry(1160,34,485),new THREE.MeshStandardMaterial({color:0x303c46,metalness:.5,roughness:.6}));deck.position.set(0,-26,0);scene.add(deck);treadmillParts.push(deck);
 for(const z of [-253,253]){const rail=new THREE.Mesh(new THREE.BoxGeometry(1180,35,20),new THREE.MeshStandardMaterial({color:0x77838c,metalness:.7,roughness:.4}));rail.position.set(0,-20,z);scene.add(rail);treadmillParts.push(rail);}
 buildProps();
 const geo=new THREE.PlaneGeometry(1140,464,180,52);geo.rotateX(-Math.PI/2);
 terrainMesh=new THREE.Mesh(geo,new THREE.MeshStandardMaterial({color:0x394952,roughness:.96,metalness:.1,side:THREE.DoubleSide,vertexColors:true}));scene.add(terrainMesh);treadmillParts.push(terrainMesh);
 const colors=new Float32Array(geo.attributes.position.count*3);geo.setAttribute('color',new THREE.BufferAttribute(colors,3));
 const grid=new THREE.GridHelper(2600,26,0x3c4952,0x29353e);grid.position.y=-45;scene.add(grid);
 buildStones();cameraUpdate();new ResizeObserver(resize).observe($('viewport'));resize();
 $('scene').addEventListener('webglcontextlost',e=>{e.preventDefault();error(Error('The graphics context was lost. Reload to restart the simulator.'));});
}
function buildStones(){
 for(const mesh of stoneMeshes){scene.remove(mesh);mesh.geometry.dispose();mesh.material.dispose();}stoneMeshes=[];
 for(const [i,s] of stones.entries()){
  const vertices=[0,1,0],indices=[];
  for(const q of [.45,1])for(let k=0;k<8;k++){const a=Math.PI/8+k*Math.PI/4,r=q/Math.cos(Math.PI/8);vertices.push(Math.cos(a)*r,stoneProfile(q),Math.sin(a)*r);}
  for(let k=0;k<8;k++){const a=1+k,b=1+(k+1)%8,c=9+k,d=9+(k+1)%8;indices.push(0,b,a,a,b,c,b,d,c);}
  const geo=new THREE.BufferGeometry();geo.setAttribute('position',new THREE.Float32BufferAttribute(vertices,3));geo.setIndex(indices);geo.computeVertexNormals();
  const mesh=new THREE.Mesh(geo,new THREE.MeshStandardMaterial({color:[0xb8aa91,0x998f7a,0xc6b79d][i%3],roughness:1,flatShading:true,side:THREE.DoubleSide}));mesh.rotation.y=-s.angle;scene.add(mesh);stoneMeshes.push(mesh);
 }
 updateStones();
}
function updateStones(){
 const amp=$('uneven').checked?+$('roughness').value:0;
 stoneMeshes.forEach((mesh,i)=>{const s=stones[i],x=wrapStone(s.x-state.distance);mesh.visible=$('stones').checked&&Math.abs(x)<535&&(state.mode==='manual'||state.activity==='walk');mesh.position.set(x,terrain(x,s.z,state.distance,amp),s.z);mesh.scale.set(s.rx,s.height*+$('stoneHeight').value,s.rz);});
}
function updateTerrain(){
 const amp=$('uneven').checked?+$('roughness').value:0,geo=terrainMesh.geometry,v=geo.attributes.position,c=geo.attributes.color;
 for(let i=0;i<v.count;i++){const x=v.getX(i),z=v.getZ(i);v.setY(i,terrain(x,z,state.distance,amp));const stripe=Math.floor((x+state.distance)/32)%2===0?.79:1; c.setXYZ(i,stripe,stripe,stripe);}
 v.needsUpdate=true;c.needsUpdate=true;geo.computeVertexNormals();updateStones();
}
// Scene props for the activities: floor, moving staircase, chair, trip obstacles.
function buildProps(){
 floorMesh=new THREE.Mesh(new THREE.BoxGeometry(1600,30,900),new THREE.MeshStandardMaterial({color:0x2e3a43,roughness:.9,metalness:.1}));floorMesh.position.set(0,-15,0);floorMesh.visible=false;scene.add(floorMesh);
 stairGroup=new THREE.Group();scene.add(stairGroup);
 const stairMat=new THREE.MeshStandardMaterial({color:0x56646e,roughness:.85,metalness:.15}),noseMat=new THREE.MeshStandardMaterial({color:0xff903d,roughness:.5,metalness:.2});
 for(let i=0;i<30;i++){const step=new THREE.Mesh(new THREE.BoxGeometry(1,1,1),stairMat),nose=new THREE.Mesh(new THREE.BoxGeometry(1,1,1),noseMat);stairGroup.add(step,nose);stairMeshes.push({step,nose});}
 chairGroup=new THREE.Group();scene.add(chairGroup);chairGroup.visible=false;
 const wood=new THREE.MeshStandardMaterial({color:0x8a6a4c,roughness:.7,metalness:.05}),frame=new THREE.MeshStandardMaterial({color:0x55616a,roughness:.5,metalness:.6});
 const seat=new THREE.Mesh(new THREE.BoxGeometry(430,28,520),wood);seat.position.set(-40,-14,-85);chairGroup.add(seat);
 const back=new THREE.Mesh(new THREE.BoxGeometry(26,420,520),wood);back.position.set(-255,210,-85);chairGroup.add(back);
 for(const x of [-235,155])for(const z of [-325,155]){const leg=new THREE.Mesh(new THREE.CylinderGeometry(13,13,1,12),frame);leg.userData.leg=true;leg.position.set(x,0,z);chairGroup.add(leg);}
 const obstacleMat=new THREE.MeshStandardMaterial({color:0xff5f57,roughness:.6,metalness:.1});
 for(let i=0;i<4;i++){const m=new THREE.Mesh(new THREE.BoxGeometry(26,34,180),obstacleMat);m.visible=false;scene.add(m);obstacleMeshes.push(m);}
}
function updateProps(p){
 const walk=state.mode==='manual'||state.activity==='walk';
 for(const m of treadmillParts)m.visible=walk;
 if(!walk)for(const m of stoneMeshes)m.visible=false;
 floorMesh.visible=state.mode==='auto'&&state.activity==='sit';
 stairGroup.visible=!!p?.stairs;
 if(p?.stairs){
  const t=p.stairs.treads,up=p.stairs.rise>0;
  stairMeshes.forEach(({step,nose},i)=>{
   const tr=t[i];step.visible=nose.visible=!!tr&&tr.x1>-1200&&tr.x0<1200;if(!step.visible)return;
   const depth=900,w=tr.x1-tr.x0;
   step.scale.set(w,depth,620);step.position.set((tr.x0+tr.x1)/2,tr.y-depth/2,0);
   nose.scale.set(16,6,622);nose.position.set(up?tr.x0+8:tr.x1-8,tr.y-2,0);
  });
 }
 chairGroup.visible=!!p?.chair;
 if(p?.chair){chairGroup.position.set(p.chair.seatX,p.chair.seatY,0);for(const c of chairGroup.children)if(c.userData.leg){c.scale.y=Math.max(10,p.chair.seatY-28);c.position.y=-28-c.scale.y/2;}}
 obstacleMeshes.forEach((m,i)=>{const o=p?.obstacles?.[i];m.visible=!!o&&walk&&Math.abs(o.x)<560;if(m.visible){const amp=$('uneven').checked?+$('roughness').value:0;m.position.set(o.x,terrain(o.x,o.z,state.distance,amp)+17,o.z);}});
}
function surfaceEnv(){
 const amp=$('uneven').checked?+$('roughness').value:0,st=$('stones').checked?stones:[],sh=+$('stoneHeight').value;
 return {surface:(x,z,d)=>surfaceHeight(x,z,d,amp,st,sh),support:$('adaptive').checked?(x,z,d)=>supportSlope(x,z,d,amp,st,sh):null};
}
function applyGaitParams(resetIfNeeded=true){
 if(!engine)return;
 const old={...engine.params};
 Object.assign(engine.params,{clearance:+$('clearance').value,pushOff:+$('pushOff').value/100,stanceKnee:+$('stanceKnee').value/100,
  stairCadence:+$('stairCadence').value,sitHold:+$('sitHold').value,autoStumble:$('autoTrip').checked});
 // Stair treads are laid out from the joint profiles; a profile change needs a fresh layout.
 if(resetIfNeeded&&state.mode==='auto'&&state.activity.startsWith('stairs')&&old.clearance!==engine.params.clearance){const running=state.running;engine.reset(state.activity);if(running)engine.startTime=0;}
}
function solvePose(dt){
 let hip=+$('hip').value,knee=+$('knee').value,pose;
 state.correctionPitch=0;state.correctionRoll=0;gp=null;
 if(state.mode==='auto'){
  const s=surfaceEnv(),walk=state.activity==='walk';
  rig.limits=ankleLimits;
  gp=engine.step(dt,{running:state.running,speed:+$('speed').value,surface:walk?s.surface:null,support:walk?s.support:null,
   height:+$('height').value,ankleLimits,fixedMount:$('fixedMount').checked});
  state.phase=gp.phase;state.distance=gp.groundX;
  state.targetHip=gp.target.hip;state.targetKnee=gp.target.knee;
  // Bridge-reported motor limits bound the command. The model shows the target pose.
  hip=clamp(gp.target.hip,+$('hip').min,+$('hip').max);knee=clamp(gp.target.knee,+$('knee').min,+$('knee').max);
  const hipClamped=Math.abs(hip-gp.target.hip)>.05,kneeClamped=Math.abs(knee-gp.target.knee)>.05;
  state.clamped=hipClamped||kneeClamped;
  // A clamped joint is held at its limit: no velocity feed-forward into the stop.
  state.kneeVelocity=state.running&&!kneeClamped?gp.rates.knee:0;state.hipVelocity=state.running&&!hipClamped?gp.rates.hip:0;
  pose={pitch:gp.our.ankle,roll:gp.our.roll,height:gp.our.height,extensions:gp.our.extensions,limited:gp.our.limited};
  state.correctionPitch=pose.pitch-gp.nominalAnkle;state.correctionRoll=pose.roll;
  const sup=gp.support;state.supportDelta=sup?sup.left-sup.right:0;
  if(walk)$('supportState').textContent=gp.stance&&sup?`Left / right support: ${sup.left.toFixed(1)} / ${sup.right.toFixed(1)} mm · Δ ${state.supportDelta.toFixed(1)} mm · weight ${gp.adaptWeight.toFixed(2)}`:gp.stance?'Left / right support: adaptive off':`Swing · pre-shaping to landing (${gp.adaptWeight.toFixed(2)})`;
  $('phaseName').textContent=gp.label;$('phaseProgress').style.width=`${gp.progress*100}%`;
  const badge=$('clampBadge');badge.hidden=!state.clamped;
  if(state.clamped)badge.textContent=`MOTOR LIMIT · ${hipClamped?`hip ${gp.target.hip.toFixed(0)}° → ${hip.toFixed(0)}°`:''}${hipClamped&&kneeClamped?' · ':''}${kneeClamped?`knee ${gp.target.knee.toFixed(0)}° → ${knee.toFixed(0)}°`:''}`;
 }else{
  state.correctionPitch=state.pitch;state.correctionRoll=state.roll;$('phaseName').textContent='MANUAL CONTROL';$('phaseProgress').style.width='0%';$('clampBadge').hidden=true;state.clamped=false;
  pose=feasiblePose(anchors,anklePivot.toArray(),state.pitch,state.roll,+$('height').value,ankleLimits);state.kneeVelocity=0;state.hipVelocity=0;
  state.targetHip=hip;state.targetKnee=knee;
 }
 state.knee=knee;state.hip=hip;state.extensions=pose.extensions;state.limited=pose.limited;state.appliedPitch=pose.pitch;state.appliedRoll=pose.roll;state.height=pose.height;
 const showHip=gp?gp.target.hip:hip,showKnee=gp?gp.target.knee:knee;
 root.rotation.z=showHip*rad;
 if(gp){// The pelvis (hip joint) comes from the activity engine; the knee pivot hangs from it.
  const c=Math.cos(showHip*rad),sn=Math.sin(showHip*rad);root.position.set(gp.pelvis[0]-(c*HIP_LOCAL[0]-sn*HIP_LOCAL[1]),gp.pelvis[1]-(sn*HIP_LOCAL[0]+c*HIP_LOCAL[1]),kneePivot.z);
 }else{// Manual: rotate the entire prosthesis about the hip, keeping the pelvis upright.
  const hipLocal=new THREE.Vector3(...HIP_LOCAL,0);
  root.position.set(kneePivot.x,kneePivot.y+baseGround,kneePivot.z).add(hipLocal).sub(hipLocal.clone().applyAxisAngle(new THREE.Vector3(0,0,1),hip*rad));
 }
 shank.rotation.z=-showKnee*rad;
 foot.position.copy(anklePivot).sub(kneePivot);foot.position.y-=pose.height;
 foot.rotation.set(pose.roll*rad,0,pose.pitch*rad,'ZXY');
 for(const a of actuators){
  const local=a.bottom.clone().sub(anklePivot).applyQuaternion(foot.quaternion).add(anklePivot);local.y-=pose.height;
  const direction=local.clone().sub(a.top).normalize();const q=new THREE.Quaternion().setFromUnitVectors(a.direction,direction);
  a.barrel.position.copy(a.top).sub(kneePivot);a.barrel.quaternion.copy(q);
  a.piston.position.copy(local).sub(kneePivot);a.piston.quaternion.copy(q);
 }
 root.updateMatrixWorld(true);
 updateProps(gp);
 const heightAt=state.mode==='manual'||state.activity==='walk'?(x,z)=>surfaceHeight(x,z,state.distance,$('uneven').checked?+$('roughness').value:0,$('stones').checked?stones:[],+$('stoneHeight').value):()=>0;
 if(dummy)dummy.update(state,gp?{soundAnkle:gp.sound.ankleWorld,soundPitch:gp.sound.footPitch,trunk:gp.trunk,swing:state.activity!=='sit',phase:gp.phase}:{heightAt});
 $('hipV').textContent=`${hip.toFixed(1)}°`;$('kneeV').textContent=`${knee.toFixed(1)}°`;$('angles').textContent=`${pose.pitch.toFixed(1)}° / ${pose.roll.toFixed(1)}°`;
 if(state.mode==='auto'){$('knee').value=knee;$('hip').value=hip;}
 pose.extensions.forEach((e,i)=>{$(`bar${i}`).style.left=`${clamp(Math.min(20,e)/50,0,1)*100}%`;$(`bar${i}`).style.width=`${clamp(Math.abs(e-20)/50,0,1)*100}%`;$(`ext${i}`).textContent=(e-20).toFixed(1);});
 $('limitState').textContent=pose.limited?'Travel limit reached · command reduced':'Within assumed 0–50 mm travel';$('limitState').classList.toggle('warn',pose.limited);
}
function updateStats(){
 if(!gp)return;
 if(state.activity==='walk'){const D=gp.stride.x;$('gaitStats').textContent=`Stride ${D.toFixed(0)} mm · ${gp.cadence.toFixed(0)} strides/min · ${engine.stumbles} trips`;}
 else if(gp.stairs)$('stairStats').textContent=`Step ${Math.abs(gp.stairs.rise).toFixed(0)} mm rise × ${gp.stairs.run.toFixed(0)} mm run · ${(gp.cadence*2).toFixed(0)} steps/min`;
 else if(gp.chair)$('sitStats').textContent=`${gp.label.toLowerCase()} · seat ${(gp.chair.seatY).toFixed(0)} mm above floor · ${engine.cycles} cycles`;
 const trip=engine.lastTrip;
 if(trip&&state.time-trip.time<2.5){$('tripState').textContent=trip.type==='elevating'?'Trip in early swing → elevating: knee and hip flex to lift the foot over.':'Trip in late swing → lowering: foot placed early with a shorter step.';$('tripState').classList.add('event');}
 else if($('tripState').classList.contains('event')){$('tripState').textContent='Early swing → elevating strategy (lift the foot over). Late swing → lowering strategy (short, early step).';$('tripState').classList.remove('event');}
 const note=$('clampState');note.hidden=!state.clamped;
 if(state.clamped)note.textContent=`This activity needs hip ${state.targetHip.toFixed(0)}° / knee ${state.targetKnee.toFixed(0)}°. Motor commands are clamped at the configured limits (hip ${$('hip').min}…${$('hip').max}°, knee ${$('knee').min}…${$('knee').max}°); the model shows the full target.`;
}
function syncControls(){
 onlineConsole.setRunning(state.running);
 const label=state.mode==='auto'?(ACTIVITY_LABELS[state.activity]||'AUTO').toUpperCase():'MANUAL';
 $('runState').textContent=state.running?`${label} / RUNNING`:'STOPPED';$('runState').classList.toggle('running',state.running);
 $('start').disabled=!state.ready||state.running;$('stop').disabled=!state.ready||!state.running;$('reset').disabled=!state.ready;
 $('hip').disabled=!state.running||state.mode==='auto';$('knee').disabled=!state.running||state.mode==='auto';$('height').disabled=!state.running;$('joy').classList.toggle('disabled',!state.running||state.mode!=='manual');
 const sel=state.mode==='manual'?'manual':state.activity==='walk'?'auto':state.activity==='sit'?'sit':'stairs';
 for(const id of ['manual','auto','stairs','sit'])$(id).classList.toggle('selected',id===sel);
 $('activityPanel').hidden=state.mode!=='auto';
 $('walkPanel').hidden=state.activity!=='walk';$('stairsPanel').hidden=!state.activity.startsWith('stairs');$('sitPanel').hidden=state.activity!=='sit';
 $('surfacePanel').hidden=state.mode==='auto'&&state.activity!=='walk';
 $('activityName').textContent=(ACTIVITY_LABELS[state.activity]||'').toUpperCase();
 $('stairsUp').classList.toggle('selected',state.stairsDir==='stairsUp');$('stairsDown').classList.toggle('selected',state.stairsDir==='stairsDown');
 $('trip').disabled=!state.running||state.mode!=='auto'||state.activity!=='walk';
 $('modeHelp').textContent=state.mode!=='auto'?'Start to enable joint controls. Stop freezes the pose.':state.activity==='walk'?'Start the walking cycle. Stop holds the current pose.':state.activity==='sit'?'Start the sit-to-stand cycle. Stop holds the current pose.':'Start climbing. Stop holds the current pose.';
}
function reset(){
 if(bridgeToken)bridgeDisarm();
 Object.assign(state,{running:false,time:0,phase:0,distance:0,knee:0,kneeVelocity:0,pitch:0,roll:0,height:0,hip:0,hipVelocity:0});$('hip').value=0;$('knee').value=0;$('height').value=0;$('heightV').textContent='0.0 mm';$('stick').style.transform='translate(0,0)';history.length=0;samples.length=0;jointHistory.length=0;energy.reset();lastSample=0;lastTerrain=-1;filteredPitch=0;filteredRoll=0;state.supportDelta=0;$('supportState').textContent='Left / right support: —';
 // Auto modes show the starting pose before Start.
 if(engine)engine.reset(state.activity,{speed:+$('speed').value});
 solvePose(0);updateStats();updateTerrain();drawChart();renderEnergy();syncControls();$('simTime').textContent='00:00.0';
}
function setJoystick(x,y){if(!state.running||state.mode!=='manual')return;const len=Math.hypot(x,y);if(len>1){x/=len;y/=len;}state.pitch=-y*18;state.roll=x*12;$('stick').style.transform=`translate(${x*43}px,${y*43}px)`;}
function releaseJoy(){joyDrag=false;if(state.running&&state.mode==='manual')setJoystick(0,0);}
function chartFrame(el){const r=el.getBoundingClientRect(),dpr=Math.min(devicePixelRatio||1,2);el.width=Math.max(1,r.width*dpr);el.height=Math.max(1,r.height*dpr);const ctx=el.getContext('2d');ctx.scale(dpr,dpr);ctx.font='11px monospace';return {ctx,w:r.width,h:r.height};}
function plotLines(ctx,w,h,rows,now,series,lo,hi,ticks){
 const pad=32,y=v=>8+(hi-v)/(hi-lo)*(h-25);
 ctx.lineWidth=1;ctx.strokeStyle='#303c46';ctx.fillStyle='#82909b';
 for(const v of ticks){ctx.beginPath();ctx.moveTo(pad,y(v));ctx.lineTo(w,y(v));ctx.stroke();ctx.fillText(String(Math.round(v)),0,y(v)+4);}
 for(const s of series){
  ctx.strokeStyle=s.color;ctx.lineWidth=s.width||1.8;ctx.setLineDash(s.dash||[]);ctx.beginPath();let first=true;
  for(const row of rows){const v=row[s.key];if(v==null||!Number.isFinite(v)){first=true;continue;}const x=w-(now-row.t)/10*(w-pad);if(x<pad)continue;first?ctx.moveTo(x,y(v)):ctx.lineTo(x,y(v));first=false;}
  ctx.stroke();ctx.setLineDash([]);
 }
 ctx.fillStyle='#82909b';ctx.fillText('−10 s',pad,h-1);ctx.fillText('now',w-25,h-1);
}
function niceRange(rows,keys,minSpan){
 let lo=Infinity,hi=-Infinity;for(const r of rows)for(const k of keys){const v=r[k];if(v!=null&&Number.isFinite(v)){lo=Math.min(lo,v);hi=Math.max(hi,v);}}
 if(!Number.isFinite(lo)){lo=-minSpan/2;hi=minSpan/2;}
 const span=Math.max(minSpan,hi-lo),mid=(hi+lo)/2,step=span>120?40:span>60?20:span>30?10:5;
 lo=Math.floor((mid-span/2-step*.2)/step)*step;hi=Math.ceil((mid+span/2+step*.2)/step)*step;
 const ticks=[];for(let v=lo;v<=hi+1e-9;v+=step)ticks.push(v);return {lo,hi,ticks};
}
function drawChart(){
 const {ctx,w,h}=chartFrame($('chart'));
 if(state.chart==='joints'){
  const keys=['hipT','hipC','hipA','kneeT','kneeC','kneeA'],r=niceRange(jointHistory,keys,40);
  plotLines(ctx,w,h,jointHistory,state.time,[
   {key:'hipT',color:'#52c8bd',dash:[5,4],width:1.2},{key:'kneeT',color:'#ff903d',dash:[5,4],width:1.2},
   {key:'hipC',color:'#52c8bd'},{key:'kneeC',color:'#ff903d'},
   {key:'hipA',color:'#a8f0e4',width:1.2},{key:'kneeA',color:'#ffd2a8',width:1.2}],r.lo,r.hi,r.ticks);
 }else if(state.chart==='energy'){
  const now=performance.now()/1000,rows=energy.history.map(x=>({...x,t:state.time-(now-x.t)}));
  const r=niceRange(rows,['knee','hip','ankle','total'],4);
  plotLines(ctx,w,h,rows,state.time,[{key:'total',color:'#e8edf0',width:1.2},{key:'knee',color:'#ff903d'},{key:'hip',color:'#52c8bd'},{key:'ankle',color:'#b79cff'}],r.lo,r.hi,r.ticks);
  if(!energy.history.some(x=>x.total!=null)){ctx.fillStyle='#82909b';ctx.fillText('No live motor feedback · arm the bridge to measure power',40,h/2);}
 }else{
  plotLines(ctx,w,h,history.map(x=>({...x,t:x.time})),state.time,[{key:'pitch',color:'#ff903d'},{key:'roll',color:'#6bd4e5'}],-20,20,[-20,-10,0,10,20]);
 }
}
const LEGENDS={
 ankle:'<span class="legend pitch"></span> Pitch <span class="legend roll"></span> Roll <span class="muted">±20° · last 10 s</span>',
 joints:'<span class="legend hip"></span> Hip <span class="legend knee"></span> Knee <span class="muted">dashed target · solid command · light actual</span>',
 energy:'<span class="legend knee"></span> Knee <span class="legend hip"></span> Hip <span class="legend ankle"></span> Ankle <span class="legend total"></span> Total <span class="muted">mechanical W</span>'};
function setChart(name){
 state.chart=name;
 for(const [id,key] of [['tabAnkle','ankle'],['tabJoints','joints'],['tabEnergy','energy']]){$(id).classList.toggle('selected',key===name);$(id).setAttribute('aria-selected',String(key===name));}
 $('chartLegend').innerHTML=LEGENDS[name];$('energyPanel').hidden=name!=='energy';$('actuatorPanel').hidden=name==='energy';
 drawChart();renderEnergy();
}
const fmtW=v=>v==null?'—':Math.abs(v)>=100?v.toFixed(0):v.toFixed(1),fmtJ=v=>Math.abs(v)>=1000?(v/1000).toFixed(2)+'k':v.toFixed(1);
function renderEnergy(){
 if(state.chart!=='energy')return;
 const rows=DEVICES.map(name=>{const d=energy.devices[name];return `<tr><td>${name[0].toUpperCase()+name.slice(1)}</td><td class="${d.power==null?'na':''}">${fmtW(d.power)}</td><td>${fmtJ(d.positive)}</td><td>${fmtJ(-d.negative)}</td></tr>`;});
 const t=energy.totals(),live=DEVICES.filter(n=>energy.devices[n].power!=null),now=live.reduce((a,n)=>a+energy.devices[n].power,0);
 rows.push(`<tr class="total"><td>Total</td><td>${live.length?fmtW(now):'—'}</td><td>${fmtJ(t.positive)}</td><td>${fmtJ(-t.negative)}</td></tr>`);
 $('energyTable').querySelector('tbody').innerHTML=rows.join('');
 const s=live.length?energy.perStride():null;
 const elec=energy.hasElectrical?` Electrical (DC sensor): ${(t.electricalJ/3600).toFixed(3)} Wh.`:' Electrical use needs DC sensors (POST /api/power).';
 $('energyNote').textContent=(live.length?'':'No live feedback · arm the bridge. ')+(s?`Per stride: +${s.positive.toFixed(1)} J / −${(-s.negative).toFixed(1)} J over ${s.duration.toFixed(2)} s.`:'+J delivered, −J absorbed.')+elec;
}
function animate(){
 requestAnimationFrame(animate);const dt=Math.min(clock.getDelta(),.05);
 if(state.ready&&state.running){state.time+=dt;solvePose(dt);
  if(state.mode==='auto')energy.cycle(gp.cycles,state.time);
  if(state.time-lastTerrain>1/30&&(state.mode==='manual'||state.activity==='walk')){updateTerrain();lastTerrain=state.time;}
  if(state.time-lastSample>=.05){lastSample=state.time;
   const row={time:state.time,activity:state.mode==='auto'?state.activity:'manual',phase:state.mode==='auto'?gp.phase:0,pitch:state.correctionPitch,roll:state.correctionRoll,
    knee:state.knee,hip:state.hip,kneeTarget:state.targetKnee,hipTarget:state.targetHip,kneeVelocity:state.kneeVelocity,hipVelocity:state.hipVelocity,
    kneeActual:state.actualKnee,hipActual:state.actualHip,anklePitch:state.appliedPitch,ankleRoll:state.appliedRoll,height:state.height,supportDelta:state.supportDelta||0,
    a1:state.extensions[0]-20,a2:state.extensions[1]-20,a3:state.extensions[2]-20,limited:state.limited?1:0,clamped:state.clamped?1:0,
    pelvisX:gp?gp.pelvis[0]:null,pelvisY:gp?gp.pelvis[1]:null,
    kneePowerW:energy.devices.knee.power,hipPowerW:energy.devices.hip.power,anklePowerW:energy.devices.ankle.power};
   history.push(row);samples.push(row);while(history.length&&history[0].time<state.time-10)history.shift();if(samples.length>72000)samples.shift();
   jointHistory.push({t:state.time,hipT:state.targetHip,hipC:state.hip,hipA:state.actualHip,kneeT:state.targetKnee,kneeC:state.knee,kneeA:state.actualKnee});
   while(jointHistory.length&&jointHistory[0].t<state.time-10)jointHistory.shift();
   drawChart();}
  $('simTime').textContent=`${String(Math.floor(state.time/60)).padStart(2,'0')}:${(state.time%60).toFixed(1).padStart(4,'0')}`;
 }
 const now=performance.now();
 if(now-lastStats>250){lastStats=now;updateStats();renderEnergy();if(!state.running&&state.chart==='energy')drawChart();}
 if(renderer)renderer.render(scene,camera);
}
function events(){
 $('motorZero').onclick=async()=>{if(!bridgeToken||bridgeArmed||state.running)return;$('motorZero').disabled=true;try{await bridgePost('update',{angle_deg:state.knee,hip_angle_deg:state.hip,hip_velocity_deg_s:state.running&&state.mode==='auto'?state.hipVelocity:0,running:false,mode:state.mode});await bridgePost('zero');}catch(e){$('motorStatus').textContent=`Motor: ${e.message}`;}};
 $('motorArm').onclick=async()=>{if(!bridgeToken||!state.ready||bridgeArmPending)return;const generation=bridgeGeneration;bridgeArmPending=true;$('motorArm').disabled=true;$('motorDisarm').disabled=false;try{await bridgePost('update',{angle_deg:state.knee,hip_angle_deg:state.hip,hip_velocity_deg_s:state.running&&state.mode==='auto'?state.hipVelocity:0,extensions_mm:[...state.extensions],velocity_deg_s:state.running&&state.mode==='auto'?state.kneeVelocity:0,running:state.running,mode:state.mode});if(generation===bridgeGeneration)await bridgePost('arm');}catch(e){$('motorStatus').textContent=`Motor: ${e.message}`;}finally{bridgeArmPending=false;$('motorArm').disabled=!bridgeToken||bridgeArmed;$('motorDisarm').disabled=!bridgeToken||!bridgeArmed;}};
 $('hipZero').onclick=async()=>{if(!bridgeToken||bridgeArmed||state.running)return;try{await bridgePost('update',{angle_deg:state.knee,hip_angle_deg:state.hip,running:false,mode:state.mode,extensions_mm:[...state.extensions]});await bridgePost('zero',{joint:'hip'});}catch(e){$('hipStatus').textContent=`Hip: ${e.message}`;}};
 $('motorDisarm').onclick=bridgeDisarm;
 $('start').onclick=()=>{state.running=true;if(engine)engine.startTime=engine.time;clock.getDelta();syncControls();};$('stop').onclick=()=>{state.running=false;joyDrag=false;syncControls();};$('reset').onclick=reset;
 const setActivity=(mode,activity)=>{state.mode=mode;if(activity)state.activity=activity;reset();};
 $('manual').onclick=()=>setActivity('manual');$('auto').onclick=()=>setActivity('auto','walk');
 $('stairs').onclick=()=>setActivity('auto',state.stairsDir);$('sit').onclick=()=>setActivity('auto','sit');
 for(const dir of ['stairsUp','stairsDown'])$(dir).onclick=()=>{state.stairsDir=dir;setActivity('auto',dir);};
 const tuning=[['clearance',v=>`+${v}°`],['pushOff',v=>`${v}%`],['stanceKnee',v=>`${v}%`],['stairCadence',v=>`${v} steps/min`],['sitHold',v=>`${(+v).toFixed(1)} s`]];
 for(const [id,fmt] of tuning)$(id).oninput=()=>{$(id+'V').textContent=fmt($(id).value);applyGaitParams();if(state.ready&&!state.running)solvePose(0);};
 $('gaitDefaults').onclick=()=>{for(const [id,v] of [['clearance',0],['pushOff',100],['stanceKnee',100]]){$(id).value=v;$(id).oninput();}};
 $('autoTrip').onchange=()=>applyGaitParams();
 $('trip').onclick=()=>{if(engine&&state.running)engine.trip();};
 $('fixedMount').onchange=()=>{if(engine)engine.pelvis=[...engine.home];if(state.ready&&!state.running)solvePose(0);};
 for(const [id,key] of [['tabAnkle','ankle'],['tabJoints','joints'],['tabEnergy','energy']])$(id).onclick=()=>setChart(key);
 $('energyReset').onclick=()=>{energy.reset();renderEnergy();drawChart();};
 $('hip').oninput=()=>{$('hipV').textContent=`${(+$('hip').value).toFixed(1)}°`;};$('knee').oninput=()=>{$('kneeV').textContent=`${(+$('knee').value).toFixed(1)}°`;};$('height').oninput=()=>{$('heightV').textContent=`${(+$('height').value).toFixed(1)} mm`;};
 $('speed').oninput=()=>{$('speedV').textContent=`${(+$('speed').value).toFixed(2)} m/s`;$('slow').classList.toggle('selected',+$('speed').value<=.3);$('normal').classList.toggle('selected',+$('speed').value>.3);};
 for(const [id,s] of [['slow',.3],['normal',.6]])$(id).onclick=()=>{$('speed').value=s;$('speed').oninput();};
 $('roughness').oninput=()=>{$('roughnessV').textContent=`${(+$('roughness').value).toFixed(1)} mm`;if(state.ready)updateTerrain();};$('uneven').onchange=()=>state.ready&&updateTerrain();
 $('stones').onchange=()=>state.ready&&updateTerrain();$('stoneHeight').oninput=()=>{$('stoneHeightV').textContent=`${$('stoneHeight').value} mm`;if(state.ready)updateTerrain();};$('shuffleStones').onclick=()=>{stones=makeStones(crypto.getRandomValues(new Uint32Array(1))[0]);buildStones();if(state.ready)reset();};
 $('dummy').onchange=()=>{if(dummy)dummy.setVisible($('dummy').checked);fitView();};$('dummyOpacity').oninput=()=>{$('dummyOpacityV').textContent=`${$('dummyOpacity').value}%`;if(dummy)dummy.setOpacity(+$('dummyOpacity').value/100);};
 $('cover').onchange=()=>{meshes.filter(m=>m.name==='LOW_POLY_COVER').forEach(m=>m.visible=$('cover').checked);};$('joints').onchange=()=>{if(axisGroup)axisGroup.visible=$('joints').checked;};$('wire').onchange=()=>meshes.forEach(m=>m.material.wireframe=$('wire').checked);
 $('applyRig').onclick=()=>{const k=+$('kp').value,a=+$('ap').value;if(!Number.isFinite(k)||!Number.isFinite(a)||k<300||k>450||a< -60||a>20){alert('Use a knee height of 300–450 mm and an ankle height of −60–20 mm.');return;}kneePivot.y=k;anklePivot.y=a;buildRig();};
 const joy=$('joy');joy.onpointerdown=e=>{if(!state.running||state.mode!=='manual')return;joyDrag=true;joy.setPointerCapture(e.pointerId);joy.onpointermove(e);};joy.onpointermove=e=>{if(!joyDrag)return;const r=joy.getBoundingClientRect();setJoystick((e.clientX-r.left-r.width/2)/43,(e.clientY-r.top-r.height/2)/43);};joy.onpointerup=releaseJoy;joy.onpointercancel=releaseJoy;joy.onlostpointercapture=releaseJoy;
 joy.onkeydown=e=>{if(!['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.key))return;e.preventDefault();setJoystick(e.key==='ArrowLeft'?-1:e.key==='ArrowRight'?1:0,e.key==='ArrowUp'?-1:e.key==='ArrowDown'?1:0);};joy.onkeyup=releaseJoy;joy.onblur=releaseJoy;
 const canvas=$('scene');canvas.oncontextmenu=e=>e.preventDefault();canvas.onpointerdown=e=>{drag={x:e.clientX,y:e.clientY,button:e.button};pointerDown={x:e.clientX,y:e.clientY};canvas.setPointerCapture(e.pointerId);};canvas.onpointermove=e=>{if(!drag)return;const dx=e.clientX-drag.x,dy=e.clientY-drag.y;if(drag.button===2||e.shiftKey){const right=new THREE.Vector3().setFromMatrixColumn(camera.matrix,0),up=new THREE.Vector3().setFromMatrixColumn(camera.matrix,1);orbit.target.addScaledVector(right,-dx*orbit.radius*.001).addScaledVector(up,dy*orbit.radius*.001);}else{orbit.theta+=dx*.007;orbit.phi=clamp(orbit.phi+dy*.006,.15,Math.PI-.15);}drag.x=e.clientX;drag.y=e.clientY;cameraUpdate();};canvas.onpointerup=e=>{if(pointerDown&&Math.hypot(e.clientX-pointerDown.x,e.clientY-pointerDown.y)<4&&state.ready){const r=canvas.getBoundingClientRect(),ray=new THREE.Raycaster();ray.setFromCamera(new THREE.Vector2((e.clientX-r.left)/r.width*2-1,-(e.clientY-r.top)/r.height*2+1),camera);const hit=ray.intersectObjects(meshes.filter(m=>m.visible),false)[0];$('partName').textContent=hit?hit.object.userData.part.path.replace('CYBER_LEG_CAD / ',''):'Drag to orbit · wheel to zoom · right-drag to pan';}drag=null;};canvas.onpointercancel=()=>drag=null;
 canvas.addEventListener('wheel',e=>{e.preventDefault();orbit.radius=clamp(orbit.radius*Math.exp(e.deltaY*.001),250,4800);cameraUpdate();},{passive:false});
 document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>{const v=b.dataset.view;if(v==='front'){orbit.theta=0;orbit.phi=Math.PI/2;}else if(v==='side'){orbit.theta=Math.PI/2;orbit.phi=Math.PI/2;}else{orbit.theta=1.05;orbit.phi=1.27;fitView();}cameraUpdate();document.querySelectorAll('[data-view]').forEach(x=>x.classList.toggle('selected',x===b));});
 $('export').onclick=()=>{if(!samples.length){alert('Start the simulator to record telemetry first.');return;}const keys=Object.keys(samples[0]),cell=v=>v==null?'':typeof v==='string'?v:Number(v).toFixed(4),csv=[keys.join(','),...samples.map(s=>keys.map(k=>cell(s[k])).join(','))].join('\n');const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([csv],{type:'text/csv'}));a.download='CYBER_LEG_telemetry.csv';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);};
 document.addEventListener('visibilitychange',()=>{if(document.hidden){if(state.running){state.running=false;syncControls();}if(bridgeToken)bridgeDisarm();}});
}
async function boot(){
 try{
  initScene();events();syncControls();
  const data=await Promise.all([fetchAsset('cad.json','json'),fetchAsset('cad.bin','arrayBuffer')]);cad=data[0];const buffer=data[1];
  $('loadDetail').textContent=`Preparing ${cad.parts.length} original CAD solids…`;
  for(const p of cad.parts){
   const geo=new THREE.BufferGeometry();geo.setAttribute('position',new THREE.BufferAttribute(new Float32Array(buffer,p.vertexOffset,p.vertexCount*3),3));geo.setIndex(new THREE.BufferAttribute(new Uint32Array(buffer,p.indexOffset,p.indexCount),1));geo.computeVertexNormals();geo.computeBoundingSphere();
   const m=new THREE.Mesh(geo,new THREE.MeshStandardMaterial({color:color(p),roughness:p.name==='PISTON'?.28:.52,metalness:p.name==='heel final'?.1:.5,side:THREE.DoubleSide}));m.name=p.name;m.userData.part=p;meshes.push(m);
  }
  buildRig();state.ready=true;syncControls();$('solidCount').textContent=`${meshes.length} SOLIDS`;$('source').textContent='Original STEP · preconverted CAD';$('modelInfo').textContent=`${meshes.length} solids · ${Math.round(cad.triangles/1000)}k triangles · mm`;
  renderer.render(scene,camera);$('loading').hidden=true;animate();bridgeConnect();setInterval(bridgeTick,20);
 }catch(e){error(e);}
}
boot();
