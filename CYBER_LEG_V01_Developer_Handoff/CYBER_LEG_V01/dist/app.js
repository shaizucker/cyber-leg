import {WalkingDummy} from './dummy.js';
import * as THREE from './three.module.min.js';
import {rad,clamp,terrain,makeStones,wrapStone,stoneProfile,surfaceHeight,supportSlope,gait,feasiblePose} from './motion.js';
const $=id=>document.getElementById(id);
const state={ready:false,running:false,mode:'manual',time:0,phase:0,distance:0,knee:0,kneeVelocity:0,pitch:0,roll:0,height:0,hip:0,correctionPitch:0,correctionRoll:0,extensions:[20,20,20],limited:false};
let dummy;
let scene,camera,renderer,root,shank,foot,cad,meshes=[],actuators=[],anchors=[],contactVertices=[],axisGroup,terrainMesh;
let kneePivot=new THREE.Vector3(-7.4,387.77,.831),anklePivot=new THREE.Vector3(.435,-16.962,.831);
let stones=makeStones(crypto.getRandomValues(new Uint32Array(1))[0]),stoneMeshes=[],filteredPitch=0,filteredRoll=0;
const history=[],samples=[];let lastSample=0,lastTerrain=-1,baseGround=94.582;
const orbit={theta:1.05,phi:1.27,radius:2750,target:new THREE.Vector3(25,820,-60)};
const clock=new THREE.Clock();let drag=null,joyDrag=false,pointerDown=null;
let ankleLimits=null;
let bridgeToken=null,bridgeBusy=false,bridgeArmed=false,bridgeArmPending=false,bridgeGeneration=0,bridgeQueue=Promise.resolve();
function bridgeDisplay(data){
 const ankle=data.ankle;
 ankleLimits=ankle?.enabled?(ankle.limits_mm||[[0,39],[0,39],[0,39]]):null;
 $('ankleStatus').textContent=!ankle?.enabled?'Ankle control disabled':ankle.error?`Ankle: ${ankle.error}`:ankle.arming?'Ankle connecting and checking nodes 1/2/3...':ankle.armed?'Ankle armed: A1/A2/A3 = nodes 1/2/3':'Ankle disarmed';
 $('ankleFeedback').textContent=ankle?.positions_mm?ankle.positions_mm.map((p,i)=>`A${i+1}: ${p.toFixed(2)} mm`).join(' / '):'Ankle tilt and height move all three actuators together.';
 $('motorZero').disabled=!bridgeToken||data.armed||bridgeArmPending||state.running;
 bridgeArmed=!!data.armed;$('motorArm').disabled=!bridgeToken||bridgeArmed||bridgeArmPending||!state.ready;$('motorDisarm').disabled=!bridgeToken||(!bridgeArmed&&!bridgeArmPending);
 const gains=data.gains?` \u00b7 P-T-M P/D ${data.gains.kp_pos}/${data.gains.kd_pos}`:'';
 $('motorStatus').textContent=(data.error?`Motor: ${data.error}`:bridgeArmed?'Motor armed':'Motor disarmed')+gains;
 const f=data.feedback;
 $('motorFeedback').textContent=f?`Actual ${(f.position_rad/rad).toFixed(1)}° · ${(f.velocity_rad_s/rad).toFixed(1)}°/s · fault ${f.fault}${bridgeArmed?` · command ${(data.setpoint_rad/rad).toFixed(1)}°`:''}`:'Stop holds; reset disarms.';
}
function bridgePost(route,payload={}){
 const operation=bridgeQueue.catch(()=>{}).then(async()=>{
  const response=await fetch(`/api/${route}`,{method:'POST',headers:{'Content-Type':'application/json','X-Bridge-Token':bridgeToken},body:JSON.stringify(payload)});
  const data=await response.json();if(!response.ok)throw Error(data.error||'Bridge request failed');bridgeDisplay(data);return data;
 });bridgeQueue=operation;return operation;
}
async function bridgeDisarm(){bridgeGeneration++;if(!bridgeToken)return;try{await bridgePost('disarm');}catch(e){$('motorStatus').textContent=`Bridge: ${e.message}`;}}
async function bridgeTick(){
 if(!bridgeToken||bridgeBusy||!state.ready)return;bridgeBusy=true;
 try{await bridgePost('update',{angle_deg:state.knee,extensions_mm:[...state.extensions],velocity_deg_s:state.running&&state.mode==='auto'?state.kneeVelocity:0,running:state.running,mode:state.mode});}catch(e){bridgeArmed=false;$('motorArm').disabled=true;$('motorDisarm').disabled=true;$('motorStatus').textContent=`Bridge disconnected: ${e.message}`;bridgeToken=null;}
 finally{bridgeBusy=false;}
}
async function bridgeConnect(){
 if(location.hostname!=='127.0.0.1')return;
 try{const r=await fetch('/api/session',{cache:'no-store'});if(!r.ok)return;const data=await r.json();bridgeToken=data.token;bridgeDisplay(data);await bridgeTick();}
 catch(e){$('motorStatus').textContent='Bridge not connected';}
}
for(let i=0;i<3;i++)$('meters').insertAdjacentHTML('beforeend',`<div class="meterrow"><span>A${i+1}</span><div class="track"><i class="fill" id="bar${i}"></i></div><output id="ext${i}">20.0</output></div>`);
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
 axisGroup=new THREE.Group();root.add(axisGroup);
 const kneeAxis=new THREE.ArrowHelper(new THREE.Vector3(0,0,1),new THREE.Vector3(0,0,-70),140,0xff903d,12,7);axisGroup.add(kneeAxis);
 const ankleAxes=new THREE.AxesHelper(65);ankleAxes.position.copy(anklePivot).sub(kneePivot);axisGroup.add(ankleAxes);
 axisGroup.visible=$('joints').checked;
 $('kp').value=kneePivot.y.toFixed(2);$('ap').value=anklePivot.y.toFixed(3);
 dummy=new WalkingDummy(scene,root,kneePivot);dummy.setOpacity(+$('dummyOpacity').value/100);dummy.setVisible($('dummy').checked);
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
 const deck=new THREE.Mesh(new THREE.BoxGeometry(1160,34,485),new THREE.MeshStandardMaterial({color:0x303c46,metalness:.5,roughness:.6}));deck.position.set(0,-26,0);scene.add(deck);
 for(const z of [-253,253]){const rail=new THREE.Mesh(new THREE.BoxGeometry(1180,35,20),new THREE.MeshStandardMaterial({color:0x77838c,metalness:.7,roughness:.4}));rail.position.set(0,-20,z);scene.add(rail);}
 const geo=new THREE.PlaneGeometry(1140,464,180,52);geo.rotateX(-Math.PI/2);
 terrainMesh=new THREE.Mesh(geo,new THREE.MeshStandardMaterial({color:0x394952,roughness:.96,metalness:.1,side:THREE.DoubleSide,vertexColors:true}));scene.add(terrainMesh);
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
 stoneMeshes.forEach((mesh,i)=>{const s=stones[i],x=wrapStone(s.x-state.distance);mesh.visible=$('stones').checked&&Math.abs(x)<535;mesh.position.set(x,terrain(x,s.z,state.distance,amp),s.z);mesh.scale.set(s.rx,s.height*+$('stoneHeight').value,s.rz);});
}
function updateTerrain(){
 const amp=$('uneven').checked?+$('roughness').value:0,geo=terrainMesh.geometry,v=geo.attributes.position,c=geo.attributes.color;
 for(let i=0;i<v.count;i++){const x=v.getX(i),z=v.getZ(i);v.setY(i,terrain(x,z,state.distance,amp));const stripe=Math.floor((x+state.distance)/32)%2===0?.79:1; c.setXYZ(i,stripe,stripe,stripe);}
 v.needsUpdate=true;c.needsUpdate=true;geo.computeVertexNormals();updateStones();
}
function solvePose(dt){
 let basePitch=0,desiredPitch=0,desiredRoll=0,hip=0,knee=+$('knee').value,lift=0;
 state.correctionPitch=0;state.correctionRoll=0;
 if(state.mode==='auto'){
  const g=gait(state.phase);hip=g.hip;knee=g.knee;lift=g.lift;
  // In stance the sole tracks the ground; swing has bounded ankle dorsiflexion.
  desiredPitch=clamp(-(hip-knee),-24,24);basePitch=desiredPitch;
  const amp=$('uneven').checked?+$('roughness').value:0;
  const support=g.stance?supportSlope(100-350*g.phase,0,state.distance,amp,$('stones').checked?stones:[],+$('stoneHeight').value):{pitch:0,roll:0,left:0,right:0};
  const active=$('adaptive').checked&&g.stance;
  const alpha=dt>0?1-Math.exp(-dt/.10):1;
  filteredPitch+=((active?support.pitch:0)-filteredPitch)*alpha;
  filteredRoll+=((active?support.roll:0)-filteredRoll)*alpha;
  desiredPitch+=filteredPitch;desiredRoll=filteredRoll;
  state.supportDelta=support.left-support.right;
  $('supportState').textContent=g.stance?`Left / right support: ${support.left.toFixed(1)} / ${support.right.toFixed(1)} mm · Δ ${state.supportDelta.toFixed(1)} mm`:'Swing · no ground feedback';
  $('phaseName').textContent=g.label;$('phaseProgress').style.width=`${g.phase*100}%`;
 }else{desiredPitch=state.pitch;desiredRoll=state.roll;state.correctionPitch=desiredPitch;state.correctionRoll=desiredRoll;$('phaseName').textContent='MANUAL CONTROL';$('phaseProgress').style.width='0%';}
 const pose=feasiblePose(anchors,anklePivot.toArray(),desiredPitch,desiredRoll,+$('height').value,ankleLimits);
 state.correctionPitch=pose.pitch-basePitch;state.correctionRoll=pose.roll;state.kneeVelocity=state.running&&state.mode==='auto'&&dt>0?(knee-state.knee)/dt:0;state.knee=knee;state.hip=hip;state.extensions=pose.extensions;state.limited=pose.limited;state.appliedPitch=pose.pitch;state.appliedRoll=pose.roll;state.height=pose.height;
 root.rotation.z=hip*rad;root.position.set(kneePivot.x,kneePivot.y+baseGround,kneePivot.z);
 shank.rotation.z=-knee*rad;
 foot.position.copy(anklePivot).sub(kneePivot);foot.position.y-=pose.height;
 foot.rotation.set(pose.roll*rad,0,pose.pitch*rad,'ZXY');
 for(const a of actuators){
  const local=a.bottom.clone().sub(anklePivot).applyQuaternion(foot.quaternion).add(anklePivot);local.y-=pose.height;
  const direction=local.clone().sub(a.top).normalize();const q=new THREE.Quaternion().setFromUnitVectors(a.direction,direction);
  a.barrel.position.copy(a.top).sub(kneePivot);a.barrel.quaternion.copy(q);
  a.piston.position.copy(local).sub(kneePivot);a.piston.quaternion.copy(q);
 }
 root.updateMatrixWorld(true);
 if(state.mode==='auto'){
  // Geometric nonpenetration only: vertical mounting follows the lowest sole sample.
  const g=gait(state.phase),s=clamp((g.phase-.62)/.38,0,1);
  const targetX=g.stance?100-350*g.phase:-117+217*(s*s*(3-2*s));
  const contactCenter=new THREE.Vector3(40-anklePivot.x,-85-anklePivot.y,-anklePivot.z).applyMatrix4(foot.matrixWorld);
  root.position.x+=targetX-contactCenter.x;root.updateMatrixWorld(true);
  const amp=$('uneven').checked?+$('roughness').value:0;let shift=-Infinity;
  for(const v of contactVertices){const w=v.clone().applyMatrix4(foot.matrixWorld);shift=Math.max(shift,surfaceHeight(w.x,w.z,state.distance,amp,$('stones').checked?stones:[],+$('stoneHeight').value)-w.y);}
  root.position.y+=shift+lift;root.updateMatrixWorld(true);
 }
 if(dummy)dummy.update(state,(x,z)=>surfaceHeight(x,z,state.distance,$('uneven').checked?+$('roughness').value:0,$('stones').checked?stones:[],+$('stoneHeight').value));
 $('kneeV').textContent=`${knee.toFixed(1)}°`;$('angles').textContent=`${pose.pitch.toFixed(1)}° / ${pose.roll.toFixed(1)}°`;
 if(state.mode==='auto')$('knee').value=knee;
 pose.extensions.forEach((e,i)=>{$(`bar${i}`).style.width=`${clamp(e/50,0,1)*100}%`;$(`ext${i}`).textContent=e.toFixed(1);});
 $('limitState').textContent=pose.limited?'Travel limit reached · command reduced':'Within assumed 0–50 mm travel';$('limitState').classList.toggle('warn',pose.limited);
}
function syncControls(){
 $('runState').textContent=state.running?(state.mode==='auto'?'AUTO / RUNNING':'MANUAL / ACTIVE'):'STOPPED';$('runState').classList.toggle('running',state.running);
 $('start').disabled=!state.ready||state.running;$('stop').disabled=!state.ready||!state.running;$('reset').disabled=!state.ready;
 $('knee').disabled=!state.running||state.mode==='auto';$('height').disabled=!state.running;$('joy').classList.toggle('disabled',!state.running||state.mode!=='manual');
 $('manual').classList.toggle('selected',state.mode==='manual');$('auto').classList.toggle('selected',state.mode==='auto');
 $('modeHelp').textContent=state.mode==='auto'?'Start the walking cycle. Stop holds the current pose.':'Start to enable joint controls. Stop freezes the pose.';
}
function reset(){
 if(bridgeToken)bridgeDisarm();
 Object.assign(state,{running:false,time:0,phase:0,distance:0,knee:0,kneeVelocity:0,pitch:0,roll:0,height:0,hip:0});$('knee').value=0;$('height').value=0;$('heightV').textContent='0.0 mm';$('stick').style.transform='translate(0,0)';history.length=0;samples.length=0;lastSample=0;lastTerrain=-1;filteredPitch=0;filteredRoll=0;state.supportDelta=0;$('supportState').textContent='Left / right support: —';
 const m=state.mode;state.mode='manual';solvePose(0);state.mode=m;updateTerrain();drawChart();syncControls();$('simTime').textContent='00:00.0';
}
function setJoystick(x,y){if(!state.running||state.mode!=='manual')return;const len=Math.hypot(x,y);if(len>1){x/=len;y/=len;}state.pitch=-y*18;state.roll=x*12;$('stick').style.transform=`translate(${x*43}px,${y*43}px)`;}
function releaseJoy(){joyDrag=false;if(state.running&&state.mode==='manual')setJoystick(0,0);}
function drawChart(){
 const el=$('chart'),r=el.getBoundingClientRect(),dpr=Math.min(devicePixelRatio||1,2);el.width=Math.max(1,r.width*dpr);el.height=Math.max(1,r.height*dpr);const ctx=el.getContext('2d');ctx.scale(dpr,dpr);const w=r.width,h=r.height,pad=25;
 ctx.font='11px monospace';ctx.lineWidth=1;ctx.strokeStyle='#303c46';ctx.fillStyle='#82909b';
 for(const v of [-20,-10,0,10,20]){const y=8+(20-v)/40*(h-25);ctx.beginPath();ctx.moveTo(pad,y);ctx.lineTo(w,y);ctx.stroke();if(v%20===0)ctx.fillText(String(v),0,y+4);}
 for(const [key,color] of [['pitch','#ff903d'],['roll','#6bd4e5']]){ctx.strokeStyle=color;ctx.lineWidth=1.8;ctx.beginPath();let first=true;for(const s of history){const x=w-(state.time-s.time)/10*(w-pad),y=8+(20-s[key])/40*(h-25);if(x<pad)continue;first?ctx.moveTo(x,y):ctx.lineTo(x,y);first=false;}ctx.stroke();}
 ctx.fillStyle='#82909b';ctx.fillText('−10 s',pad,h-1);ctx.fillText('now',w-25,h-1);
}
function animate(){
 requestAnimationFrame(animate);const dt=Math.min(clock.getDelta(),.05);
 if(state.ready&&state.running){state.time+=dt;if(state.mode==='auto'){const speed=+$('speed').value;state.distance+=speed*1000*dt;state.phase=(state.phase+speed/.35*dt)%1;}solvePose(dt);
  if(state.time-lastTerrain>1/30){updateTerrain();lastTerrain=state.time;}
  if(state.time-lastSample>=.05){lastSample=state.time;const row={time:state.time,pitch:state.correctionPitch,roll:state.correctionRoll,knee:state.knee,anklePitch:state.appliedPitch,ankleRoll:state.appliedRoll,height:state.height,supportDelta:state.supportDelta||0,a1:state.extensions[0],a2:state.extensions[1],a3:state.extensions[2],limited:state.limited?1:0};history.push(row);samples.push(row);while(history.length&&history[0].time<state.time-10)history.shift();if(samples.length>72000)samples.shift();drawChart();}
  $('simTime').textContent=`${String(Math.floor(state.time/60)).padStart(2,'0')}:${(state.time%60).toFixed(1).padStart(4,'0')}`;
 }
 if(renderer)renderer.render(scene,camera);
}
function events(){
 $('motorZero').onclick=async()=>{if(!bridgeToken||bridgeArmed||state.running)return;$('motorZero').disabled=true;try{await bridgePost('update',{angle_deg:state.knee,running:false,mode:state.mode});await bridgePost('zero');}catch(e){$('motorStatus').textContent=`Motor: ${e.message}`;}};
 $('motorArm').onclick=async()=>{if(!bridgeToken||!state.ready||bridgeArmPending)return;const generation=bridgeGeneration;bridgeArmPending=true;$('motorArm').disabled=true;$('motorDisarm').disabled=false;try{await bridgePost('update',{angle_deg:state.knee,extensions_mm:[...state.extensions],velocity_deg_s:state.running&&state.mode==='auto'?state.kneeVelocity:0,running:state.running,mode:state.mode});if(generation===bridgeGeneration)await bridgePost('arm');}catch(e){$('motorStatus').textContent=`Motor: ${e.message}`;}finally{bridgeArmPending=false;$('motorArm').disabled=!bridgeToken||bridgeArmed;$('motorDisarm').disabled=!bridgeToken||!bridgeArmed;}};
 $('motorDisarm').onclick=bridgeDisarm;
 $('start').onclick=()=>{state.running=true;clock.getDelta();syncControls();};$('stop').onclick=()=>{state.running=false;joyDrag=false;syncControls();};$('reset').onclick=reset;
 for(const mode of ['manual','auto'])$(mode).onclick=()=>{state.mode=mode;reset();};
 $('knee').oninput=()=>{$('kneeV').textContent=`${(+$('knee').value).toFixed(1)}°`;};$('height').oninput=()=>{$('heightV').textContent=`${(+$('height').value).toFixed(1)} mm`;};
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
 $('export').onclick=()=>{if(!samples.length){alert('Start the simulator to record telemetry first.');return;}const keys=Object.keys(samples[0]),csv=[keys.join(','),...samples.map(s=>keys.map(k=>Number(s[k]).toFixed(4)).join(','))].join('\n');const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([csv],{type:'text/csv'}));a.download='CYBER_LEG_telemetry.csv';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);};
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
