// Activity generator for the CYBER LEG simulator: level walking, stairs,
// sit-to-stand and stumble recovery. Pure module (no DOM, no Three.js) so the
// same code runs in the browser and in `node scripts/check_gait.mjs`.
//
// Design
// - Joint motion is defined in ANATOMICAL angles (thigh from vertical, knee
//   flexion, ankle dorsiflexion) by periodic C2 cubic splines through
//   normative keyframes, then converted to the rig's command angles using the
//   CAD's built-in thigh/shank lean. Every curve is C2 around the cycle, so
//   commanded joint speed never jumps.
// - The pelvis (hip mount) is NOT fixed. Horizontally it moves so that the
//   loaded foot's contact point travels exactly with the ground (belt or
//   moving staircase); the two legs hand over with C2 weights across double
//   support. Vertically it rests on whichever foot needs it highest (smooth
//   maximum over the real CAD sole vertices), so uneven ground and stones lift
//   the body instead of the foot sinking in. Stairs integrate the vertical
//   motion the same way as the horizontal one.
// - Cadence follows from the curves: the stride is the distance the loaded
//   foot travels relative to the pelvis per cycle, so cadence = speed / stride
//   keeps the planted foot locked to the belt at every speed.
// Units: mm, degrees, seconds. +X forward, +Y up. Phase 0 = initial contact.
import {rad,clamp,smooth5,periodicSpline,rotateFoot,feasiblePose} from './motion.js';

// Normative-style keyframes [phase, degrees]. Hip = thigh angle from vertical
// (flexion +), knee = flexion, ankle = dorsiflexion (+) / plantarflexion (−).
export const PROFILES={
 walk:{label:'Level walking',stance:.62,ds:.12,
  keys:{hip:[[0,21],[.10,19],[.30,4],[.50,-10],[.58,-7],[.68,6],[.82,22],[.92,23]],
   knee:[[0,4],[.13,16],[.36,5],[.48,9],[.62,37],[.72,58],[.85,30],[.95,5]],
   // Push-off is deliberately soft (−12°): a rigid prosthetic sole otherwise
   // lifts the pelvis sharply at toe-off.
   ankle:[[0,0],[.06,-5],[.20,3],[.42,9],[.52,5],[.62,-12],[.70,-6],[.80,1],[.92,1]]}},
 // Stair keyframes start from published ascent/descent patterns and were then
 // adjusted (scripts/check_activities.mjs) for ≥ 20 mm toe clearance over the
 // nosing with this CAD foot, which has no toe joint.
 stairsUp:{label:'Stair ascent',stance:.62,ds:.12,
  keys:{hip:[[0,58],[.12,45],[.32,25],[.50,11],[.60,9],[.70,24],[.82,52],[.92,65]],
   knee:[[0,62],[.10,52],[.30,32],[.48,11],[.58,12],[.68,61],[.78,94],[.88,94],[.95,71]],
   ankle:[[0,8],[.10,13],[.30,12],[.48,-8],[.60,-12],[.70,-1],[.80,9],[.90,9]]}},
 stairsDown:{label:'Stair descent',stance:.62,ds:.12,
  keys:{hip:[[0,18],[.15,20],[.38,12],[.56,40],[.66,40],[.78,51],[.90,26]],
   knee:[[0,13],[.15,24],[.38,55],[.56,78],[.66,103],[.78,93],[.88,61],[.96,10]],
   ankle:[[0,-16],[.10,-9],[.35,9],[.52,18],[.64,-7],[.78,-4],[.90,-14]]}}
};
export const ACTIVITY_LABELS={walk:'Level walking',stairsUp:'Stair ascent',stairsDown:'Stair descent',sit:'Sit / stand'};
export const DEFAULT_PARAMS={clearance:0,pushOff:1,stanceKnee:1,stairCadence:70,sitHold:2,autoStumble:false};

const splines={};
function compiled(name){
 if(!splines[name]){const k=PROFILES[name].keys;splines[name]={hip:periodicSpline(k.hip),knee:periodicSpline(k.knee),ankle:periodicSpline(k.ankle)};}
 return splines[name];
}
// Replace keyframes at runtime (tuning tools); clears compiled splines and strides.
export function setProfileKeys(name,keys){Object.assign(PROFILES[name].keys,keys);delete splines[name];strideCache.clear();}
const wrap=p=>((p%1)+1)%1;
// C2 window: 1 inside [a,b], ramps over r at both ends.
export const window2=(x,a,b,r)=>smooth5((x-a)/r)*smooth5((b-x)/r);
// C2 bump on [0,1], peak 1 at 3/8: used for swing clearance.
const SWING_NORM=(3/8)**3*(5/8)**5;
export function swingBump(phase,stance){
 const p=wrap(phase);if(p<stance)return 0;const s=(p-stance)/(1-stance);return s**3*(1-s)**5/SWING_NORM;
}
// Symmetric C2 bump on [0,1] with peak 1 at 0.5.
export const bump=s=>s<=0||s>=1?0:64*s**3*(1-s)**3;

// Anatomical angles of one leg at a phase, with speed scaling and tuning.
export function profileAngles(name,phase,params=DEFAULT_PARAMS,speed=1){
 const p=wrap(phase),sp=compiled(name),P=PROFILES[name];
 let H=sp.hip(p)[0],K=sp.knee(p)[0],A=sp.ankle(p)[0];
 if(name==='walk'){
  // Slower walking uses smaller excursions; knee swing is kept for clearance.
  const r=clamp(speed/.9,0,1);
  H=6+(H-6)*(.45+.55*r);K=4+(K-4)*(.86+.14*r);A*=.75+.25*r;
  K=4+(K-4)*(1+(params.stanceKnee-1)*window2(p,0,.42,.08));
  A*=1+(params.pushOff-1)*window2(p,.48,.80,.06);
 }
 K+=(params.clearance||0)*swingBump(p,P.stance);
 return {H,K,A};
}

// Rig geometry: u = knee->hip (thigh frame), w = knee->ankle (shank frame).
export function legGeometry(rig){
 const [ux,uy]=rig.u,[wx,wy]=rig.w;
 return {at:Math.atan2(-ux,uy)/rad,as:Math.atan2(wx,-wy)/rad,thigh:Math.hypot(ux,uy),shank:Math.hypot(wx,wy)};
}
export const toJoint=(a,g)=>({hip:a.H-g.at,knee:a.K-g.at+g.as,ankle:a.A+g.as});
export const toAnatomical=(j,g)=>({H:j.hip+g.at,K:j.knee+g.at-g.as,A:j.ankle-g.as});

// Planar forward kinematics relative to the hip joint (pelvis).
export function legPose(rig,j,roll=0,height=0,zOffset=0){
 const h=j.hip*rad,q=(j.hip-j.knee)*rad,[ux,uy]=rig.u,[wx,w0]=rig.w,wy=w0-height;
 const kx=-(Math.cos(h)*ux-Math.sin(h)*uy),ky=-(Math.sin(h)*ux+Math.cos(h)*uy);
 return {knee:[kx,ky],ankle:[kx+Math.cos(q)*wx-Math.sin(q)*wy,ky+Math.sin(q)*wx+Math.cos(q)*wy],
  pitch:j.hip-j.knee+j.ankle,roll,z:rig.z+zOffset};
}
export function pointOf(leg,c){const r=rotateFoot(c,leg.pitch,leg.roll);return [leg.ankle[0]+r[0],leg.ankle[1]+r[1],leg.z+r[2]];}

// Pelvis height needed for this foot to touch the surface, plus the material
// sole point carrying the load (soft weighting over the CAD sole vertices).
const CONTACT_T=.6,CONTACT_REF=Math.log(8);
export function footContact(leg,contact,px,surface){
 const req=new Float64Array(contact.length);let m=-Infinity;
 for(let i=0;i<contact.length;i++){const p=pointOf(leg,contact[i]);req[i]=surface(px+p[0],p[2])-p[1];if(req[i]>m)m=req[i];}
 let sum=0;const c=[0,0,0];
 for(let i=0;i<contact.length;i++){const e=Math.exp((req[i]-m)/CONTACT_T);sum+=e;c[0]+=e*contact[i][0];c[1]+=e*contact[i][1];c[2]+=e*contact[i][2];}
 return {req:m+CONTACT_T*(Math.log(sum)-CONTACT_REF),c:c.map(v=>v/sum)};
}
// Smooth maximum, exact to within eps.
export const smoothMax=(a,b,eps=1)=>(a+b)/2+Math.sqrt(((a-b)/2)**2+eps*eps)-eps;

// Which leg drives the pelvis. The prosthetic foot is the reference for its
// whole stance, so it never slips; the sound leg takes over while the
// prosthesis swings. Handovers are short C2 ramps placed while the prosthetic
// foot is in the air (just before landing, just after toe-off). The sound
// foot is held on the ground separately (see soundFoot), so double support
// has no visible slip on either side.
const HANDOVER=.02;
export function loadShare(phase,P){
 const p=wrap(phase),a=HANDOVER,off=P.stance-a;
 if(p<a)return smooth5((p+a)/(2*a));
 if(p<off)return 1;
 if(p<off+2*a)return 1-smooth5((p-off)/(2*a));
 if(p>1-a)return smooth5((p-1+a)/(2*a));
 return 0;
}
// Stride (mm per cycle, x and y) the loaded foot travels relative to the pelvis.
const strideCache=new Map();
export function strideOf(name,rig,params,speed){
 speed=Math.round(speed*100)/100;
 const key=`${name}|${speed.toFixed(3)}|${params.clearance}|${params.pushOff}|${params.stanceKnee}|${rig.key||''}`;
 if(strideCache.has(key))return strideCache.get(key);
 const g=legGeometry(rig),P=PROFILES[name],N=240,d=1e-3,flat=()=>0;let sx=0,sy=0;
 for(let i=0;i<N;i++){
  const p=(i+.5)/N,w=loadShare(p,P);
  for(const [phase,share] of [[p,w],[p+.5,1-w]]){
   if(share<=0)continue;
   const j=toJoint(profileAngles(name,phase,params,speed),g),c=footContact(legPose(rig,j),rig.contact,0,flat).c;
   const a=pointOf(legPose(rig,toJoint(profileAngles(name,phase+d,params,speed),g)),c);
   const b=pointOf(legPose(rig,toJoint(profileAngles(name,phase-d,params,speed),g)),c);
   sx+=share*(a[0]-b[0])/(2*d)/N;sy+=share*(a[1]-b[1])/(2*d)/N;
  }
 }
 const result={x:-sx,y:-sy};
 if(strideCache.size>200)strideCache.clear();
 strideCache.set(key,result);return result;
}

// Sit-to-stand profile over s = 0 (standing) .. 1 (seated). Feet stay flat,
// so the ankle follows the shank; K−H stays inside the ankle platform range.
// Standing keeps 3° of knee flexion: the CAD knee at command 0° is about 2° hyperextended anatomically.
const SIT_KEYS={H:[[0,1],[.3,25],[.6,60],[1,88]],K:[[0,3],[.3,38],[.6,75],[1,95]]};
function monotone(keys,x){
 const n=keys.length;if(x<=keys[0][0])return keys[0][1];if(x>=keys[n-1][0])return keys[n-1][1];
 let i=0;while(x>keys[i+1][0])i++;
 const slope=k=>{if(k===0||k===n-1)return 0;const a=(keys[k][1]-keys[k-1][1])/(keys[k][0]-keys[k-1][0]),b=(keys[k+1][1]-keys[k][1])/(keys[k+1][0]-keys[k][0]);return a*b<=0?0:2/(1/a+1/b);};
 const [x0,y0]=keys[i],[x1,y1]=keys[i+1],h=x1-x0,t=(x-x0)/h,m0=slope(i)*h,m1=slope(i+1)*h;
 return (2*t**3-3*t*t+1)*y0+(t**3-2*t*t+t)*m0+(-2*t**3+3*t*t)*y1+(t**3-t*t)*m1;
}
export function sitAngles(s){return {H:monotone(SIT_KEYS.H,s),K:monotone(SIT_KEYS.K,s),A:0};}

const STUMBLE={
 // Early/mid swing: lift the foot over the obstacle.
 elevating:{dur:.32,H:10,K:28,A:6},
 // Late swing: put the foot down early with a shorter, straighter step.
 lowering:{dur:.22,H:-7,K:0,A:4,warp:1.6}
};

export class GaitEngine{
 constructor(rig){this.params={...DEFAULT_PARAMS};this.setRig(rig);this.reset('walk');}
 setRig(rig){this.rig=rig;this.geo=legGeometry(rig);this.soundZ=-170;}
 reset(activity=this.activity,env={}){
  Object.assign(this,{groundVel:[0,0],activity,phase:0,time:0,startTime:0,cycles:0,speed:env.speed??.3,groundX:0,escalator:[0,0],
   events:[],obstacles:[],warp:1,pendingTrip:null,stumbles:0,lastAutoCycle:-1});
  this.sit={stage:'stand',t:0,s:0};this.sf=null;this.soundLanding=null;
  const flat=()=>0;
  if(activity==='sit'){
   const j=this._sitJoint(0),c=footContact(legPose(this.rig,j),this.rig.contact,0,flat).c,p=pointOf(legPose(this.rig,j),c);
   this.sitContact=c;this.pelvis=[-p[0],-p[1]];this.home=[...this.pelvis];this.ground=[0,0];
   const j1=this._sitJoint(1),p1=pointOf(legPose(this.rig,j1),c);this.seated=[-p1[0],-p1[1]];
  }else{
   const our=this._legs(0,0),cs=[our.our,our.sound].map(l=>footContact(l.leg,this.rig.contact,0,flat));
   const po=pointOf(our.our.leg,cs[0].c),ps=pointOf(our.sound.leg,cs[1].c);
   this.pelvis=[-(po[0]+ps[0])/2,-Math.min(po[1],ps[1])];
   if(activity!=='walk')this.pelvis=[-(po[0]+ps[0])/2,-po[1]];
   this.home=[...this.pelvis];
   if(activity!=='walk'&&!env.probe)this.anchor=this._stairAnchor();
  }
  return this;
 }
 // Centres of the planted soles in mid-stance, in the moving-staircase frame.
 // Each foot's treads repeat every stride; the two sequences interleave.
 _stairAnchor(){
  const probe=new GaitEngine(this.rig);probe.params={...this.params};probe.reset(this.activity,{probe:true});
  const env={running:true,height:0},dt=1/120,centre=(ankle,pitch)=>{
   const pts=this.rig.contact.map(c=>{const r=rotateFoot(c,pitch,0);return [ankle[0]+r[0],ankle[1]+r[1]];});
   return [pts.reduce((a,q)=>a+q[0],0)/pts.length,Math.min(...pts.map(q=>q[1]))];
  };
  let our=null,sound=null;
  for(let i=0;i<6000&&!(our&&sound);i++){
   const p=probe.step(dt,env),e=p.escalator;
   if(probe.cycles>=1&&!our&&p.phase>=.3){const c=centre([p.pelvis[0]+p.our.leg.ankle[0],p.pelvis[1]+p.our.leg.ankle[1]],p.our.leg.pitch);our=[c[0]-e[0],c[1]-e[1]];}
   if(probe.cycles>=1&&!sound&&p.phase>=.8){const c=centre(p.sound.ankleWorld,p.sound.footPitch);sound=[c[0]-e[0],c[1]-e[1]];}
  }
  our=our||[0,0];sound=sound||[0,0];
  // Place the sound foot exactly midway between the prosthetic treads so the
  // staircase is regular; its swing blends into this landing offset.
  const D=probe.stride(),mid=[our[0]+D.x/2,our[1]+D.y/2];
  let k=Math.round((sound[0]-mid[0])/D.x);mid[0]+=k*D.x;mid[1]+=k*D.y;
  this.soundLanding=[mid[0]-sound[0],mid[1]-sound[1]];
  return {our,sound:mid};
 }
 // Stair treads (world coordinates) around the feet: [{x0,x1,y,cx}], sorted.
 treads(count=7){
  if(!this.anchor)return [];
  const D=this.stride(),e=this.escalator,list=[];
  const k0=Math.round(-(this.anchor.our[0]+e[0])/Math.max(D.x,1));
  for(let k=k0-count;k<=k0+count;k++)for(const a of [this.anchor.our,this.anchor.sound])list.push({cx:a[0]+k*D.x+e[0],y:a[1]+k*D.y+e[1]});
  list.sort((a,b)=>a.cx-b.cx);
  list.forEach((t,i)=>{t.x0=i?(list[i-1].cx+t.cx)/2:t.cx-D.x/4;t.x1=i<list.length-1?(t.cx+list[i+1].cx)/2:t.cx+D.x/4;});
  return list;
 }
 profile(){return PROFILES[this.activity]||PROFILES.walk;} profile(){return PROFILES[this.activity]||PROFILES.walk;}
 stride(){return strideOf(this.activity,this.rig,this.params,this.activity==='walk'?this.speed:1);}
 // Cycles per second at full ramp.
 cadence(){
  if(this.activity==='walk'){const D=this.stride().x;return D>1?this.speed*1000/D:0;}
  return this.params.stairCadence/120;
 }
 _sitJoint(s){const a=sitAngles(s),g=this.geo,j=toJoint(a,g);j.ankle=j.knee-j.hip;return j;}
 // Our leg includes stumble perturbations; sound leg follows the same curves half a cycle later.
 _anatomical(phase,time){
  const a=profileAngles(this.activity,phase,this.params,this.speed);
  for(const e of this.events){const b=bump((time-e.t0)/e.dur);a.H+=b*e.H;a.K+=b*e.K;a.A+=b*e.A;}
  return a;
 }
 _legs(phase,time,ankleExtra=null){
  const g=this.geo,our=toJoint(this._anatomical(phase,time),g),sound=toJoint(profileAngles(this.activity,phase+.5,this.params,this.speed),g);
  if(ankleExtra){our.ankle+=ankleExtra.pitch;}
  return {our:{j:our,leg:legPose(this.rig,our,ankleExtra?.roll||0,ankleExtra?.height||0)},sound:{j:sound,leg:legPose(this.rig,sound,0,0,this.soundZ)}};
 }
 trip(){
  if(this.activity!=='walk')return false;
  const P=this.profile(),p=this.phase;
  if(p<P.stance)this.pendingTrip=P.stance+.08;// trip early in the next swing
  else this._startTrip();
  return true;
 }
 _startTrip(){
  const P=this.profile(),s=(this.phase-P.stance)/(1-P.stance),type=s<.45?'elevating':'lowering',S=STUMBLE[type];
  this.events.push({type,t0:this.time,dur:S.dur,H:S.H,K:S.K,A:S.A});
  if(S.warp)this.warp=S.warp;
  this.stumbles++;this.lastTrip={type,time:this.time};
  // Obstacle drawn at the toe, fixed to the belt.
  const legs=this._legs(this.phase,this.time),toe=Math.max(...this.rig.contact.map(c=>pointOf(legs.our.leg,c)[0]));
  this.obstacles.push({belt:this.pelvis[0]+toe+12+this.groundX,z:this.rig.z,type});
 }
 // Advance by dt seconds. env: {speed, surface(x,z,d), support(x,z,d)|null, height, ankleLimits, fixedMount, running}
 step(dt,env){
  const P=this.profile();
  if(env.running&&dt>0){
   this.time+=dt;
   const ramp=smooth5((this.time-this.startTime)/1.5);
   if(this.activity==='walk')this.speed+=(env.speed-this.speed)*-Math.expm1(-dt/.8);
   this.ramp=ramp;
   if(this.activity==='sit')this._advanceSit(dt);
   else{
    const f=this.cadence()*ramp,previous=this.phase;
    this.phase=this.phase+f*this.warp*dt;
    if(this.pendingTrip!==null&&previous<this.pendingTrip&&this.phase>=this.pendingTrip){this.pendingTrip=null;this._startTrip();}
    if(this.params.autoStumble&&this.activity==='walk'&&previous<.66&&this.phase>=.66&&this.cycles!==this.lastAutoCycle){
     this.lastAutoCycle=this.cycles;if(Math.random()<.35){this.phase=Math.min(this.phase,.999);this._startTrip();}
    }
    if(this.phase>=1){this.phase-=1;this.cycles++;this.warp=1;}
    // Ground velocity. Slow drift of the body is corrected by nudging the
    // ground (belt or staircase), never the body, so planted feet do not slip.
    const D=this.stride(),k=.25,walk=this.activity==='walk';
    this.groundVel=walk?[-this.speed*ramp*1000+k*(this.pelvis[0]-this.home[0]),0]
     :[-D.x*f+k*(this.pelvis[0]-this.home[0]),-D.y*f+k*(this.pelvis[1]-this.home[1])];
    if(walk)this.groundX-=this.groundVel[0]*dt;
    else{this.escalator[0]+=this.groundVel[0]*dt;this.escalator[1]+=this.groundVel[1]*dt;}
   }
   this.events=this.events.filter(e=>this.time-e.t0<e.dur);
   this.obstacles=this.obstacles.filter(o=>o.belt-this.groundX>-900);
  }
  return this.solve(dt,env);
 }
 _advanceSit(dt){
  const S=this.sit,len={stand:1.2,down:2.6,seated:this.params.sitHold,up:2.2},next={stand:'down',down:'seated',seated:'up',up:'stand'};
  S.t+=dt;if(S.t>=len[S.stage]){S.t-=len[S.stage];S.stage=next[S.stage];if(S.stage==='stand')this.cycles++;}
  const u=clamp(S.t/len[S.stage],0,1);
  S.s=S.stage==='stand'?0:S.stage==='seated'?1:S.stage==='down'?smooth5(u):1-smooth5(u);
  S.u=u;
 }
 _sitTrunk(){const S=this.sit;if(S.stage==='down')return 8*S.s+30*bump(S.u);if(S.stage==='up')return 8*S.s+36*bump(Math.min(1,S.u*1.25));return 8*S.s;}
 _ankleTarget(env,legs,stance){
  // Adaptive ankle (walking): follow the support slope in stance, pre-shape late swing.
  if(this.activity!=='walk'||!env.support)return {pitch:0,roll:0,weight:0,support:null};
  const P=this.profile(),p=this.phase,weight=p<P.stance?1-smooth5((p-.44)/.16):smooth5(((p-P.stance)/(1-P.stance)-.55)/.45);
  const x=this.pelvis[0]+legs.our.leg.ankle[0]+40,d=p<P.stance?this.groundX:this.groundX+(1-p)*this.stride().x;
  const landX=p<P.stance?x:this.pelvis[0]+this._landingAnkleX()+40;
  const sup=env.support(landX,this.rig.z,d);
  return {pitch:weight*sup.pitch,roll:weight*sup.roll,weight,support:sup};
 }
 _landingAnkleX(){const j=toJoint(profileAngles(this.activity,0,this.params,this.speed),this.geo);return legPose(this.rig,j).ankle[0];}
 solve(dt,env){
  const P=this.profile(),flat=()=>0,limits=env.ankleLimits||null,height=env.height||0;
  let legs,ankle={pitch:0,roll:0,weight:0,support:null},trunk=0,stance,label,progress;
  if(this.activity==='sit'){
   const j=this._sitJoint(this.sit.s);
   legs={our:{j:{...j}},sound:{j:{...j}}};trunk=this._sitTrunk();stance=true;
   label={stand:'STANDING',down:'SITTING DOWN',seated:'SEATED',up:'STANDING UP'}[this.sit.stage];
   progress=this.sit.stage==='down'?this.sit.s*.5:this.sit.stage==='seated'?.5:this.sit.stage==='up'?.5+(1-this.sit.s)*.5:0;
  }else{
   legs=this._legs(this.phase,this.time);stance=this.phase<P.stance;
   ankle=this._ankleTarget(env,legs,stance);
   label=phaseLabel(this.activity,this.phase,P);progress=this.phase;
  }
  // Ankle feasibility on the three-actuator platform; the foot uses what it can reach.
  const wanted=legs.our.j.ankle+ankle.pitch,pose=feasiblePose(this.rig.anchors,this.rig.pivot,wanted,ankle.roll,height,limits);
  const ourJ={...legs.our.j,ankle:pose.pitch},our=legPose(this.rig,ourJ,pose.roll,pose.height),sound=legPose(this.rig,legs.sound.j,0,0,this.soundZ);
  if(env.fixedMount){
   this.pelvis=[...this.home];
  }else if(this.activity==='sit'){
   const p=pointOf(our,this.sitContact);this.pelvis=[this.ground[0]-p[0],this.ground[1]-p[1]];
  }else if(env.running&&dt>0){
   // Horizontal: the loaded foot's contact point moves with the ground.
   const w=loadShare(this.phase,P),surface=this._surface(env),cs=[footContact(our,this.rig.contact,this.pelvis[0],surface),footContact(sound,this.rig.contact,this.pelvis[0],surface)];
   const v=[this._pointVelocity(cs[0].c,true,pose.pitch-legs.our.j.ankle,pose),this._pointVelocity(cs[1].c,false)];
   const ground=this.groundVel||[0,0];
   this.pelvis[0]+=(ground[0]-(w*v[0][0]+(1-w)*v[1][0]))*dt;
   if(this.activity!=='walk')this.pelvis[1]+=(ground[1]-(w*v[0][1]+(1-w)*v[1][1]))*dt;
  }
  if(this.activity==='walk'&&!env.fixedMount){
   const surface=this._surface(env);
   this.pelvis[1]=smoothMax(footContact(our,this.rig.contact,this.pelvis[0],surface).req,footContact(sound,this.rig.contact,this.pelvis[0],surface).req);
  }
  const soundAnkle=this._soundFoot(dt,env,sound);
  const rates=this._rates(env);
  const D=this.activity==='sit'?{x:0,y:0}:this.stride();
  return {activity:this.activity,label,progress,phase:this.phase,stance,trunk,cycles:this.cycles,nominalAnkle:legs.our.j.ankle,
   pelvis:[...this.pelvis],target:{hip:legs.our.j.hip,knee:legs.our.j.knee,ankle:wanted},
   our:{hip:ourJ.hip,knee:ourJ.knee,ankle:pose.pitch,roll:pose.roll,height:pose.height,limited:pose.limited,extensions:pose.extensions,leg:our},
   sound:{...legs.sound.j,leg:sound,ankleWorld:[soundAnkle[0],soundAnkle[1],sound.z],footPitch:sound.pitch},
   rates,support:ankle.support,adaptWeight:ankle.weight,basePitch:legs.our.j.hip-legs.our.j.knee+legs.our.j.ankle,
   groundX:this.groundX,escalator:[...this.escalator],stride:D,cadence:this.activity==='sit'?0:this.cadence()*60,
   stairs:this.activity.startsWith('stairs')?{run:D.x/2,rise:D.y/2,treads:this.treads()}:null,
   chair:this.activity==='sit'?{seatX:this.seated[0],seatY:this.seated[1]-92}:null,
   obstacles:this.obstacles.map(o=>({x:o.belt-this.groundX,z:o.z,type:o.type})),lastTrip:this.lastTrip,stumbles:this.stumbles};
 }
 // Displayed sound foot: rolls on the ground without slipping during its
 // stance, and blends from where it lifted off to the curve's landing in swing.
 _soundFoot(dt,env,sound){
  const P=this.profile(),curve=[this.pelvis[0]+sound.ankle[0],this.pelvis[1]+sound.ankle[1]];
  if(this.activity==='sit'||env.fixedMount){this.sf=null;return curve;}
  const sp=wrap(this.phase+.5),stance=sp<P.stance;
  if(!this.sf){this.sf={ankle:curve,stance,pitch:sound.pitch,lift:[0,0]};return curve;}
  const sf=this.sf;
  if(env.running&&dt>0){
   if(stance){
    sf.stance=true;
    const c=footContact(sound,this.rig.contact,0,()=>0).c,r1=rotateFoot(c,sound.pitch,0),r0=rotateFoot(c,sf.pitch,0);
    const g=this.groundVel||[0,0];
    sf.ankle=[sf.ankle[0]-(r1[0]-r0[0])+g[0]*dt,sf.ankle[1]-(r1[1]-r0[1])+g[1]*dt];
   }else{
    if(sf.stance){sf.stance=false;sf.lift=[sf.ankle[0]-curve[0],sf.ankle[1]-curve[1]];}
    const b=1-smooth5((sp-P.stance)/(1-P.stance)),land=this.soundLanding||[0,0];
    sf.ankle=[curve[0]+sf.lift[0]*b+land[0]*(1-b),curve[1]+sf.lift[1]*b+land[1]*(1-b)];
   }
  }
  sf.pitch=sound.pitch;
  return sf.ankle;
 }
 _surface(env){if(this.activity!=='walk'||!env.surface)return ()=>0;const d=this.groundX;return (x,z)=>env.surface(x,z,d);}
 // Velocity (mm/s, pelvis frame) of a fixed sole point, from the time derivative of the joint curves.
 // ankleOffset = adaptive/limited ankle correction, held constant over the derivative.
 _pointVelocity(c,ourLeg,ankleOffset=0,pose=null){
  const h=.004,at=tau=>{
   if(this.activity==='sit')return null;
   const phase=this.phase+this.cadence()*(this.ramp??1)*this.warp*tau,t=this.time+tau;
   const legs=this._legs(phase,t);
   const j=ourLeg?{...legs.our.j,ankle:legs.our.j.ankle+ankleOffset}:legs.sound.j;
   return pointOf(legPose(this.rig,j,ourLeg&&pose?pose.roll:0,ourLeg&&pose?pose.height:0,ourLeg?0:this.soundZ),c);
  };
  const a=at(h),b=at(-h);return a?[(a[0]-b[0])/(2*h),(a[1]-b[1])/(2*h)]:[0,0];
 }
 // Command-angle rates (deg/s) of the prosthetic hip and knee, analytic in time.
 _rates(env){
  if(!env.running)return {hip:0,knee:0};
  const h=.004;let a,b;
  if(this.activity==='sit'){
   const S=this.sit;if(S.stage==='stand'||S.stage==='seated')return {hip:0,knee:0};
   const len=S.stage==='down'?2.6:2.2,s=u=>S.stage==='down'?smooth5(u):1-smooth5(u);
   a=this._sitJoint(s(clamp(S.u+h/len,0,1)));b=this._sitJoint(s(clamp(S.u-h/len,0,1)));
  }else{
   const f=this.cadence()*(this.ramp??1)*this.warp;
   a=toJoint(this._anatomical(this.phase+f*h,this.time+h),this.geo);b=toJoint(this._anatomical(this.phase-f*h,this.time-h),this.geo);
  }
  return {hip:(a.hip-b.hip)/(2*h),knee:(a.knee-b.knee)/(2*h)};
 }
}

export function phaseLabel(activity,p,P){
 if(activity==='walk')return p<.07?'HEEL CONTACT':p<.45?'STANCE':p<P.stance?'PUSH-OFF':'SWING';
 if(activity==='stairsUp')return p<.12?'WEIGHT ACCEPTANCE':p<.4?'PULL-UP':p<P.stance?'FORWARD CONTINUANCE':p<.8?'FOOT CLEARANCE':'FOOT PLACEMENT';
 if(activity==='stairsDown')return p<.12?'WEIGHT ACCEPTANCE':p<.4?'FORWARD CONTINUANCE':p<P.stance?'CONTROLLED LOWERING':p<.8?'LEG PULL-THROUGH':'FOOT PLACEMENT';
 return '';
}
