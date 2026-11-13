// Builds the simulator rig from the preconverted CAD (same construction as
// dist/app.js buildRig) and a headless runner for dist/gait.js. Used by
// check_activities.mjs and tune_profile.mjs.
import {readFileSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {dirname,join} from 'node:path';
import {GaitEngine,pointOf,footContact} from '../dist/gait.js';
import {rotateFoot} from '../dist/motion.js';
const dist=join(dirname(fileURLToPath(import.meta.url)),'..','dist');
const cad=JSON.parse(readFileSync(join(dist,'cad.json'),'utf8')),bin=readFileSync(join(dist,'cad.bin'));
const buffer=bin.buffer.slice(bin.byteOffset,bin.byteOffset+bin.byteLength);

// Same rig construction as app.js buildRig().
const kneePivot=[-7.4,387.77,.831],anklePivot=[.435,-16.962,.831],HIP_LOCAL=[-22,395];
const center=p=>p.min.map((v,i)=>(v+p.max[i])/2);
const anchors=[];
for(let a=0;a<3;a++){const first=1+a*28,top=center(cad.parts[first+3]),bottom=center(cad.parts[first+27]);anchors.push({top,bottom,length:Math.hypot(...bottom.map((v,i)=>v-top[i]))});}
let contactAll=[];
for(const p of cad.parts){
 if(!(p.id===86||p.path.includes('completed foot assembly')))continue;
 if(!['heel final','base plate','CARRAIGE BOLT_M8x20-CAR-ZP'].includes(p.name))continue;
 const v=new Float32Array(buffer,p.vertexOffset,p.vertexCount*3);
 for(let i=0;i<v.length;i+=3)if(v[i+1]<-75)contactAll.push([v[i]-anklePivot[0],v[i+1]-anklePivot[1],v[i+2]-anklePivot[2]]);
}
const cells=new Map();for(const v of contactAll){const key=`${Math.round(v[0]/5)},${Math.round(v[2]/5)}`;if(!cells.has(key)||cells.get(key)[1]>v[1])cells.set(key,v);}
export const contact=[...cells.values()];
export const rig={u:HIP_LOCAL,w:[anklePivot[0]-kneePivot[0],anklePivot[1]-kneePivot[1]],contact,anchors,pivot:anklePivot,limits:[[0,39],[0,39],[0,39]],z:anklePivot[2]};


export function run(activity,{speed=.3,seconds=8,dt=1/120,surface=null,support=null,params={},trip=null}={}){
 const e=new GaitEngine(rig);Object.assign(e.params,params);e.reset(activity,{speed});
 const env={running:true,speed,surface:surface||((x,z,d)=>0),support,height:0,ankleLimits:rig.limits};
 const rows=[];let prev=null;
 for(let t=0;t<seconds;t+=dt){
  if(trip!==null&&Math.abs(t-trip)<dt/2)e.trip();
  const p=e.step(dt,env);
  // Displayed sound foot (held on the ground), expressed relative to the pelvis like ourPts.
  const sa=p.sound.ankleWorld,soundPts=contact.map(c=>{const r=rotateFoot(c,p.sound.footPitch,0);return [sa[0]+r[0]-p.pelvis[0],sa[1]+r[1]-p.pelvis[1],sa[2]+r[2]];});
  rows.push({g:[...e.groundVel],t,p,ourPts:contact.map(c=>pointOf(p.our.leg,c)),soundPts,sa:[...sa],sp:p.sound.footPitch});
 }
 return {e,rows};
}
// Slip speed (mm/s) of each loaded foot's sole point relative to a ground moving at g.
export function footSlip(r,q,dt,g,horizontal=false){
 const H=(x,y)=>horizontal?Math.abs(x):Math.hypot(x,y);
 const c=footContact(q.p.our.leg,contact,0,()=>0).c,a=pointOf(r.p.our.leg,c),b=pointOf(q.p.our.leg,c);
 const our=H((a[0]+r.p.pelvis[0]-b[0]-q.p.pelvis[0])/dt-g[0],(a[1]+r.p.pelvis[1]-b[1]-q.p.pelvis[1])/dt-g[1]);
 const cs=footContact(q.p.sound.leg,contact,0,()=>0).c,ra=rotateFoot(cs,r.sp,0),rb=rotateFoot(cs,q.sp,0);
 const sound=H((r.sa[0]+ra[0]-q.sa[0]-rb[0])/dt-g[0],(r.sa[1]+ra[1]-q.sa[1]-rb[1])/dt-g[1]);
 return [our,sound];
}
export const range=(rows,f)=>{let lo=Infinity,hi=-Infinity;for(const r of rows){const v=f(r);if(v<lo)lo=v;if(v>hi)hi=v;}return [lo,hi];};
export const maxAbsJump=(rows,f,dt)=>{let m=0;for(let i=2;i<rows.length;i++){const a=f(rows[i]),b=f(rows[i-1]),c=f(rows[i-2]);m=Math.max(m,Math.abs(a-2*b+c)/(dt*dt));}return m;};

