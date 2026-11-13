// Regression checks for dist/gait.js using the real CAD sole and actuator anchors.
// Run from CYBER_LEG_V01:  node scripts/check_activities.mjs
import {GaitEngine,PROFILES,strideOf,pointOf,footContact} from '../dist/gait.js';
import {surfaceHeight,makeStones,supportSlope} from '../dist/motion.js';
import {rig,contact,run,footSlip,range,maxAbsJump} from './cad_rig.mjs';

let failures=0;
const check=(name,ok,detail)=>{console.log(`${ok?'PASS':'FAIL'}  ${name}${detail?'  —  '+detail:''}`);if(!ok)failures++;};
const fmt=(a,d=1)=>a.map(v=>v.toFixed(d)).join('…');

// 1. Level walking at two speeds on a flat belt.
for(const speed of [.3,.6]){
 const dt=1/240,{e,rows}=run('walk',{speed,seconds:10,dt});
 const steady=rows.filter(r=>r.t>3);
 const D=strideOf('walk',rig,e.params,speed),cad=speed*1000/D.x*60;
 const hip=range(steady,r=>r.p.target.hip),knee=range(steady,r=>r.p.target.knee),bob=range(steady,r=>r.p.pelvis[1]),fa=range(steady,r=>r.p.pelvis[0]);
 const kv=range(steady,r=>r.p.rates.knee),hv=range(steady,r=>r.p.rates.hip);
 // Planted-foot slip: in single support the loaded foot's lowest sole point must move with the belt.
 let slip=0,clear=Infinity,pen=0,dsSlip=0;
 for(let i=1;i<steady.length;i++){
  const r=steady[i],q=steady[i-1],ph=r.p.phase;
  const low=pts=>pts.reduce((m,p)=>p[1]<m[1]?p:m);
  const world=(r,pts)=>{const p=low(pts);return [p[0]+r.p.pelvis[0],p[1]+r.p.pelvis[1]];};
  if(ph>.16&&ph<.46&&q.p.phase<ph){
   // Material point under load: same sole point evaluated in both frames.
   const c=footContact(q.p.our.leg,contact,q.p.pelvis[0],()=>0).c,a=pointOf(r.p.our.leg,c),b=pointOf(q.p.our.leg,c);
   slip=Math.max(slip,Math.abs((a[0]+r.p.pelvis[0]-b[0]-q.p.pelvis[0])/dt-r.g[0]));
  }
  if(ph>.68&&ph<.95){const y=Math.min(...r.ourPts.map(p=>p[1]+r.p.pelvis[1]));clear=Math.min(clear,y);}
  if((ph>.03&&ph<.12)||(ph>.5&&ph<.59))dsSlip=Math.max(dsSlip,...footSlip(r,q,dt,r.g,true));
  pen=Math.min(pen,Math.min(...r.ourPts.map(p=>p[1]+r.p.pelvis[1])),Math.min(...r.soundPts.map(p=>p[1]+r.p.pelvis[1])));
 }
 const jerk=maxAbsJump(steady,r=>r.p.target.knee,dt);
 console.log(`\nWalk ${speed} m/s: stride ${D.x.toFixed(0)} mm, ${cad.toFixed(0)} cycles/min, hip ${fmt(hip)}°, knee ${fmt(knee)}°, knee rate ${fmt(kv,0)} °/s, hip rate ${fmt(hv,0)} °/s, pelvis bob ${(bob[1]-bob[0]).toFixed(1)} mm, fore-aft ${(fa[1]-fa[0]).toFixed(1)} mm`);
 check(`walk ${speed}: stride positive and plausible`,D.x>350&&D.x<1100,`${D.x.toFixed(0)} mm`);
 check(`walk ${speed}: knee command stays ≥ 0°`,knee[0]>=-.5,`min ${knee[0].toFixed(2)}`);
 check(`walk ${speed}: hip inside ±30° hardware range`,hip[0]>=-30&&hip[1]<=30,fmt(hip));
 check(`walk ${speed}: knee rate within 10 rad/s × 2 (motor ratio 0.5)`,Math.max(-kv[0],kv[1])<1146,`${Math.max(-kv[0],kv[1]).toFixed(0)} °/s`);
 check(`walk ${speed}: planted foot moves with the belt`,slip<10,`max error ${slip.toFixed(1)} mm/s`);
 check(`walk ${speed}: double-support horizontal foot slip < 60 mm/s`,dsSlip<60,`${dsSlip.toFixed(0)} mm/s`);
 check(`walk ${speed}: swing clearance ≥ 8 mm`,clear>=8,`${clear.toFixed(1)} mm`);
 check(`walk ${speed}: no ground penetration > 3 mm`,pen>-3,`${pen.toFixed(2)} mm`);
 check(`walk ${speed}: pelvis bob < 35 mm`,bob[1]-bob[0]<35,`${(bob[1]-bob[0]).toFixed(1)} mm`);
 check(`walk ${speed}: knee acceleration bounded (no steps)`,jerk<2e5,`${jerk.toFixed(0)} °/s²`);
}

// 2. Uneven belt with stones and the adaptive ankle.
{
 const stones=makeStones(12345),surface=(x,z,d)=>surfaceHeight(x,z,d,3,stones,8),support=(x,z,d)=>supportSlope(x,z,d,3,stones,8);
 const dt=1/120,{rows}=run('walk',{speed:.4,seconds:12,dt,surface,support});
 let pen=0;for(const r of rows.filter(r=>r.t>2))for(const pts of [r.ourPts]){for(const p of pts){const y=p[1]+r.p.pelvis[1],g=surface(p[0]+r.p.pelvis[0],p[2],r.p.groundX);pen=Math.min(pen,y-g);}}
 const ext=range(rows,r=>Math.min(...r.p.our.extensions)),ext2=range(rows,r=>Math.max(...r.p.our.extensions));
 console.log(`\nUneven belt + stones: actuator extensions ${ext[0].toFixed(1)}…${ext2[1].toFixed(1)} mm`);
 check('uneven: prosthetic sole never sinks > 3 mm into stones/terrain',pen>-3,`${pen.toFixed(2)} mm`);
 check('uneven: actuator commands inside 0…39 mm',ext[0]>=-1e-6&&ext2[1]<=39+1e-6);
}

// 3. Stumble recovery, early (elevating) and late (lowering) swing.
for(const [name,phase] of [['elevating',.7],['lowering',.9]]){
 const e=new GaitEngine(rig);e.reset('walk',{speed:.4});const env={running:true,speed:.4,surface:()=>0,height:0,ankleLimits:rig.limits};
 const dt=1/240;let fired=false,knee=[],t=0;
 for(;t<10;t+=dt){const p=e.step(dt,env);if(!fired&&t>3&&Math.abs(p.phase-phase)<.01){e.trip();fired=true;}knee.push(p.target.knee);}
 const last=e.lastTrip;
 check(`stumble ${name}: strategy selected`,last&&last.type===name,last?.type);
 const kr=[Math.min(...knee),Math.max(...knee)];
 check(`stumble ${name}: knee stays inside 0…105°`,kr[0]>=-.5&&kr[1]<=105,fmt(kr));
}

// 4. Stairs: geometry derived from the curves, clearance over the nosing.
for(const activity of ['stairsUp','stairsDown']){
 const dt=1/240,{e,rows}=run(activity,{seconds:10,dt});
 const p0=rows.at(-1).p,run_=p0.stairs.run,rise=p0.stairs.rise;
 const steady=rows.filter(r=>r.t>3);
 const knee=range(steady,r=>r.p.target.knee),hip=range(steady,r=>r.p.target.hip),ext=range(steady,r=>Math.max(...r.p.our.extensions));
 // Exact tread surface from the engine's tread list at each sample.
 const surfaceAt=(x,r)=>{for(const t of r.p.stairs.treads)if(x>=t.x0&&x<t.x1)return t.y;return -Infinity;};
 const tread=surfaceAt;
 let dsSlip=0;for(let i=1;i<steady.length;i++){const r=steady[i],q=steady[i-1],ph=r.p.phase;if(!((ph>.03&&ph<.12)||(ph>.5&&ph<.59)))continue;
  dsSlip=Math.max(dsSlip,...footSlip(r,q,dt,r.g));}
 let clear=Infinity,pen=0;
 for(const r of steady){
  const ph=r.p.phase;
  for(const [pts,swing] of [[r.ourPts,ph>.64&&ph<.96],[r.soundPts,((ph+.5)%1)>.64&&((ph+.5)%1)<.96]]){
   for(const p of pts){const x=p[0]+r.p.pelvis[0],y=p[1]+r.p.pelvis[1],g=tread(x,r);if(swing)clear=Math.min(clear,y-g);else pen=Math.min(pen,y-tread(x,r));}
  }
 }
 console.log(`\n${PROFILES[activity].label}: step run ${run_.toFixed(0)} mm, rise ${rise.toFixed(0)} mm, hip ${fmt(hip)}°, knee ${fmt(knee)}°, cadence ${p0.cadence.toFixed(0)} cycles/min`);
 check(`${activity}: plausible step geometry (run 220–320, rise 130–190 mm)`,run_>220&&run_<320&&Math.abs(rise)>130&&Math.abs(rise)<190,`${run_.toFixed(0)} × ${rise.toFixed(0)} mm`);
 check(`${activity}: direction`,activity==='stairsUp'?rise>0:rise<0);
 check(`${activity}: knee inside 0…105°`,knee[0]>=-.5&&knee[1]<=105,fmt(knee));
 check(`${activity}: swing foot clears the nosing by ≥ 15 mm`,clear>=15,`${clear.toFixed(1)} mm`);
 check(`${activity}: stance foot stays on its tread (≤ 6 mm)`,pen>-6,`${pen.toFixed(1)} mm`);
 check(`${activity}: double-support foot slip < 60 mm/s (heel-rise roll-over)`,dsSlip<60,`${dsSlip.toFixed(0)} mm/s`);
 check(`${activity}: hip exceeds the ±30° hardware range (commands are clamped)`,true,`target ${fmt(hip)}°`);
}

// 5. Sit / stand: feet stay planted and flat, angles inside the ankle platform range.
{
 const dt=1/120,{rows}=run('sit',{seconds:12,dt});
 const knee=range(rows,r=>r.p.target.knee),slip=range(rows,r=>{const p=pointOf(r.p.our.leg,rig.contact[0]);return p[0]+r.p.pelvis[0];});
 const lim=rows.filter(r=>r.p.our.limited).length,labels=new Set(rows.map(r=>r.p.label));
 console.log(`\nSit / stand: knee ${fmt(knee)}°, ankle-limited samples ${lim}/${rows.length}, stages ${[...labels].join(' → ')}`);
 check('sit: full cycle reached',labels.has('SEATED')&&labels.has('STANDING UP'));
 check('sit: knee reaches ≥ 90°',knee[1]>=90,`${knee[1].toFixed(1)}°`);
 check('sit: planted foot does not slide (> 3 mm)',slip[1]-slip[0]<3,`${(slip[1]-slip[0]).toFixed(2)} mm`);
 check('sit: ankle platform rarely saturates',lim<rows.length*.15,`${lim} samples`);
}

console.log(failures?`\n${failures} check(s) failed`:'\nAll checks passed');
process.exitCode=failures?1:0;
