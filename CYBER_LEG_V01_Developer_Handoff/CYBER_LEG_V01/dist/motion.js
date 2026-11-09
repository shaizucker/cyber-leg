export const rad=Math.PI/180;
export const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
export const smooth=t=>{t=clamp(t,0,1);return t*t*(3-2*t)};
export function terrain(x,z,distance,amplitude){
 const u=x+distance;
 return amplitude*(.43*Math.sin(u*.021+z*.012)+.28*Math.sin(u*.047-z*.031+1.3)+.19*Math.sin(u*.009+z*.039+2.8)+.10*Math.sin(u*.083+z*.055));
}
export function terrainSlope(x,z,distance,amplitude){
 const dx=(terrain(x+3,z,distance,amplitude)-terrain(x-3,z,distance,amplitude))/6;
 const dz=(terrain(x,z+3,distance,amplitude)-terrain(x,z-3,distance,amplitude))/6;
 return {pitch:Math.atan(dx)/rad,roll:-Math.atan(dz)/rad};
}
export const STONE_PERIOD=2800;
export const wrapStone=x=>((x+STONE_PERIOD/2)%STONE_PERIOD+STONE_PERIOD)%STONE_PERIOD-STONE_PERIOD/2;
export function makeStones(seed){
 let n=seed>>>0;const random=()=>{n=(Math.imul(n,1664525)+1013904223)>>>0;return n/4294967296;};
 const result=[];
 // Randomized placements near alternating sole edges ensure useful left/right trials.
 for(let i=0;i<8;i++)result.push({x:100+350*i+(random()-.5)*60,z:(i%2?1:-1)*(16+random()*10),rx:13+random()*9,rz:10+random()*7,height:.6+random()*.4,angle:random()*Math.PI});
 for(let i=0;i<36;i++)result.push({x:random()*STONE_PERIOD,z:(random()-.5)*260,rx:10+random()*11,rz:8+random()*9,height:.45+random()*.55,angle:random()*Math.PI});
 return result;
}
// Octagonal, faceted stone: the renderer uses these same rings and height profile.
export function stoneProfile(q){return q>=1?0:q<=.45?1-.18*q/.45:.82*(1-q)/.55;}
export function stoneGauge(x,z,s){
 const c=Math.cos(s.angle),r=Math.sin(s.angle),u=(c*x+r*z)/s.rx,v=(-r*x+c*z)/s.rz;
 let q=0;for(let k=0;k<8;k++)q=Math.max(q,u*Math.cos(k*Math.PI/4)+v*Math.sin(k*Math.PI/4));return q;
}
export function surfaceHeight(x,z,distance,amplitude,stones=[],height=8){
 let y=terrain(x,z,distance,amplitude);
 for(const s of stones){const cx=wrapStone(s.x-distance),dx=x-cx,dz=z-s.z;if(Math.abs(dx)>s.rx+s.rz||Math.abs(dz)>s.rx+s.rz)continue;
  const q=stoneGauge(dx,dz,s);if(q<1)y=Math.max(y,terrain(cx,s.z,distance,amplitude)+s.height*height*stoneProfile(q));
 }return y;
}
export function supportSlope(x,z,distance,amplitude,stones=[],height=8){
 let left=-Infinity,right=-Infinity,front=-Infinity,back=-Infinity;
 for(let dx=-70;dx<=90;dx+=10)for(const dz of [-26,-18,-8,8,18,26]){
  const y=surfaceHeight(x+dx,z+dz,distance,amplitude,stones,height);
  if(dz>0)left=Math.max(left,y);else right=Math.max(right,y);
  if(dx>10)front=Math.max(front,y);else back=Math.max(back,y);
 }
 return {pitch:clamp(Math.atan((front-back)/95)/rad,-18,18),roll:clamp(-Math.atan((left-right)/44)/rad,-18,18),left,right};
}
// ---------------------------------------------------------------------------
// Smooth gait generator.
// Design rules: (1) every trajectory is C2-continuous around the whole cycle,
// so commanded joint speed never jumps; (2) the ankle moves exactly with the
// belt during stance and lands with belt speed and zero vertical speed;
// (3) hip and knee come from inverse kinematics about a FIXED hip mount, so
// the joint angles sent to the motors are the ones that produce the foot path.
// Edit GAIT to tune. Angles in degrees, lengths in mm, phase 0 = heel strike.
export const GAIT={
 stance:.62,          // stance fraction of the cycle
 stride:350,          // belt travel per cycle (mm); cadence = speed / stride
 hipLead:.55,         // share of the stance path in front of the hip
 clearance:38,        // extra ankle lift in swing (mm), peak near 43% of swing
 minKnee:4,           // knee flexion at the longest reach; keeps IK away from the straight-leg singularity
 // Foot angle to the ground, toes up positive: heel strike, foot flat, heel rise, push-off, swing.
 footPitch:[[0,4],[.07,0],[.36,0],[.50,-6],[.62,-24],[.70,-14],[.80,-3],[.92,3]],
 // Terrain adaptation weight: fades out over heel rise, fades in over late swing.
 adaptFade:[.44,.60],adaptIn:.55
};
export const smooth5=t=>{t=clamp(t,0,1);return t*t*t*(10+t*(6*t-15));};
// Periodic C2 cubic spline through [phase, value] keys (period 1). Returns p => [value, d/dphase].
export function periodicSpline(keys){
 const n=keys.length,t=keys.map(k=>k[0]),y=keys.map(k=>k[1]);
 const h=t.map((v,i)=>i<n-1?t[i+1]-v:1+t[0]-v);
 const A=Array.from({length:n},()=>new Float64Array(n+1));
 for(let i=0;i<n;i++){const ip=(i-1+n)%n,inx=(i+1)%n;
  A[i][ip]+=h[ip];A[i][i]+=2*(h[ip]+h[i]);A[i][inx]+=h[i];
  A[i][n]=6*((y[inx]-y[i])/h[i]-(y[i]-y[ip])/h[ip]);}
 for(let c=0;c<n;c++){let r=c;for(let k=c+1;k<n;k++)if(Math.abs(A[k][c])>Math.abs(A[r][c]))r=k;[A[c],A[r]]=[A[r],A[c]];
  for(let k=0;k<n;k++)if(k!==c){const f=A[k][c]/A[c][c];for(let j=c;j<=n;j++)A[k][j]-=f*A[c][j];}}
 const M=A.map((row,i)=>row[n]/row[i]);
 return phase=>{
  const p=((phase%1)+1)%1;let i=n-1;for(let k=0;k<n-1;k++)if(p>=t[k]&&p<t[k+1]){i=k;break;}
  let x=p-t[i];if(x<0)x+=1;const hi=h[i],j=(i+1)%n,a=(hi-x)/hi,b=x/hi;
  const v=a*y[i]+b*y[j]+((a*a*a-a)*M[i]+(b*b*b-b)*M[j])*hi*hi/6;
  const d=(y[j]-y[i])/hi+((1-3*a*a)*M[i]+(3*b*b-1)*M[j])*hi/6;
  return [v,d];
 };
}
// Quintic Hermite: position, velocity and acceleration matched at both ends.
function quintic(p0,v0,a0,p1,v1,a1,s){
 const s2=s*s,s3=s2*s,s4=s3*s,s5=s4*s;
 return p0*(1-10*s3+15*s4-6*s5)+v0*(s-6*s3+8*s4-3*s5)+a0*(.5*s2-1.5*s3+1.5*s4-.5*s5)
  +a1*(.5*s3-s4+.5*s5)+v1*(-4*s3+7*s4-3*s5)+p1*(10*s3-15*s4+6*s5);
}
const LIFT_PEAK=(3/7)**3*(4/7)**4;
let pitchSpline=null,pitchKeys=null;
export function footPitchAt(phase){if(pitchKeys!==GAIT.footPitch){pitchKeys=GAIT.footPitch;pitchSpline=periodicSpline(pitchKeys);}return pitchSpline(phase)[0];}
// Ankle path relative to the hip mount (x forward). No vertical terms besides swing lift.
export function footPath(phase){
 const p=((phase%1)+1)%1,{stance,stride,hipLead,clearance}=GAIT,Ls=stance*stride,xHS=hipLead*Ls,xTO=xHS-Ls;
 if(p<stance)return {x:xHS-stride*p,lift:0,stance:true,s:p/stance,phase:p};
 const s=(p-stance)/(1-stance),v=-stride*(1-stance);
 return {x:quintic(xTO,v,0,xHS,v,0,s),lift:clearance*s*s*s*(1-s)**4/LIFT_PEAK,stance:false,s,phase:p};
}
export function gaitLabel(p){const{stance}=GAIT;return p<.07?'HEEL CONTACT':p<.45?'STANCE':p<stance?'PUSH-OFF':'SWING';}
function adaptWeight(p){const{stance,adaptFade,adaptIn}=GAIT;
 if(p<stance)return 1-smooth5((p-adaptFade[0])/(adaptFade[1]-adaptFade[0]));
 return smooth5(((p-stance)/(1-stance)-adaptIn)/(1-adaptIn));}
// Ankle height that puts the sole on the surface (foot orientation is world pitch/roll).
// A soft maximum (1 mm temperature) replaces the hard max so the heel-to-flat-to-toe
// hand-over has no kink; it sits at most ~1 mm x ln(samples) above the hard contact.
export function contactHeight(contact,pitch,roll,ax,az,surface,T=1){
 const ys=contact.map(v=>{const r=rotateFoot(v,pitch,roll);return surface(ax+r[0],az+r[2])-r[1];});
 const m=Math.max(...ys);let s=0;for(const y of ys)s+=Math.exp((y-m)/T);return m+T*Math.log(s)-T*Math.log(ys.length);
}
// Planar two-link IK. u = knee->hip in the thigh frame, w = knee->ankle in the shank frame.
export function legLength(u,w,knee){const c=Math.cos(knee*rad),s=Math.sin(knee*rad);return Math.hypot(c*w[0]+s*w[1]-u[0],-s*w[0]+c*w[1]-u[1]);}
export function solveLeg(dx,dy,u,w,maxKnee=105){
 const Lu=Math.hypot(u[0],u[1]),Lw=Math.hypot(w[0],w[1]),D=Math.hypot(dx,dy);
 const c=clamp((Lu*Lu+Lw*Lw-D*D)/(2*Lu*Lw),-1,1);
 let beta=Math.atan2(w[1],w[0])-Math.atan2(u[1],u[0]);beta=((beta%(2*Math.PI))+2*Math.PI)%(2*Math.PI);
 const raw=(beta-Math.acos(c))/rad,knee=clamp(raw,0,maxKnee);
 const cs=Math.cos(knee*rad),sn=Math.sin(knee*rad),vx=cs*w[0]+sn*w[1]-u[0],vy=-sn*w[0]+cs*w[1]-u[1];
 return {hip:(Math.atan2(dy,dx)-Math.atan2(vy,vx))/rad,knee,reach:raw<0||raw>maxKnee};
}
// Fixed hip height: the longest reach in the flat-ground cycle gets exactly minKnee.
export function hipHeight(rig){
 const L=legLength(rig.u,rig.w,GAIT.minKnee),flat=()=>0;let H=Infinity;
 for(let i=0;i<400;i++){const p=i/400,f=footPath(p),y=contactHeight(rig.contact,footPitchAt(p),0,0,0,flat)+f.lift;
  if(Math.abs(f.x)<L)H=Math.min(H,y+Math.sqrt(L*L-f.x*f.x));}
 return H;
}
/* Full pose for one phase. Pure function of (phase, belt distance at that phase).
 rig: {u,w,contact,anchors,pivot,limits,hipX,hipY,z}
 env: {surface(x,z,distance), support(x,z,distance) -> {pitch,roll}|null, height} */
export function gaitPose(phase,distance,rig,env){
 const p=((phase%1)+1)%1,{stance,stride}=GAIT,f=footPath(p),flat=()=>0;
 const surf=d=>(x,z)=>env.surface(x,z,d);
 // Terrain under a future/past stance foot is fixed to the belt, so it can be looked up exactly.
 const landD=distance+(1-p)*stride,toD=distance-(p-stance)*stride;
 const sup=env.support?env.support(rig.hipX+f.x+40,rig.z,p<stance?distance:landD):null,w=adaptWeight(p);
 let pitch=footPitchAt(p)+(sup?w*sup.pitch:0),roll=sup?w*sup.roll:0;
 const ax=rig.hipX+f.x;
 const ankleY=(pt,rl)=>{
  if(p<stance)return contactHeight(rig.contact,pt,rl,ax,rig.z,surf(distance));
  // Swing: blend the terrain offset from toe-off to the predicted landing.
  const p0=footPitchAt(0),pTO=footPitchAt(stance),sL=env.support?env.support(rig.hipX+footPath(0).x+40,rig.z,landD):null;
  const gTO=contactHeight(rig.contact,pTO,0,rig.hipX+footPath(stance).x,rig.z,surf(toD))-contactHeight(rig.contact,pTO,0,0,0,flat);
  const pL=p0+(sL?sL.pitch:0),rL=sL?sL.roll:0,xL=rig.hipX+footPath(0).x;
  const gL=contactHeight(rig.contact,pL,rL,xL,rig.z,surf(landD))-contactHeight(rig.contact,pL,rL,0,0,flat);
  return contactHeight(rig.contact,pt,rl,0,0,flat)+f.lift+gTO+(gL-gTO)*smooth5(f.s);
 };
 const w0=[rig.w[0],rig.w[1]-(env.height||0)];
 let ay=ankleY(pitch,roll),leg=solveLeg(ax-rig.hipX,ay-rig.hipY,rig.u,w0);
 let rel=pitch-(leg.hip-leg.knee),pose=feasiblePose(rig.anchors,rig.pivot,rel,roll,env.height||0,rig.limits);
 if(pose.limited){// Re-seat the foot with the orientation the ankle can actually reach.
  pitch=pose.pitch+(leg.hip-leg.knee);roll=pose.roll;ay=ankleY(pitch,roll);leg=solveLeg(ax-rig.hipX,ay-rig.hipY,rig.u,w0);
  pose=feasiblePose(rig.anchors,rig.pivot,pitch-(leg.hip-leg.knee),roll,env.height||0,rig.limits);}
 return {phase:p,stance:p<stance,label:gaitLabel(p),hip:leg.hip,knee:leg.knee,reach:leg.reach,footPitch:pitch,basePitch:footPitchAt(p)-(leg.hip-leg.knee),
  pose,ankle:[ax,ay],support:sup,weight:w};
}
// Joint speeds (deg/s) by central difference of the pure pose function.
export function gaitRates(phase,distance,rig,env,speed){
 const h=1e-3,r=speed*1000/GAIT.stride,a=gaitPose(phase+h,distance+h*GAIT.stride,rig,env),b=gaitPose(phase-h,distance-h*GAIT.stride,rig,env);
 return {hip:(a.hip-b.hip)/(2*h)*r,knee:(a.knee-b.knee)/(2*h)*r};
}
// Rotation matches THREE Euler(roll, 0, pitch, 'ZXY'): Rz * Rx.
export function rotateFoot(v,pitch,roll){
 const p=pitch*rad,r=roll*rad,cr=Math.cos(r),sr=Math.sin(r),cp=Math.cos(p),sp=Math.sin(p);
 const y=cr*v[1]-sr*v[2],z=sr*v[1]+cr*v[2];
 return [cp*v[0]-sp*y,sp*v[0]+cp*y,z];
}
export function actuatorExtensions(anchors,pivot,pitch,roll,height){
 return anchors.map(a=>{
  const r=rotateFoot(a.bottom.map((v,i)=>v-pivot[i]),pitch,roll);
  const point=r.map((v,i)=>v+pivot[i]-(i===1?height:0));
  return 20+Math.hypot(...point.map((v,i)=>v-a.top[i]))-a.length;
 });
}
export function feasiblePose(anchors,pivot,pitch,roll,height,limits=null){
 const valid=e=>e.every((x,i)=>x>=(limits?.[i]?.[0]??0)&&x<=(limits?.[i]?.[1]??50));
 let e=actuatorExtensions(anchors,pivot,pitch,roll,height);
 if(valid(e))return {pitch,roll,height,extensions:e,limited:false};
 let lo=0,hi=1;
 for(let i=0;i<22;i++){
  const m=(lo+hi)/2;
  if(valid(actuatorExtensions(anchors,pivot,pitch*m,roll*m,height*m)))lo=m;else hi=m;
 }
 return {pitch:pitch*lo,roll:roll*lo,height:height*lo,extensions:actuatorExtensions(anchors,pivot,pitch*lo,roll*lo,height*lo),limited:true};
}
