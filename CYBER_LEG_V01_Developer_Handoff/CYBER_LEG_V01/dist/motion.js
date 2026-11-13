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
// Shared curve helpers. The gait and activity generator lives in gait.js.
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
