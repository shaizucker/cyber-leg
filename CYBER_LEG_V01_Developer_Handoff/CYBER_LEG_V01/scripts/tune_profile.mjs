// Random-search tuner for stair keyframes: maximises swing toe clearance over
// the nosing while keeping step geometry plausible and the curves close to the
// starting (normative) shape. Prints keyframes to paste into dist/gait.js.
// Usage: node scripts/tune_profile.mjs stairsUp '<keys JSON>' [iterations]
import {setProfileKeys} from '../dist/gait.js';
import {run} from './cad_rig.mjs';
const act=process.argv[2];
function evaluate(keys){
 setProfileKeys(act,keys);
 const dt=1/120,{rows}=run(act,{seconds:7,dt});
 const tread=(x,r)=>{for(const t of r.p.stairs.treads)if(x>=t.x0&&x<t.x1)return t.y;return -Infinity;};
 let clear=Infinity,pen=0;
 for(const r of rows.filter(r=>r.t>3)){const ph=r.p.phase;
  for(const [pts,sp] of [[r.ourPts,ph],[r.soundPts,(ph+.5)%1]])for(const p of pts){const x=p[0]+r.p.pelvis[0],y=p[1]+r.p.pelvis[1],d=y-tread(x,r);if(sp>.64&&sp<.96)clear=Math.min(clear,d);else if(sp<.58&&sp>.04)pen=Math.min(pen,d);}}
 const p=rows.at(-1).p,run_=p.stairs.run,rise=Math.abs(p.stairs.rise);
 const geo=Math.max(0,230-run_)+Math.max(0,run_-300)+Math.max(0,140-rise)+Math.max(0,rise-185);
 const kmax=Math.max(...rows.map(r=>r.p.target.knee));
 return {score:Math.min(clear,25)+Math.min(pen,0)*2-geo*.5-Math.max(0,kmax-103)*3,clear,pen,run:run_,rise,kmax};
}
const base=JSON.parse(process.argv[3]);
// Stay close to the normative shape and keep curves smooth.
const reg=k=>{let e=0;for(const j of ['hip','knee','ankle']){const a=k[j],b=base[j];for(let i=0;i<a.length;i++){e+=.004*(a[i][1]-b[i][1])**2;const p=a[(i-1+a.length)%a.length][1],n=a[(i+1)%a.length][1];e+=.002*(a[i][1]-(p+n)/2)**2;}}return e;};
let best=JSON.parse(JSON.stringify(base)),bs=evaluate(best);bs.score-=reg(best);
console.log('start',JSON.stringify(bs));
let seed=7;const rnd=()=>{seed=(seed*1103515245+12345)%2147483648;return seed/2147483648;};
for(let it=0;it<Number(process.argv[4]||150);it++){
 const cand=JSON.parse(JSON.stringify(best));
 for(const j of ['hip','knee','ankle']){if(rnd()<.6)continue;const ks=cand[j];const i=Math.floor(rnd()*ks.length);if(i===0&&rnd()<.7)continue;ks[i][1]+=(rnd()-.5)*12;
   if(j==='ankle')ks[i][1]=Math.max(-18,Math.min(18,ks[i][1]));}
 const r=evaluate(cand);r.score-=reg(cand);if(r.score>bs.score){best=cand;bs=r;}
}
console.log('best',JSON.stringify(bs));
console.log(JSON.stringify(Object.fromEntries(Object.entries(best).map(([k,v])=>[k,v.map(([a,b])=>[a,+b.toFixed(1)])]))));
