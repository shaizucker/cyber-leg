// Checks for dist/energy.js. Run from CYBER_LEG_V01: node scripts/check_energy.mjs
import {EnergyMeter,splitWork} from '../dist/energy.js';
let failures=0;const check=(name,ok,detail='')=>{console.log(`${ok?'PASS':'FAIL'}  ${name}${detail?'  —  '+detail:''}`);if(!ok)failures++;};
const near=(a,b,e=1e-9)=>Math.abs(a-b)<=e;

// Trapezoid split: sign change at the midpoint gives equal +/− areas.
{const [p,n]=splitWork(10,-10,1);check('split: symmetric crossing',near(p,2.5)&&near(n,-2.5),`${p} / ${n}`);}
{const [p,n]=splitWork(4,6,2);check('split: positive only',near(p,10)&&n===0);}

// Constant 5 W for 2 s at 100 Hz → 10 J positive.
{const m=new EnergyMeter();for(let i=0;i<=200;i++)m.sample('knee',i/100,5);check('constant power integrates',near(m.devices.knee.positive,10,1e-6),m.devices.knee.positive.toFixed(4));}

// Sinusoidal power over whole periods: net ≈ 0, positive ≈ |negative|.
{const m=new EnergyMeter();for(let i=0;i<=1000;i++){const t=i/250;m.sample('hip',t,8*Math.sin(2*Math.PI*t));}
 const d=m.devices.hip;check('sinusoid: work delivered equals work absorbed',near(d.positive,-d.negative,1e-3)&&near(d.positive,4*8/Math.PI,2e-3),`+${d.positive.toFixed(4)} / ${d.negative.toFixed(4)}`);}

// Gaps longer than 0.5 s and stale samples are not integrated, never counted as zero power.
{const m=new EnergyMeter();m.sample('knee',0,10);m.sample('knee',2,10);check('gap not integrated',m.devices.knee.positive===0);
 m.markStale('knee');check('stale shows no power',m.devices.knee.power===null);
 m.ingest({rotary:{knee:{stale:true,mechanical_power_w:3,sampled_at:5}}},5);check('ingest ignores stale feedback',m.devices.knee.power===null&&m.history.at(-1).total===null);}

// Bridge-shaped status: duplicate sample times are not double counted; ankle sums three nodes.
{const m=new EnergyMeter();
 for(let i=0;i<=100;i++){const t=i/50,s={rotary:{knee:{stale:false,mechanical_power_w:2,sampled_at:t},hip:{stale:false,mechanical_power_w:-1,sampled_at:t}},ankle:{nodes:[1,2,3].map(()=>({stale:false,mechanical_power_w:1}))}};m.ingest(s,t);m.ingest(s,t);}
 check('knee 2 W × 2 s = 4 J',near(m.devices.knee.positive,4,1e-6),m.devices.knee.positive.toFixed(4));
 check('hip −1 W × 2 s = −2 J absorbed',near(m.devices.hip.negative,-2,1e-6));
 check('ankle sums its three nodes',near(m.devices.ankle.power,3)&&near(m.devices.ankle.positive,6,1e-6),m.devices.ankle.positive.toFixed(4));
 check('no electrical without DC sensors',!m.hasElectrical);}

// External DC sensor data is integrated separately as electrical energy.
{const m=new EnergyMeter();for(let i=0;i<=100;i++)m.sample('knee',i/100,1,48*.5);check('electrical energy from DC sensor',m.hasElectrical&&near(m.devices.knee.electricalJ,24,1e-6),m.devices.knee.electricalJ.toFixed(3));}

// Per-stride bookkeeping.
{const m=new EnergyMeter();let c=0;for(let i=0;i<=400;i++){const t=i/100;if(i%100===0)m.cycle(c++,t);m.sample('knee',t,3);}
 const s=m.perStride();check('per-stride work',s&&near(s.positive,3,.05)&&near(s.duration,1,1e-9),s&&`${s.positive.toFixed(3)} J / ${s.duration} s`);}

console.log(failures?`\n${failures} check(s) failed`:'\nAll checks passed');process.exitCode=failures?1:0;
