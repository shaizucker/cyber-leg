// Energy and power bookkeeping from bridge motor feedback. Pure module.
//
// What is measured: MECHANICAL power at each output, from motor feedback
// (rotary: torque × angular speed; linear actuators: force × speed, as the
// bridge reports it). Positive = motor delivers work, negative = motor absorbs
// work (braking / potentially regenerative). This is NOT electrical
// consumption: the drives report no DC bus current. Electrical power appears
// only for devices with an external DC sensor posting to /api/power.
// Stale or missing samples are never counted as zero.

export const DEVICES=['knee','hip','ankle'];
const GAP=.5;// seconds; longer gaps are not integrated

function emptyDevice(){return {power:null,electrical:null,positive:0,negative:0,electricalJ:0,last:null,lastE:null,available:false,stale:true};}

// Signed trapezoid split into positive and negative work.
export function splitWork(p0,p1,dt){
 if(p0>=0&&p1>=0)return [(p0+p1)/2*dt,0];
 if(p0<=0&&p1<=0)return [0,(p0+p1)/2*dt];
 const t=Math.abs(p0)/(Math.abs(p0)+Math.abs(p1))*dt,a=p0*t/2,b=p1*(dt-t)/2;
 return p0>0?[a,b]:[b,a];
}

export class EnergyMeter{
 constructor(){this.reset();}
 reset(){
  this.devices=Object.fromEntries(DEVICES.map(d=>[d,emptyDevice()]));
  this.history=[];this.strides=[];this.strideStart=null;this.lastCycle=null;this.hasElectrical=false;
 }
 // Integrate one device sample. t in seconds (any monotonic clock per device).
 sample(name,t,power,electrical=null){
  const d=this.devices[name];
  if(!Number.isFinite(power)||!Number.isFinite(t)){d.power=null;d.stale=true;d.last=null;return;}
  if(d.last&&t>d.last.t&&t-d.last.t<=GAP){const [pos,neg]=splitWork(d.last.p,power,t-d.last.t);d.positive+=pos;d.negative+=neg;}
  if(!d.last||t!==d.last.t)d.last={t,p:power};
  d.power=power;d.available=true;d.stale=false;
  if(Number.isFinite(electrical)){
   this.hasElectrical=true;
   if(d.lastE&&t>d.lastE.t&&t-d.lastE.t<=GAP)d.electricalJ+=(d.lastE.p+electrical)/2*(t-d.lastE.t);
   d.lastE={t,p:electrical};d.electrical=electrical;
  }else{d.electrical=null;d.lastE=null;}
 }
 markStale(name){const d=this.devices[name];d.power=null;d.stale=true;d.last=null;d.electrical=null;d.lastE=null;}
 // Read the bridge status (data.console). now = browser seconds, used for the ankle.
 ingest(consoleStatus,now){
  const rotary=consoleStatus?.rotary||{};
  for(const name of ['knee','hip']){
   const r=rotary[name];
   if(!r||r.stale||r.mechanical_power_w==null)this.markStale(name);
   else this.sample(name,r.sampled_at,r.mechanical_power_w,r.power?.electrical_power_w);
  }
  const nodes=consoleStatus?.ankle?.nodes;
  if(!nodes||nodes.some(n=>n.stale||n.mechanical_power_w==null))this.markStale('ankle');
  else{
   const e=nodes.map(n=>n.power?.electrical_power_w);
   this.sample('ankle',now,nodes.reduce((a,n)=>a+n.mechanical_power_w,0),e.every(Number.isFinite)?e.reduce((a,b)=>a+b,0):null);
  }
  const row={t:now};let total=0,any=false;
  for(const name of DEVICES){const p=this.devices[name].power;row[name]=p;if(p!=null){total+=p;any=true;}}
  row.total=any?total:null;
  this.history.push(row);
  while(this.history.length&&this.history[0].t<now-10)this.history.shift();
 }
 totals(){
  const out={positive:0,negative:0,electricalJ:0};
  for(const d of Object.values(this.devices)){out.positive+=d.positive;out.negative+=d.negative;out.electricalJ+=d.electricalJ;}
  return out;
 }
 // Call with the gait cycle counter; records net/positive work per completed cycle.
 cycle(count,time){
  if(count===this.lastCycle)return;
  const t=this.totals();
  if(this.strideStart&&this.lastCycle!==null)this.strides.push({time,positive:t.positive-this.strideStart.positive,negative:t.negative-this.strideStart.negative,duration:time-this.strideStart.time});
  if(this.strides.length>50)this.strides.shift();
  this.strideStart={...t,time};this.lastCycle=count;
 }
 perStride(){
  const s=this.strides.slice(-5);if(!s.length)return null;
  const n=s.length;return {positive:s.reduce((a,x)=>a+x.positive,0)/n,negative:s.reduce((a,x)=>a+x.negative,0)/n,duration:s.reduce((a,x)=>a+x.duration,0)/n};
 }
}
