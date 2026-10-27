// Motor settings are separate from animation state. Rendering never sends writes.
export function installMotorConsole({post, onReference, onLimits}) {
 const dialog=document.getElementById('motorConsole');
 const host=document.getElementById('motorConsoleCards');
 const message=document.getElementById('motorConsoleMessage');
 let latest=null, busy=false;
 const $=id=>document.getElementById(id);
 const fmt=(n,unit='',digits=2)=>Number.isFinite(n)?`${n.toFixed(digits)}${unit}`:'Unavailable';
 const input=(id,label,step='0.1')=>`<label>${label}<input id="${id}" type="number" step="${step}"></label>`;
 const metric=(id,label)=>`<div><dt>${label}</dt><dd id="${id}">Unavailable</dd></div>`;
 function build(){
  for(const name of ['knee','hip'])host.insertAdjacentHTML('beforeend',`
   <article class="motor-card" id="mc-${name}"><h3>${name==='hip'?'Hip · CAN 2':'Knee · CAN 1'} <span id="mc-${name}-fresh" class="motor-fresh">Disabled</span></h3>
   <dl class="motor-metrics">${metric(`mc-${name}-angle`,'Actual / target from zero')}${metric(`mc-${name}-torque`,'Motor torque')}${metric(`mc-${name}-voltage`,'DC supply voltage')}${metric(`mc-${name}-power`,'DC electrical power')}${metric(`mc-${name}-mechanical`,'Mechanical power')}${metric(`mc-${name}-zero-value`,'Zero at motor angle')}</dl>
   <p class="motor-note">Voltage and input power require an external DC sensor; they are not included in rotary CAN feedback.</p>
   <div class="motor-fields">${input(`mc-${name}-p`,'Position P')}${input(`mc-${name}-i`,'Position I (bridge)')}${input(`mc-${name}-d`,'Position D')}${input(`mc-${name}-icap`,'Integral cap · Nm','0.01')}</div>
   <p class="motor-note">P/D run in the motor. I uses bounded torque feedforward with anti-windup; read the motor limit before enabling it.</p>
   <button data-action="pid" data-target="${name}">Apply PID live</button>
   <div class="motor-fields">${input(`mc-${name}-min`,'Command minimum · °')}${input(`mc-${name}-max`,'Command maximum · °')}${input(`mc-${name}-cap`,'Motor limit [21] · firmware units','0.01')}</div>
   <p class="motor-note" id="mc-${name}-hardware">Hardware limits not read. Command limits narrow the existing verified motor range.</p>
   <div class="motor-actions"><button data-action="read" data-target="${name}">Read hardware</button><button data-action="limits" data-target="${name}">Apply limits</button><button data-action="zero" data-target="${name}">Current pose = zero</button></div>
   </article>`);
  host.insertAdjacentHTML('beforeend',`<article class="motor-card motor-ankle" id="mc-ankle"><h3>Ankle · nodes 1 / 2 / 3 <span id="mc-ankle-fresh" class="motor-fresh">Disabled</span></h3>
   <p>One position PID set applies to all three actuators. Each actuator keeps its own measured force, current, zero and limits.</p>
   <div class="motor-fields">${input('mc-ankle-p','Shared position P','0.00001')}${input('mc-ankle-i','Shared position I','0.00001')}${input('mc-ankle-d','Shared position D','0.00001')}</div>
   <p id="mc-ankle-pid-note" class="motor-note">Read hardware to load the three existing PID sets before applying a shared set.</p>
   <div class="motor-actions"><button data-action="read" data-target="ankle">Read all three</button><button data-action="pid" data-target="ankle">Apply PID to all three</button><button data-action="zero" data-target="ankle">Current ankle pose = zero</button></div>
   <div class="motor-table-wrap"><table class="motor-table"><thead><tr><th>Node</th><th>Position from zero</th><th>Force</th><th>Motor current</th><th>Motor torque</th><th>DC voltage</th><th>DC input power</th><th>Mechanical power</th></tr></thead><tbody>${[1,2,3].map(n=>`<tr id="mc-a${n}"><th>A${n}</th>${['pos','force','current','torque','voltage','power','mechanical'].map(key=>`<td id="mc-a${n}-${key}">Unavailable</td>`).join('')}</tr>`).join('')}</tbody></table></div>
   <p class="motor-note">Linear output is measured in N, not Nm. Motor torque is unavailable without a documented conversion. Motor current is not a verified DC bus-current measurement; electrical consumption is shown only from DC sensors.</p>
   <div class="motor-node-limits">${[1,2,3].map(n=>`<fieldset><legend>A${n} · limits relative to zero</legend>${input(`mc-a${n}-min`,'Minimum · mm')}${input(`mc-a${n}-max`,'Maximum · mm')}${input(`mc-a${n}-cap`,'Peak motor current · A','0.01')}</fieldset>`).join('')}</div>
   <button data-action="limits" data-target="ankle">Apply actuator limits</button></article>`);
 }
 function value(id){const el=$(id);if(el.value.trim()==='')throw Error('Fill in '+el.parentElement.textContent.trim());const n=Number(el.value);if(!Number.isFinite(n))throw Error('Enter finite numeric values');return n;}
 function populate(force=false){
  if(!latest?.console)return;
  const c=latest.console;
  for(const name of ['knee','hip']){
   const r=c.rotary[name],card=$(`mc-${name}`);card.classList.toggle('motor-disabled',!r);
   if(!r)continue;
   if(force||!card.dataset.populated){
    for(const key of ['p','i','d'])$(`mc-${name}-${key}`).value=r.pid[key];
    $(`mc-${name}-icap`).value=r.integral_limit_nm;
    $(`mc-${name}-min`).value=String(Math.ceil(r.limits_deg[0]*1000)/1000);$(`mc-${name}-max`).value=String(Math.floor(r.limits_deg[1]*1000)/1000);
    $(`mc-${name}-cap`).value=r.expected_torque_limit_value??r.hardware?.torque_limit_value??'';
    card.dataset.populated='yes';
   }
  }
  const a=c.ankle,card=$('mc-ankle');card.classList.toggle('motor-disabled',!a);
  if(a&&(force||!card.dataset.populated)){
   const rows=a.hardware?.pid_by_node,pid=a.pid||rows?.[0];
   if(pid)for(const [i,key]of ['p','i','d'].entries())$(`mc-ankle-${key}`).value=pid[i];
   $('mc-ankle-pid-note').textContent=rows?rows.map((row,i)=>`A${i+1}: ${row.join(' / ')}`).join(' · '):'Read hardware to load existing PID sets. No default PID values are invented.';
   for(let i=0;i<3;i++){
    $(`mc-a${i+1}-min`).value=a.limits_mm?.[i]?.[0]?.toFixed(3)??'';
    $(`mc-a${i+1}-max`).value=a.limits_mm?.[i]?.[1]?.toFixed(3)??'';
    $(`mc-a${i+1}-cap`).value=a.current_limits_a?.[i]??a.hardware?.current_limits_a?.[i]??'';
   }
   card.dataset.populated='yes';
  }
 }
 async function action(target,what){
  if(busy)return;busy=true;render(latest);message.textContent='Working…';
  try{
   let response;
   if(what==='read')response=await post('read-hardware',{target});
   else if(what==='zero'){
    response=await post('reference',{target});onReference(target);
   }else{
    let settings;
    if(target==='ankle'){
     if(what==='pid')settings={pid:['p','i','d'].map(k=>value(`mc-ankle-${k}`))};
     else{
      settings={limits_mm:[1,2,3].map(n=>[value(`mc-a${n}-min`),value(`mc-a${n}-max`)])};
      if([1,2,3].every(n=>$(`mc-a${n}-cap`).value!==''))settings.current_limits_a=[1,2,3].map(n=>value(`mc-a${n}-cap`));
     }
    }else if(what==='pid')settings={p:value(`mc-${target}-p`),i:value(`mc-${target}-i`),d:value(`mc-${target}-d`),integral_limit_nm:value(`mc-${target}-icap`)};
    else{
     settings={limits_deg:[value(`mc-${target}-min`),value(`mc-${target}-max`)]};
     if($(`mc-${target}-cap`).value!=='')settings.torque_limit_value=value(`mc-${target}-cap`);
    }
    response=await post('configure',{target,settings});
   }
   latest=response;populate(true);onLimits(response);message.textContent=response.console.message;
  }catch(e){message.textContent=e.message;}
  finally{busy=false;render(latest);}
 }
 function render(data){
  if(data)latest=data;
  if(!latest?.console){$('openMotorConsole').disabled=true;return;}
  $('openMotorConsole').disabled=false;
  const c=latest.console;
  for(const name of ['knee','hip']){
   const r=c.rotary[name];if(!r)continue;
   $(`mc-${name}-fresh`).textContent=r.stale?'Last sample / stale':latest.armed?'Live':'Stopped sample';
   $(`mc-${name}-fresh`).classList.toggle('live',!r.stale);
   $(`mc-${name}-angle`).textContent=`${fmt(r.actual_angle_deg,'°',1)} / ${fmt(r.command_angle_deg,'°',1)}`;
   $(`mc-${name}-torque`).textContent=fmt(r.torque_nm,' Nm');
   $(`mc-${name}-mechanical`).textContent=fmt(r.mechanical_power_w,' W');
   $(`mc-${name}-voltage`).textContent=fmt(r.power?.voltage_v,' V');
   $(`mc-${name}-power`).textContent=fmt(r.power?.electrical_power_w,' W');
   $(`mc-${name}-zero-value`).textContent=fmt(r.reference_motor_rad*180/Math.PI,'°',2)+(r.reference_saved?' · captured':' · native motor zero');
   $(`mc-${name}-hardware`).textContent=r.hardware?`Verified motor range ${r.hardware_limits_deg.map(x=>x.toFixed(2)+'°').join(' … ')}; reported [21] ${r.hardware.torque_limit_value}${r.expected_torque_limit_value!=null?` / saved ${r.expected_torque_limit_value}`:''}; torque constant ${r.hardware.torque_constant}. Command limits are relative to the captured zero.`:'Read hardware before editing the motor output cap. Encoding/protection range stays unchanged by command-limit edits.';
  }
  if(c.ankle){
   $('mc-ankle-fresh').textContent=latest.ankle?.armed?'Live / per-node timestamps':'Stopped';
   for(const n of c.ankle.nodes){
    const prefix=`mc-a${n.node}`;
    $(prefix+'-pos').textContent=fmt(n.position_mm,' mm');$(prefix+'-force').textContent=fmt(n.force_n,' N');
    $(prefix+'-current').textContent=fmt(n.motor_current_a,' A');$(prefix+'-torque').textContent='Unavailable';
    $(prefix+'-voltage').textContent=fmt(n.power?.voltage_v,' V');$(prefix+'-power').textContent=fmt(n.power?.electrical_power_w,' W');
    $(prefix+'-mechanical').textContent=fmt(n.mechanical_power_w,' W');$(prefix).classList.toggle('motor-stale',n.stale);
   }
  }
  const stopped=!latest.armed&&!latest.ankle?.arming&&!latest.ankle?.closing;
  for(const button of host.querySelectorAll('button')){
   const name=button.dataset.target,enabled=name==='ankle'?!!c.ankle:!!c.rotary[name];
   const needsStop=button.dataset.action!=='pid'||name==='ankle';
   button.disabled=busy||!enabled||(needsStop&&(!stopped||latest.simulator_running));
  }
  $('motorReferenceAll').disabled=busy||latest.armed||latest.ankle?.arming||latest.ankle?.closing||latest.simulator_running;
  if(dialog.open)populate();
 }
 build();
 $('openMotorConsole').onclick=()=>{dialog.showModal();populate(true);render(latest);};
 $('closeMotorConsole').onclick=()=>dialog.close();
 $('motorReferenceAll').onclick=()=>action('all','zero');
 host.addEventListener('click',e=>{const b=e.target.closest('button[data-action]');if(b)action(b.dataset.target,b.dataset.action);});
 function disconnect(){
  latest=null;busy=true;$('openMotorConsole').disabled=true;message.textContent='Bridge disconnected. These are last readings; refresh after reconnecting.';
  for(const button of host.querySelectorAll('button'))button.disabled=true;
  $('motorReferenceAll').disabled=true;
  for(const badge of host.querySelectorAll('.motor-fresh')){badge.textContent='Last sample / disconnected';badge.classList.remove('live');}
 }
 return {render,disconnect,setRunning(running){if(latest){latest.simulator_running=running;render(latest);}}};
}
