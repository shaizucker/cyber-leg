// Optional UI integration test. All CAN/serial devices are fake.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const {spawn}=require('node:child_process');
const {createInterface}=require('node:readline');
const {randomBytes}=require('node:crypto');
const {mkdir}=require('node:fs/promises');
const path=require('node:path');
const assert=require('node:assert/strict');
const root=path.resolve(__dirname,'..');
const token=randomBytes(16).toString('hex');
const script=`import sys
sys.path.insert(0,'drivers')
from knee_bridge import KneeBridge,HipBridge,BridgeServer,make_handler
from ankle_bridge import AnkleBridge
from motor_console import MotorConsole
from test_motor_console import ConsoleMotor,ConsoleActuators
k=ConsoleMotor();h=ConsoleMotor(True);a=ConsoleActuators()
b=KneeBridge(lambda:k,hip=HipBridge(lambda:h),ankle=AnkleBridge(lambda:a));MotorConsole(b)
s=BridgeServer(('127.0.0.1',0),make_handler(b,'${token}',0))
s.RequestHandlerClass=make_handler(b,'${token}',s.server_address[1])
print('READY:'+str(s.server_address[1]),flush=True)
try:s.serve_forever()
finally:b.close()
`;
const server=spawn(process.env.PYTHON || (process.platform==='win32'?'python':'python3'),['-u','-c',script],
 {cwd:root,windowsHide:true,stdio:['ignore','pipe','pipe']});
const ready=new Promise((resolve,reject)=>{
 const timer=setTimeout(()=>reject(Error('Fake server did not start')),10000);
 createInterface({input:server.stdout}).on('line',line=>{if(line.startsWith('READY:')){clearTimeout(timer);resolve(Number(line.slice(6)));}});
 server.once('error',e=>{clearTimeout(timer);reject(e);});
 server.stderr.on('data',data=>process.stderr.write(data));
});
(async()=>{
 let browser;
 try{
  const port=await ready;
  browser=await chromium.launch({headless:true,...(process.env.BROWSER_CHANNEL?{channel:process.env.BROWSER_CHANNEL}:{})});
  const page=await browser.newPage({viewport:{width:1440,height:1100}});
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  page.on('response',async r=>{if(r.url().includes('/api/')&&r.status()>=400)errors.push(`${r.status()} ${r.url()} ${await r.text()}`);});
  await page.goto(`http://127.0.0.1:${port}/`);
  await page.waitForFunction(()=>document.getElementById('loading').hidden,null,{timeout:60000});
  await page.locator('#openMotorConsole').click();
  assert.equal(await page.locator('#mc-hip-power').textContent(),'Unavailable');
  await page.locator('[data-target=hip][data-action=read]').click();
  await page.waitForFunction(()=>document.getElementById('mc-hip-cap').value==='1');
  await page.locator('#mc-hip-p').fill('12');
  await page.locator('[data-target=hip][data-action=pid]').click();
  await page.waitForFunction(()=>document.getElementById('motorConsoleMessage').textContent.startsWith('hip: settings applied'));
  await page.locator('[data-target=ankle][data-action=read]').click();
  await page.waitForFunction(()=>document.getElementById('mc-ankle-p').value==='1');
  await page.locator('#mc-ankle-p').fill('2');
  await page.locator('[data-target=ankle][data-action=pid]').click();
  await page.waitForFunction(()=>document.getElementById('motorConsoleMessage').textContent.startsWith('ankle: settings applied'));
  await page.locator('#motorReferenceAll').click();
  await page.waitForFunction(()=>document.getElementById('motorConsoleMessage').textContent.includes('captured as signed zero'));
  assert.ok(Number(await page.locator('#knee').getAttribute('min'))<0);
  assert.equal(await page.locator('#ext0').textContent(),'0.0');
  await page.locator('#closeMotorConsole').click();
  await page.locator('#start').click();await page.locator('#motorArm').click();
  await page.waitForFunction(()=>document.getElementById('hipStatus').textContent.includes('Hip armed'));
  await page.locator('#hip').fill('5');await page.locator('#openMotorConsole').click();
  assert.equal(await page.locator('[data-target=hip][data-action=read]').isDisabled(),true);
  assert.equal(await page.locator('[data-target=hip][data-action=pid]').isDisabled(),false);
  assert.equal(await page.locator('[data-target=ankle][data-action=pid]').isDisabled(),true);
  await page.locator('#mc-hip-p').fill('10');await page.locator('[data-target=hip][data-action=pid]').click();
  await page.waitForFunction(()=>document.getElementById('hipStatus').textContent.includes('P/D 10/4'));
  await page.locator('#closeMotorConsole').click();await page.locator('#stop').click();await page.locator('#motorDisarm').click();
  await page.waitForFunction(()=>document.getElementById('hipStatus').textContent.includes('Hip disarmed'));
  await page.locator('#openMotorConsole').click();
  await page.locator('#mc-hip-min').fill('-3');await page.locator('#mc-hip-max').fill('3');
  await page.locator('[data-target=hip][data-action=limits]').click();
  await page.waitForFunction(()=>document.getElementById('hip').max==='3');
  // Previously queued heartbeat payloads must use the new limits, not old 5° targets.
  await page.waitForTimeout(200);
  assert.ok(!/disconnected/i.test(await page.locator('#motorStatus').textContent()));
  assert.ok(Number(await page.locator('#hip').inputValue())<=3);
  assert.equal(await page.locator('#mc-a1-power').textContent(),'Unavailable');
  await mkdir(path.join(root,'tmp'),{recursive:true});
  await page.locator('#mc-hip').scrollIntoViewIfNeeded();
  await page.screenshot({path:path.join(root,'tmp/motor-console-rotary.png')});
  await page.locator('#mc-ankle').scrollIntoViewIfNeeded();
  await page.screenshot({path:path.join(root,'tmp/motor-console-ankle.png')});
  await page.setViewportSize({width:390,height:844});
  await page.locator('#motorConsole').evaluate(el=>el.scrollTop=0);
  await page.screenshot({path:path.join(root,'tmp/motor-console-mobile.png')});
  const bounds=await page.locator('#motorConsole').evaluate(el=>({client:el.clientWidth,scroll:el.scrollWidth}));
  assert.ok(bounds.scroll<=bounds.client+2,JSON.stringify(bounds));
  server.kill();
  await page.waitForFunction(()=>document.getElementById('mc-hip-fresh').textContent.includes('disconnected'));
  assert.equal(await page.locator('[data-target=hip][data-action=pid]').isDisabled(),true);
  assert.deepEqual(errors,[]);
  console.log('Motor console UI passed: PID, zero, signed values, live interlocks, new limit/heartbeat race, missing sensors and mobile layout.');
 }finally{if(browser)await browser.close();server.kill();}
})().catch(e=>{console.error(e);process.exitCode=1;});
