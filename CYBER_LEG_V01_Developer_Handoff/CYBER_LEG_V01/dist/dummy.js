import * as T from './three.module.min.js';
import {gait,clamp,rad} from './motion.js';

// Geometric two-link limb, with a forward knee. No dynamics or balance model.
export function legIK(hip,ankle,upper=420,lower=420){
 const delta=ankle.clone().sub(hip),raw=delta.length(),d=clamp(raw,Math.abs(upper-lower)+.001,upper+lower-.001);
 const axis=raw>1e-9?delta.divideScalar(raw):new T.Vector3(0,-1,0);
 const end=hip.clone().addScaledVector(axis,d);
 const along=(upper*upper-lower*lower+d*d)/(2*d),bend=Math.sqrt(Math.max(0,upper*upper-along*along));
 let forward=new T.Vector3(1,0,0).addScaledVector(axis,-axis.x);
 if(forward.lengthSq()<1e-8)forward.set(0,0,1);forward.normalize();
 return {knee:hip.clone().addScaledVector(axis,along).addScaledVector(forward,bend),ankle:end,limited:Math.abs(d-raw)>.01};
}

export class WalkingDummy{
 constructor(scene,root,kneePivot){
  this.root=root;this.scene=scene;this.materials=[];this.joints=[];
  const material=(color,alpha=1)=>{const m=new T.MeshStandardMaterial({color,transparent:true,opacity:.28*alpha,roughness:.65,metalness:.02,depthWrite:false,side:T.DoubleSide});m.userData.alpha=alpha;this.materials.push(m);return m;};
  const skin=material(0x9bd5d7),residual=material(0xe7ba8d,1.5),socketMat=material(0xffb368,1.3),jointMat=material(0xbbeaf0,.85);
  this.attached=new T.Group();root.add(this.attached);
  this.body=new T.Group();scene.add(this.body);
  const ellipsoid=(parent,radii,pos,mat=skin)=>{const m=new T.Mesh(new T.SphereGeometry(1,20,14),mat);m.scale.set(...radii);m.position.set(...pos);parent.add(m);return m;};
  const limb=(parent,radius,mat=skin)=>{const m=new T.Mesh(new T.CylinderGeometry(radius*.85,radius,1,16),mat);parent.add(m);return m;};
  this.segment=(mesh,a,b)=>{mesh.position.copy(a).add(b).multiplyScalar(.5);mesh.scale.y=a.distanceTo(b);mesh.quaternion.setFromUnitVectors(new T.Vector3(0,1,0),b.clone().sub(a).normalize());};
  const mountY=470.78-kneePivot.y;
  this.hipLocal=new T.Vector3(-22,395,0);
  // Socket bottom meets the existing rotor's top attachment plane.
  const shell=new T.Mesh(new T.CylinderGeometry(63,32,175,24,1,true),socketMat);shell.position.set(-22,mountY+87.5,0);this.attached.add(shell);
  const adapter=new T.Mesh(new T.CylinderGeometry(32,25,12,20),socketMat);adapter.position.set(-22,mountY+6,0);this.attached.add(adapter);
  ellipsoid(this.attached,[41,46,41],[-22,mountY+72,0],residual);
  const thigh=new T.Mesh(new T.CylinderGeometry(61,38,1,20),residual);this.attached.add(thigh);this.segment(thigh,new T.Vector3(-22,mountY+92,0),this.hipLocal);
  ellipsoid(this.attached,[62,62,62],this.hipLocal.toArray(),residual);
  // Body origin is the prosthetic-side hip; the opposite hip is 170 mm medial.
  ellipsoid(this.body,[90,95,125],[0,55,-85]);
  ellipsoid(this.body,[87,140,113],[0,190,-85]);
  ellipsoid(this.body,[106,195,168],[0,350,-85]);
  ellipsoid(this.body,[44,59,45],[0,548,-85]);
  ellipsoid(this.body,[81,108,79],[3,673,-85]);
  // Small nose makes the forward direction clear in side and front views.
  ellipsoid(this.body,[20,22,18],[77,679,-85]);
  this.arms=[];
  for(const z of [-280,110]){
   const shoulder=new T.Vector3(0,450,z);
   this.arms.push({shoulder,upper:limb(this.body,32),lower:limb(this.body,25),hand:ellipsoid(this.body,[23,42,27],[0,0,0]),elbow:ellipsoid(this.body,[28,28,28],[0,0,0],jointMat)});
   ellipsoid(this.body,[43,43,43],shoulder.toArray());
  }
  this.otherHip=new T.Vector3(0,0,-170);
  this.otherThigh=limb(this.body,51);this.otherShin=limb(this.body,33);
  this.otherKnee=ellipsoid(this.body,[38,38,38],[0,0,0],jointMat);
  this.otherFoot=ellipsoid(this.body,[112,34,43],[0,0,0]);
  this.socketBottom=new T.Vector3(-22,mountY,0);
 }
 setOpacity(value){for(const m of this.materials)m.opacity=Math.min(.85,value*m.userData.alpha);}
 setVisible(show){this.attached.visible=show;this.body.visible=show;}
 update(state,heightAt){
  this.root.updateMatrixWorld(true);
  const hipWorld=this.root.localToWorld(this.hipLocal.clone());
  this.body.position.copy(hipWorld);this.body.rotation.set(0,0,0);this.body.updateMatrixWorld(true);
  const walking=state.mode==='auto',phase=walking?(state.phase+.5)%1:0;
  const g=gait(phase),s=clamp((phase-.62)/.38,0,1);
  const x=walking?(g.stance?100-350*phase:-117+217*s*s*(3-2*s)):hipWorld.x+30;
  const z=hipWorld.z-170,ankleWorld=new T.Vector3(x,heightAt(x,z)+38+(walking?g.lift:0),z);
  const target=this.body.worldToLocal(ankleWorld.clone());
  const solved=legIK(this.otherHip,target);
  this.segment(this.otherThigh,this.otherHip,solved.knee);this.segment(this.otherShin,solved.knee,solved.ankle);
  this.otherKnee.position.copy(solved.knee);this.otherFoot.position.copy(solved.ankle).add(new T.Vector3(40,-7,0));
  this.otherFoot.rotation.z=walking&&!g.stance?8*Math.sin(Math.PI*s)*rad:0;
  for(const [i,a] of this.arms.entries()){
   const angle=(walking?18*Math.sin(state.phase*Math.PI*2+(i?Math.PI:0)):0)*rad;
   const elbow=a.shoulder.clone().add(new T.Vector3(Math.sin(angle)*265,-Math.cos(angle)*265,0));
   const hand=elbow.clone().add(new T.Vector3(Math.sin(angle+.25)*235,-Math.cos(angle+.25)*235,0));
   this.segment(a.upper,a.shoulder,elbow);this.segment(a.lower,elbow,hand);a.elbow.position.copy(elbow);a.hand.position.copy(hand);
  }
 }
 dispose(){
  this.root.remove(this.attached);this.scene.remove(this.body);
  for(const g of [this.attached,this.body])g.traverse(o=>{if(o.geometry)o.geometry.dispose();});
  for(const m of this.materials)m.dispose();
 }
}
