import * as THREE from 'three';
import {TransformControls} from 'three/addons/controls/TransformControls.js';
import {HandViewer} from './viewer.js';

const COLORS = [0xce8869, 0xb5a149, 0x609c7a, 0x628fbe, 0xa773ae];

/** The same URDF viewer, with points in URDF base coordinates (metres). */
export class CalibrationViewer extends HandViewer {
  constructor(element, {editable=false, onSelect=()=>{}, onMove=()=>{}}={}) {
    super(element);
    Object.assign(this, {editable, onSelect, onMove, selected:0, directDrag:null});
    this.pointGroup = new THREE.Group(); this.scene.add(this.pointGroup);
    this.nodes = Array.from({length:21}, (_,i) => {
      const node = new THREE.Mesh(new THREE.SphereGeometry(.0026,12,8),
        new THREE.MeshBasicMaterial({color:i ? COLORS[Math.floor((i-1)/4)] : 0x477f93, depthTest:false}));
      node.renderOrder = 5; node.userData.index = i; this.pointGroup.add(node);
      const canvas = document.createElement('canvas'); canvas.width=64; canvas.height=48;
      const ctx = canvas.getContext('2d'); ctx.font='bold 28px sans-serif'; ctx.textAlign='center';
      ctx.strokeStyle='white'; ctx.lineWidth=6; ctx.strokeText(String(i),32,33);
      ctx.fillStyle='#254b5b'; ctx.fillText(String(i),32,33);
      const texture = new THREE.CanvasTexture(canvas);
      const label = new THREE.Sprite(new THREE.SpriteMaterial({map:texture,depthTest:false,transparent:true}));
      label.scale.set(.010,.0075,1); label.position.set(.003,0,.005); label.renderOrder=6; node.add(label);
      return node;
    });
    this.lines = COLORS.map(color => {
      const geometry=new THREE.BufferGeometry();
      geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(15),3));
      const line=new THREE.Line(geometry,new THREE.LineBasicMaterial({color,depthTest:false}));
      line.frustumCulled=false; line.renderOrder=4; this.pointGroup.add(line); return line;
    });
    this.target = new THREE.Mesh(new THREE.SphereGeometry(.0033,16,10),
      new THREE.MeshBasicMaterial({color:0xa64cc8,wireframe:true,depthTest:false}));
    this.target.renderOrder=8; this.target.visible=false; this.scene.add(this.target);
    this.pointGroup.visible=false;
    this.transform=new TransformControls(this.camera,this.renderer.domElement);
    this.transform.setSize(.65); this.transform.setMode('translate');
    this.scene.add(this.transform.getHelper());
    this.transform.addEventListener('dragging-changed', event=>{this.controls.enabled=!event.value;});
    this.transform.addEventListener('objectChange',()=>{
      if(this.editable) this.onMove(this.selected,this.nodes[this.selected].position.toArray());
    });
    this.raycaster=new THREE.Raycaster(); this.pointer=new THREE.Vector2();
    this.listeners={pointerdown:e=>this.pointerDown(e),pointermove:e=>this.pointerMove(e),pointerup:e=>this.pointerUp(e),pointercancel:e=>this.pointerUp(e)};
    for(const [event,handler] of Object.entries(this.listeners)) this.renderer.domElement.addEventListener(event,handler,true);
  }
  async load(side,model) {
    const description=await super.load(side,model);
    return description;
  }
  zeroPose() {
    // A3.2 mechanical zero can be outside solve limits; the calibration file
    // stores link-local points at URDF q=0, so do not clamp this static pose.
    for(const joint of this.robot.joints.values()) joint.motion.quaternion.identity();
    this.robot.root.updateMatrixWorld(true);
  }
  setEditable(value) {
    if(this.editable===value)return;
    this.editable=value; this.directDrag=null; this.controls.enabled=true;
    if(value && this.pointGroup.visible) this.transform.attach(this.nodes[this.selected]);
    else this.transform.detach();
  }
  setPoints(points) {
    this.pointGroup.visible=!!points;
    if(!points) {this.transform.detach();return;}
    this.nodes.forEach((node,i)=>node.position.fromArray(points[i]));
    this.lines.forEach((line,finger)=>{
      const position=line.geometry.attributes.position;
      [0,1+4*finger,2+4*finger,3+4*finger,4+4*finger].forEach((index,j)=>position.setXYZ(j,...points[index]));
      position.needsUpdate=true;
    });
    if(this.editable) this.transform.attach(this.nodes[this.selected]);
  }
  select(index) {
    this.nodes[this.selected].scale.setScalar(1); this.selected=index; this.nodes[index].scale.setScalar(1.2);
    if(this.editable) this.transform.attach(this.nodes[index]);
  }
  setBindings(rows) {
    this.transform.setSpace('local');
    rows.forEach(row=>this.robot.links.get(row.link).getWorldQuaternion(this.nodes[row.id].quaternion));
  }
  setTarget(point) {this.target.visible=!!point;if(point)this.target.position.fromArray(point);}
  worldPoint(row) {return this.robot.links.get(row.link).localToWorld(new THREE.Vector3(...row.local_xyz)).toArray();}
  localPoint(link, point) {return this.robot.links.get(link).worldToLocal(new THREE.Vector3(...point)).toArray();}
  cast(event) {
    const bounds=this.renderer.domElement.getBoundingClientRect();
    this.pointer.set((event.clientX-bounds.left)/bounds.width*2-1,1-(event.clientY-bounds.top)/bounds.height*2);
    this.scene.updateMatrixWorld(true); this.raycaster.setFromCamera(this.pointer,this.camera);
  }
  pointerDown(event) {
    if(!this.editable || event.button!==0 || this.transform.dragging)return;
    this.cast(event);
    const hit=this.raycaster.intersectObjects(this.nodes,false)[0]; if(!hit)return;
    event.stopImmediatePropagation(); event.preventDefault();
    this.select(hit.object.userData.index);this.onSelect(this.selected);
    const point=this.nodes[this.selected].position;
    const plane=new THREE.Plane().setFromNormalAndCoplanarPoint(this.camera.getWorldDirection(new THREE.Vector3()),point);
    const intersection=this.raycaster.ray.intersectPlane(plane,new THREE.Vector3());
    if(!intersection)return;
    this.directDrag={plane,offset:point.clone().sub(intersection),pointerId:event.pointerId};
    this.controls.enabled=false; this.renderer.domElement.setPointerCapture(event.pointerId);
  }
  pointerMove(event) {
    if(!this.directDrag)return;
    event.stopImmediatePropagation(); this.cast(event);
    let point;
    if(this.snapLink) {
      const meshes=this.robot.links.get(this.snapLink)?.children.filter(node=>node.isMesh)||[];
      point=this.raycaster.intersectObjects(meshes,false)[0]?.point;
      if(!point)return;
    } else {
      point=this.raycaster.ray.intersectPlane(this.directDrag.plane,new THREE.Vector3());
      if(point)point.add(this.directDrag.offset);
    }
    if(point)this.onMove(this.selected,point.toArray());
  }
  pointerUp(event) {
    if(!this.directDrag)return;
    event.stopImmediatePropagation();
    if(this.renderer.domElement.hasPointerCapture(event.pointerId))this.renderer.domElement.releasePointerCapture(event.pointerId);
    this.directDrag=null;this.controls.enabled=true;
  }
  dispose() {
    for(const [event,handler] of Object.entries(this.listeners))this.renderer.domElement.removeEventListener(event,handler,true);
    for(const node of this.nodes)node.children[0].material.map.dispose();
    this.transform.dispose();super.dispose();
  }
}
