"""Pre-tessellate the source STEP, preserving XCAF assembly names and placements.
Usage: python convert_cad.py source.STEP dist
Requires cadquery (OCP) and numpy. No CAD parser is needed in the browser.
"""
import sys,json,time
from pathlib import Path
import numpy as np
import cadquery as cq
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence,TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location

source,out=sys.argv[1],Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
r=STEPCAFControl_Reader();r.SetNameMode(True)
d=TDocStd_Document(TCollection_ExtendedString('cad'))
print('Reading STEP',flush=True);r.ReadFile(source);r.Transfer(d)
st=XCAFDoc_DocumentTool.ShapeTool_s(d.Main());roots=TDF_LabelSequence();st.GetFreeShapes(roots)
parts=[];chunks=[];offset=0
def name(l):
    a=TDataStd_Name()
    return a.Get().ToExtString() if l.FindAttribute(TDataStd_Name.GetID_s(),a) else 'Part'
def walk(label,loc,path):
    global offset
    ref=TDF_Label()
    if st.IsReference_s(label):
        st.GetReferredShape_s(label,ref)
        loc=loc.Multiplied(st.GetLocation_s(label)); label=ref
    path=path+[name(label)]
    children=TDF_LabelSequence();st.GetComponents_s(label,children)
    if children.Length():
        for i in range(1,children.Length()+1):walk(children.Value(i),loc,path)
        return
    shape=cq.Shape(st.GetShape_s(label).Moved(loc))
    for si,solid in enumerate(shape.Solids()):
        vs,ts=solid.tessellate(.28,.28)
        if not ts:continue
        v=np.array([[p.x,p.y,p.z] for p in vs],dtype=np.float32)
        # Source is Y-up, toes +Z. Display: forward X, vertical Y, lateral Z.
        v=v[:,[2,1,0]].copy();v[:,2]*=-1
        ix=np.array(ts,dtype=np.uint32).flatten()
        vb=v.astype('<f4').tobytes();ib=ix.astype('<u4').tobytes()
        part={'id':len(parts),'name':path[-1],'path':' / '.join(path),'solid':si,
              'vertexOffset':offset,'vertexCount':len(v),'indexOffset':offset+len(vb),'indexCount':len(ix),
              'min':v.min(0).round(3).tolist(),'max':v.max(0).round(3).tolist()}
        chunks.extend([vb,ib]);offset+=len(vb)+len(ib);parts.append(part)
        print(part['id'],part['path'],len(ix)//3,part['min'],part['max'],flush=True)
for i in range(1,roots.Length()+1):walk(roots.Value(i),TopLoc_Location(),[])
(out/'cad.bin').write_bytes(b''.join(chunks))
(out/'cad.json').write_text(json.dumps({'source':Path(source).name,'units':'mm','parts':parts,'triangles':sum(p['indexCount']//3 for p in parts)},separators=(',',':')))
print('DONE',len(parts),'solids',offset,'bytes',flush=True)
