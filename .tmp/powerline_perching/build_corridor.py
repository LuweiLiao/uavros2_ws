"""Generate a full-size corridor using the repository's existing tower CAD mesh."""
import copy
import math
import struct
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

WS = Path('/home/llw/Projects/uavros2_ws')
PKG = WS/'src/uav_simulator/uav_gazebo'
ROOT = PKG/'models/powerline_perching/models/powerline_corridor'
ROOT.mkdir(parents=True, exist_ok=True)
(ROOT/'meshes').mkdir(exist_ok=True)

# Split the existing CAD mesh by its insulator bounds, preserving every facet.
source = PKG/'models/tsd_model/tsd_electric_tower/models/tsd_electric_tower_base/meshes/base.STL'
facets = np.fromfile(source, dtype=np.dtype([('normal','<f4',3),('v','<f4',(3,3)),('attr','<u2')]), offset=84)
v=facets['v'];z=v[:,:,2]+3.7708724
ceramic=(abs(v[:,:,0]).max(1)<.4)&(abs(v[:,:,1]).min(1)>3.05)&(((z.min(1)>12.28)&(z.max(1)<13.30))|((z.min(1)>14.78)&(z.max(1)<15.80)))
assert 0 < ceramic.sum() < len(facets)
for name, selected in [('steel',~ceramic),('insulators',ceramic)]:
    data=facets[selected]
    (ROOT/'meshes'/f'{name}.stl').write_bytes(b'Derived from repository tsd_electric_tower_base/base.STL'.ljust(80,b' ')+struct.pack('<I',len(data))+data.tobytes())

def elem(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, attrs)
    if text is not None:
        e.text = str(text)
    return e

def write(root,path):
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)

sdf = ET.Element('sdf', version='1.10')
model = elem(sdf,'model',name='powerline_corridor')
elem(model,'static','true')
link = elem(model,'link',name='corridor')

def shape(name, geom, pose=(0,0,0,0,0,0), color='.55 .58 .6 1', collision=True, owner=link):
    v=elem(owner,'visual',name=name)
    elem(v,'pose',' '.join(map(str,pose)))
    g=elem(v,'geometry');g.append(geom)
    m=elem(v,'material');elem(m,'ambient',color);elem(m,'diffuse',color)
    elem(m,'specular','.25 .25 .25 1')
    if collision:
        c=elem(owner,'collision',name=name+'_collision')
        elem(c,'pose',' '.join(map(str,pose)));c.append(copy.deepcopy(g))
    return v

def box(name,xyz,size,color='.55 .58 .6 1',collision=True):
    g=ET.Element('box');elem(g,'size',' '.join(map(str,size)))
    return shape(name,g,(*xyz,0,0,0),color,collision)

def cylinder(name,a,b,r,color='.6 .64 .68 1',collision=True):
    d=[b[i]-a[i] for i in range(3)];length=math.sqrt(sum(v*v for v in d))
    pitch=math.acos(max(-1,min(1,d[2]/length)));yaw=math.atan2(d[1],d[0])
    g=ET.Element('cylinder');elem(g,'radius',r);elem(g,'length',length)
    return shape(name,g,(*( (a[i]+b[i])/2 for i in range(3)),0,pitch,yaw),color,collision)

for tower,x in enumerate((-50,50)):
    for part,color in [('steel','.48 .53 .58 1'),('insulators','.16 .42 .36 1')]:
        mesh=ET.Element('mesh');elem(mesh,'uri',f'model://powerline_corridor/meshes/{part}.stl')
        shape(f'tower_{tower}_{part}',mesh,(x,0,3.7708724,0,0,0),color,False)
    # Conservative stepped envelopes keep the lattice non-traversable in DART.
    for i in range(12):
        half=3.77-.245*i
        c=elem(link,'collision',name=f'tower_{tower}_envelope_{i}')
        elem(c,'pose',f'{x} 0 {i+.5} 0 0 0')
        g=elem(c,'geometry');b=elem(g,'box');elem(b,'size',f'{half*2} {half*2} 1')
    for i,zarm in enumerate((13.65,16.15)):
        c=elem(link,'collision',name=f'tower_{tower}_arm_{i}')
        elem(c,'pose',f'{x} 0 {zarm} 0 0 0')
        g=elem(c,'geometry');b=elem(g,'box');elem(b,'size','1.5 7.1 .45')
    c=elem(link,'collision',name=f'tower_{tower}_upper_core')
    elem(c,'pose',f'{x} 0 14.8 0 0 0')
    g=elem(c,'geometry');b=elem(g,'box');elem(b,'size','1.5 1.5 5.6')
    for i,(dx,dy) in enumerate(((-3.6,-3.6),(-3.6,3.6),(3.6,-3.6),(3.6,3.6))):
        box(f'footing_{tower}_{i}',(x+dx,dy,.05),(1.1,1.1,.55),'.56 .56 .53 1')
    box(f'plate_{tower}',(x,-2.4,5),(1.0,.025,.6),'.18 .27 .31 1',False)

# Four suspended conductors follow the existing CAD tower's two crossarm levels.
# Exact catenary: 1.25 m sag over 100 m; all cylinders carry collision geometry.
a=1000.0
sag=a*(math.cosh(50/a)-1)
for level,attach in enumerate((12.287,14.787)):
    for side,y in enumerate((-3.35,3.35)):
        tag=f'phase_{level}_{side}'
        for tower,x in enumerate((-50,50)):
            cylinder(f'{tag}_core_{tower}',(x,y,attach),(x,y,attach+1.1),.035,'.36 .39 .42 1')
            box(f'{tag}_clamp_{tower}',(x,y,attach),(.46,.10,.10),'.30 .33 .36 1')
            dx=.8 if x<0 else -.8
            zd=attach-sag+a*(math.cosh((x+dx)/a)-1)
            cylinder(f'{tag}_damper_stem_{tower}',(x+dx,y,zd),(x+dx,y,zd-.19),.012,collision=False)
            cylinder(f'{tag}_damper_{tower}',(x+dx-.16,y,zd-.19),(x+dx+.16,y,zd-.19),.045,'.26 .28 .3 1',False)
        for i in range(100):
            x0,x1=-50+i,-49+i
            z0=attach-sag+a*(math.cosh(x0/a)-1)
            z1=attach-sag+a*(math.cosh(x1/a)-1)
            cylinder(f'{tag}_{i}',(x0,y,z0),(x1,y,z1),.012,'.63 .66 .69 1')

for i in range(100):
    x0,x1=-50+i,-49+i
    z0=17.55-.75+1666.7*(math.cosh(x0/1666.7)-1)
    z1=17.55-.75+1666.7*(math.cosh(x1/1666.7)-1)
    cylinder(f'earth_wire_{i}',(x0,0,z0),(x1,0,z1),.008,'.40 .43 .46 1')

# Spatially partition static collisions so DART can reject distant span sections.
wire_meshes={}
for visual in list(link.findall('visual')):
    name=visual.get('name')
    parts=name.split('_')
    is_phase=len(parts)==4 and parts[0]=='phase' and parts[-1].isdigit()
    if not (is_phase or name.startswith('earth_wire_')):
        continue
    key='_'.join(parts[:-1])
    x,y,z,roll,pitch,yaw=map(float,visual.findtext('pose').split())
    radius=float(visual.findtext('geometry/cylinder/radius'))
    length=float(visual.findtext('geometry/cylinder/length'))
    axis=np.array([math.sin(pitch)*math.cos(yaw),math.sin(pitch)*math.sin(yaw),math.cos(pitch)])
    u=np.cross(axis,[0,0,1]);u/=np.linalg.norm(u);v=np.cross(axis,u)
    center=np.array([x,y,z]);rings=[]
    for sign in (-1,1):
        rings.append([center+sign*length/2*axis+radius*(math.cos(t)*u+math.sin(t)*v) for t in np.arange(12)*math.tau/12])
    triangles=wire_meshes.setdefault(key,[])
    for i in range(12):
        j=(i+1)%12
        triangles.extend([(rings[0][i],rings[0][j],rings[1][j]),(rings[0][i],rings[1][j],rings[1][i]),
                          (center-length/2*axis,rings[0][j],rings[0][i]),(center+length/2*axis,rings[1][i],rings[1][j])])
    link.remove(visual)
for key,triangles in wire_meshes.items():
    data=np.zeros(len(triangles),dtype=facets.dtype)
    data['v']=triangles
    normal=np.cross(data['v'][:,1]-data['v'][:,0],data['v'][:,2]-data['v'][:,0])
    normal/=np.linalg.norm(normal,axis=1)[:,None];data['normal']=normal
    (ROOT/'meshes'/f'{key}.stl').write_bytes(b'Generated catenary visual mesh'.ljust(80,b' ')+struct.pack('<I',len(data))+data.tobytes())
    mesh=ET.Element('mesh');elem(mesh,'uri',f'model://powerline_corridor/meshes/{key}.stl')
    shape(key,mesh,color='.63 .66 .69 1' if key.startswith('phase') else '.40 .43 .46 1',collision=False)
groups={}
for collision in list(link.findall('collision')):
    x=float(collision.findtext('pose').split()[0])
    group=math.floor((x+50)/5)
    if group not in groups:
        groups[group]=elem(model,'link',name=f'span_collision_{group+10}')
    link.remove(collision)
    groups[group].append(collision)
write(sdf,ROOT/'model.sdf')
config=ET.Element('model');elem(config,'name','Powerline corridor');elem(config,'version','1.0')
elem(config,'sdf','model.sdf',version='1.10')
elem(config,'description','100 m full-scale corridor, existing tower CAD mesh, catenary wires and suspension hardware. Static conceptual geometry; not an electrical design.')
write(config,ROOT/'model.config')

world=ET.parse(PKG/'worlds/powerline_perching_flight.world')
w=world.getroot().find('world');w.set('name','powerline_corridor')
w.find('scene/ambient').text='.45 .45 .45 1'
for m in list(w.findall('model')):
    if m.get('name')!='ground':w.remove(m)
ground=w.find("model[@name='ground']")
for size in ground.findall('.//plane/size'):size.text='260 180'
for material in ground.findall('.//material'):
    material.find('ambient').text='.25 .34 .22 1';material.find('diffuse').text='.25 .34 .22 1'
include=elem(w,'include');elem(include,'uri','model://powerline_corridor')
gui=elem(w,'gui',fullscreen='false')
for filename in ('MinimalScene','GzSceneManager','InteractiveViewControl','CameraTracking','Screenshot'):
    p=elem(gui,'plugin',filename=filename,name=filename)
    settings=elem(p,'gz-gui')
    elem(settings,'property','false',key='showTitleBar',type='bool')
    elem(settings,'property','docked' if filename=='MinimalScene' else 'floating',key='state',type='string')
    if filename=='MinimalScene':
        elem(p,'engine','ogre2');elem(p,'scene','scene');elem(p,'camera_pose','65 -80 38 0 .25 2.25')
    else:
        elem(settings,'property','5',key='width',type='double');elem(settings,'property','5',key='height',type='double')
write(world.getroot(),PKG/'worlds/powerline_corridor.world')
vehicle=ET.parse(PKG/'models/powerline_perching/models/powerline_perching/flight.sdf').getroot().find('model')
vehicle.find(".//sensor[@type='rgbd_camera']/update_rate").text='10'
elem(vehicle,'pose','0 -4.35 0.174 0 0 0')
w.append(vehicle)
write(world.getroot(),PKG/'worlds/powerline_corridor_flight.world')
print('Generated',ROOT, 'sag',sag)
