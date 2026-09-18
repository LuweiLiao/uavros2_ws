#!/usr/bin/env python3
"""Generate the user-requested new design, not a replacement ROS 1 migration.
SI units; +X follows the wire, +Y left, +Z up. No third-party dependencies.
Box mass properties are rotated and combined with the parallel-axis theorem.
"""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as E

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'models' / 'powerline_perching'
OUT.mkdir(parents=True, exist_ok=True)
COLOR = {'frame': '.12 .16 .20 1', 'panel': '.12 .48 .78 1',
         'battery': '.18 .43 .28 1', 'rotor': '.07 .08 .10 1',
         'saddle': '.92 .58 .12 1'}
PARTS = []

def part(name, xyz, size, mass, color='frame', roll=0):
    PARTS.append(dict(name=name, xyz=xyz, size=size, mass=mass, color=color, roll=roll))

# Masses are engineering assumptions, not measured hardware properties.
for sign, side in [(1, 'left'), (-1, 'right')]:
    part(side+'_body', [0,sign*.22,0], [.48,.09,.075], .38)
    part(side+'_battery', [0,sign*.22,-.0825], [.22,.078,.09], .42, 'battery')
    # Flare edges: y=+/-0.17 at z=-0.035 to +/-0.02 at z=0.24.
    dy, dz = sign*(.02-.17), .24-(-.035)
    length = math.hypot(dy,dz)
    angle = -math.atan2(dy,dz)
    part(side+'_flare', [0,sign*.095,.1025], [.40,.008,length], .12, 'panel',angle)
    part(side+'_throat', [0,sign*.020,.32], [.40,.008,.16], .07, 'panel')
    for x, suffix in [(.26,'front'),(-.26,'rear')]:
        part(side+'_'+suffix+'_arm',[x,sign*.295,.015],[.055,.23,.035],.055)
        part(side+'_'+suffix+'_motor',[x,sign*.37,.045],[.045,.045,.045],.07)
    # Longitudinal skids and lower rails remain outside the central passage.
    part(side+'_skid',[0,sign*.22,-.155],[.48,.025,.035],.055)
    for x,suffix in [(.18,'front'),(-.18,'rear')]:
        part(side+'_'+suffix+'_mount',[x,sign*.178,-.007],[.035,.09,.04],.025)
# Only the raised roof bridges the left and right halves.
part('saddle',[0,0,.410],[.42,.056,.020],.13,'saddle')
part('camera_mast',[.18,.08,.505],[.025,.025,.19],.015)
part('camera_mount',[.18,.044,.423],[.025,.10,.012],.003)
part('camera_boom',[.25,.04,.61],[.18,.10,.018],.018)
part('camera_body',[.32,0,.59],[.05,.045,.035],.035)
ROTORS = [( .26,-.37,'ccw'),(-.26,-.37,'cw'),(-.26,.37,'ccw'),(.26,.37,'cw')]
ROTOR_MASS, ROTOR_RADIUS, ROTOR_Z = .018, .145, .079
WIRE_RADIUS, SADDLE_Z = .012, .400


def fmt(v):
    return ' '.join(f'{x:.10g}' for x in v)

def add(p, tag, text=None, **attrs):
    e=E.SubElement(p,tag,attrs)
    if text is not None: e.text=str(text)
    return e

def props(parts):
    mass=sum(p['mass'] for p in parts)
    cg=[sum(p['mass']*p['xyz'][i] for p in parts)/mass for i in range(3)]
    inertia=[[0.]*3 for _ in range(3)]
    for p in parts:
        m=p['mass']; a,b,c=p['size']; angle=p['roll']
        local=[m*(b*b+c*c)/12,m*(a*a+c*c)/12,m*(a*a+b*b)/12]
        co,si=math.cos(angle),math.sin(angle)
        rotation=[[1,0,0],[0,co,-si],[0,si,co]]
        d=[p['xyz'][i]-cg[i] for i in range(3)]
        for i in range(3):
            for j in range(3):
                inertia[i][j]+=sum(rotation[i][k]*local[k]*rotation[j][k] for k in range(3))+m*((sum(x*x for x in d) if i==j else 0)-d[i]*d[j])
    return mass,cg,inertia

def sdf_inertial(link,m,cg,I):
    n=add(link,'inertial');add(n,'pose',fmt(cg+[0,0,0]));add(n,'mass',m)
    n=add(n,'inertia')
    for name,i,j in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]:add(n,name,f'{I[i][j]:.12g}')

def shape(link,p):
    for kind in ['visual','collision']:
        n=add(link,kind,name=p['name']+'_'+kind)
        add(n,'pose',fmt(p['xyz']+[p['roll'],0,0]));add(add(add(n,'geometry'),'box'),'size',fmt(p['size']))
        if kind=='visual':
            mat=add(n,'material');add(mat,'ambient',COLOR[p['color']]);add(mat,'diffuse',COLOR[p['color']])
        else:
            surface=add(n,'surface');fr=add(add(surface,'friction'),'ode');add(fr,'mu',.65);add(fr,'mu2',.65)

def write(path,root):
    E.indent(root,space='  ');E.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)

sdf=E.Element('sdf',version='1.10');model=add(sdf,'model',name='powerline_perching');add(model,'self_collide','false')
base=add(model,'link',name='base_link');mass,cg,I=props(PARTS);sdf_inertial(base,mass,cg,I)
for p in PARTS:shape(base,p)
imu=add(base,'sensor',name='imu',type='imu');add(imu,'always_on','true');add(imu,'update_rate',200);add(imu,'topic','/powerline_perching/imu');add(imu,'imu')
camera=add(base,'sensor',name='down_camera',type='rgbd_camera')
add(camera,'pose','0.32 0 0.568 0 1.57079632679 0')
add(camera,'always_on','true');add(camera,'update_rate',20)
add(camera,'topic','/powerline_perching/down_camera')
cam=add(camera,'camera');add(cam,'horizontal_fov',1.3)
im=add(cam,'image');add(im,'width',640);add(im,'height',480);add(im,'format','R8G8B8')
clip=add(cam,'clip');add(clip,'near',.04);add(clip,'far',12)
depth=add(cam,'depth_camera');clip=add(depth,'clip');add(clip,'near',.04);add(clip,'far',12)
contact=add(base,'sensor',name='saddle_contact',type='contact');add(contact,'always_on','true');add(contact,'update_rate',100);add(contact,'topic','/powerline_perching/contact')
ct=add(contact,'contact');add(ct,'topic','/powerline_perching/contact')
for p in PARTS:
    if any(s in p['name'] for s in ['saddle','flare','throat']):add(ct,'collision',p['name']+'_collision')
for i,(x,y,direction) in enumerate(ROTORS):
    link=add(model,'link',name=f'rotor_{i}');add(link,'pose',fmt([x,y,ROTOR_Z,0,0,0]))
    # Rotor inertia corresponds to the visible two-blade equivalent box.
    blade=dict(name=f'rotor_{i}',xyz=[0,0,0],size=[2*ROTOR_RADIUS,.024,.004],mass=ROTOR_MASS,color='rotor',roll=0)
    rm,rc,ri=props([blade]);sdf_inertial(link,rm,rc,ri);shape(link,blade)
    joint=add(model,'joint',name=f'rotor_{i}_joint',type='revolute');add(joint,'parent','base_link');add(joint,'child',f'rotor_{i}')
    ax=add(joint,'axis');add(ax,'xyz','0 0 1');lim=add(ax,'limit');add(lim,'lower',-1e16);add(lim,'upper',1e16)
    add(add(ax,'dynamics'),'damping',1e-6)
    plugin=add(model,'plugin',name=f'prop_{i}_plugin',filename='librotors_gazebo_motor_model.so')
    fields=dict(robotNamespace='powerline_perching',jointName=f'rotor_{i}_joint',linkName=f'rotor_{i}',turningDirection=direction,
        timeConstantUp=.0125,timeConstantDown=.025,maxRotVelocity=900,motorConstant=1.8e-5,momentConstant=.016,
        commandSubTopic='command/motor_speed',motorNumber=i,rotorDragCoefficient=.0001,rollingMomentCoefficient=1e-6,
        motorSpeedPubTopic=f'motor_speed/motor_{i}',rotorVelocitySlowdownSim=10)
    for key,value in fields.items():add(plugin,key,value)
odom=add(model,'plugin',filename='gz-sim-odometry-publisher-system',name='gz::sim::systems::OdometryPublisher')
for k,v in dict(odom_frame='world',robot_base_frame='base_link',odom_publish_frequency=50,dimensions=3,odom_topic='/powerline_perching/odometry').items():add(odom,k,v)
write(OUT/'model.sdf',sdf)
# Keep the contact-only model usable without opening SITL UDP ports.
import copy
flight=copy.deepcopy(sdf)
# ArduRotorNormPlugin forwards raw IMU vectors; SITL expects FRD, not FLU.
add(flight.find("model/link/sensor[@name='imu']"),'pose','0 0 0 3.14159265359 0 0')
flight.find("model/link/sensor[@name='imu']/update_rate").text='1000'
ap=add(flight.find('model'),'plugin',name='ArduRotorNormPlugin',filename='libArduRotorNormPlugin.so')
for key,value in dict(fdm_addr='127.0.0.1',fdm_port_in=9002,fdm_port_out=9003,
        modelXYZToAirplaneXForwardZDown='0 0 0 3.14159265359 0 0',
        gazeboXYZToNED='0 0 0 3.14159265359 0 0',motor_num=4,servo_num=0,
        motor_pub='/powerline_perching/command/motor_speed',imuName='base_link::imu',
        connectionTimeoutMaxCount=3).items():add(ap,key,value)
write(OUT/'flight.sdf',flight)
world_path=ROOT.parent.parent/'worlds'
flight_world=E.parse(world_path/'powerline_perching.world')
world=flight_world.getroot().find('world')
world.remove(world.find('include'))
vehicle=copy.deepcopy(flight.find('model'))
add(vehicle,'pose','0 -1 0.174 0 0 0')
world.append(vehicle)
write(world_path/'powerline_perching_flight.world',flight_world.getroot())
config=E.Element('model');add(config,'name','powerline_perching');add(config,'version','0.1.0');add(config,'sdf','model.sdf',version='1.10');add(config,'description','Raised flared saddle quadrotor; new ROS 2 concept, SI units.');write(OUT/'model.config',config)

# Same geometry and physical properties for robot_state_publisher / RViz.
robot=E.Element('robot',name='powerline_perching')
for key,value in COLOR.items():add(add(robot,'material',name=key),'color',rgba=value)
def urdf_link(name,parts):
    link=add(robot,'link',name=name);m,c,I=props(parts);n=add(link,'inertial');add(n,'origin',xyz=fmt(c),rpy='0 0 0');add(n,'mass',value=str(m))
    add(n,'inertia',**{name:f'{I[i][j]:.12g}' for name,i,j in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]})
    for p in parts:
        for kind in ['visual','collision']:
            n=add(link,kind,name=p['name']+'_'+kind);add(n,'origin',xyz=fmt(p['xyz']),rpy=fmt([p['roll'],0,0]));add(add(n,'geometry'),'box',size=fmt(p['size']))
            if kind=='visual':add(n,'material',name=p['color'])
    return link
urdf_link('base_link',PARTS)
for i,(x,y,_) in enumerate(ROTORS):
    urdf_link(f'rotor_{i}',[dict(name=f'rotor_{i}',xyz=[0,0,0],size=[2*ROTOR_RADIUS,.024,.004],mass=ROTOR_MASS,color='rotor',roll=0)])
    j=add(robot,'joint',name=f'rotor_{i}_joint',type='continuous');add(j,'parent',link='base_link');add(j,'child',link=f'rotor_{i}');add(j,'origin',xyz=fmt([x,y,ROTOR_Z]),rpy='0 0 0');add(j,'axis',xyz='0 0 1')
write(OUT/'model.urdf',robot)
total=mass+4*ROTOR_MASS;total_cg=[mass*cg[0]/total,mass*cg[1]/total,(mass*cg[2]+4*ROTOR_MASS*ROTOR_Z)/total]
report=dict(total_mass_kg=total,center_of_mass_m=total_cg,saddle_underside_z_m=SADDLE_Z,wire_radius_m=WIRE_RADIUS,
    support_to_com_vertical_m=SADDLE_Z-WIRE_RADIUS-total_cg[2],throat_clear_width_m=.032,entry_clear_width_m=.333,
    saddle_length_m=.42,rotor_radius_m=ROTOR_RADIUS,motor_positions_m=[[x,y,ROTOR_Z] for x,y,_ in ROTORS],
    wire_to_rotor_swept_clearance_m=.37-ROTOR_RADIUS-WIRE_RADIUS,hover_speed_rad_s=math.sqrt(total*9.81/(4*1.8e-5)),
    estimated_max_thrust_weight_ratio=4*1.8e-5*900**2/(total*9.81),base_inertia_kg_m2=I,
    assumptions='Concept masses, rigid cable, no latch; simulation-only visual guidance, not hardware measurements.')
(ROOT/'design_parameters.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
