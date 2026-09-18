from pathlib import Path
import xml.etree.ElementTree as E

ws = Path('/home/llw/Projects/uavros2_ws')
folder = ws / 'src/uav_simulator/uav_gazebo'
tree = E.parse(folder / 'worlds/powerline_perching.world')
w = tree.getroot().find('world')
sensors = E.SubElement(w, 'plugin', filename='gz-sim-sensors-system', name='gz::sim::systems::Sensors')
E.SubElement(sensors, 'render_engine').text = 'ogre2'
include = w.find('include')
w.remove(include)
model = E.parse(folder / 'models/powerline_perching/models/powerline_perching/flight.sdf').getroot().find('model')
E.SubElement(model, 'pose').text = '0 -1 0.174 0 0 0'
w.append(model)
E.indent(tree, space='  ')
tree.write(folder / 'worlds/powerline_perching_flight.world', encoding='utf-8', xml_declaration=True)
