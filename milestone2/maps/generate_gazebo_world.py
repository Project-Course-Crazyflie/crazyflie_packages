#!/usr/bin/env python

"""
    @author: Daniel Duberg (dduberg@kth.se)
"""

from __future__ import print_function, unicode_literals
import xml.etree.cElementTree as ET
import math
import re
import numpy as np


def indent(elem, level=0):
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i


def add_wall(world, elem, index):
    model = None
    if 'gate' in elem:
        name = 'gate_' + str(elem['gate'])

        for m in world.iter('model'):
            if m.attrib['name'] == 'gate_' + str(elem['gate']):
                model = m
    else:
        name = 'wall_' + str(index)
        index = index + 1

    if not model:
        model = ET.SubElement(world, "model", name=name)
        ET.SubElement(model, "pose", frame="''").text = '0 0 0 0 0 0'

    link = None
    for l in model.iter('link'):
        link = l

    if not link:
        link = ET.SubElement(model, "link", name="link_0")

    ET.SubElement(link, "pose", frame="").text = '0 0 0 0 0 0'
    ET.SubElement(link, "gravity").text = "0"
    ET.SubElement(link, "self_collide").text = "0"
    ET.SubElement(link, "kinematic").text = "1"

    number = 0
    for v in link.iter('visual'):
        match = int(re.match('.*?([0-9]+)$', v.attrib['name']).group(1))
        if number <= match:
            number = match + 1

    visual = ET.SubElement(
        link, "visual", name=name + '_visual_' + str(number))
    ET.SubElement(visual, "pose",
                  frame="").text = str((elem['plane']['stop'][0] + elem['plane']['start'][0])/2) + ' ' + str((elem['plane']['stop'][1] + elem['plane']['start'][1])/2) + ' ' + str(
        (elem['plane']['stop'][2] + elem['plane']['start'][2])/2) + ' 0 0 ' + str(math.atan2(elem['plane']['stop'][1] - elem['plane']['start'][1], elem['plane']['stop'][0] - elem['plane']['start'][0]))
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = str(math.hypot(elem['plane']['stop'][0] - elem['plane']['start'][0], elem['plane']
                                                     ['stop'][1] - elem['plane']['start'][1])) + ' 0.02 ' + str(math.fabs((elem['plane']['stop'][2] - elem['plane']['start'][2])))

    material = ET.SubElement(visual, "material")
    script = ET.SubElement(material, "script")
    ET.SubElement(
        script, "uri").text = 'file://media/materials/scripts/gazebo.material'
    ET.SubElement(
        script, "name").text = 'Gazebo/Orange'

    ET.SubElement(visual, "cast_shadows").text = '1'
    ET.SubElement(visual, "transparency").text = '0'

    collision = ET.SubElement(
        link, "collision", name=name + '_collision_' + str(number))
    ET.SubElement(collision, "pose", frame="").text = str((elem['plane']['stop'][0] + elem['plane']['start'][0])/2) + ' ' + str((elem['plane']['stop'][1] + elem['plane']['start'][1])/2) + ' ' + str(
        (elem['plane']['stop'][2] + elem['plane']['start'][2])/2) + ' 0 0 ' + str(math.atan2(elem['plane']['stop'][1] - elem['plane']['start'][1], elem['plane']['stop'][0] - elem['plane']['start'][0]))
    collision_geometry = ET.SubElement(collision, "geometry")
    collision_box = ET.SubElement(collision_geometry, "box")
    ET.SubElement(collision_box, "size").text = str(math.hypot(elem['plane']['stop'][0] - elem['plane']['start'][0], elem['plane']
                                                               ['stop'][1] - elem['plane']['start'][1])) + ' 0.02 ' + str(math.fabs((elem['plane']['stop'][2] - elem['plane']['start'][2])))


def generate_world(save_path, filename, data, package_path, physics_iterations):
    sdf = ET.Element("sdf", version="1.5")

    world = ET.SubElement(sdf, "world", name="default")

    # Add sun
    ET.SubElement(ET.SubElement(world, "include"), "uri").text = "model://sun"

    # Add ground plane
    ground_include = ET.SubElement(world, "include")
    ET.SubElement(ground_include, "uri").text = "model://ground_plane"
    ET.SubElement(ground_include, "pose").text = "0 0 -0.001 0 0 0"

    # Add markers from json file
    for elem in data['markers']:
        marker_include = ET.SubElement(world, "include")
        ET.SubElement(marker_include, "uri").text = "model://" + \
            'marker_aruco-' + str(elem['id'])
        # Calculate pose
        elem['pose']['orientation'][0] = elem['pose']['orientation'][0] - 90
        elem['pose']['position'] = np.array(elem['pose']['position'])
        offset_heuristic = elem['pose']['position'][2] > 0.0
        if elem['pose'].get('offset', offset_heuristic):
            elem['pose']['position'] += 0.01005 * np.r_[np.cos(np.radians(elem['pose']['orientation'][2] + 90)),
                                                     np.sin(np.radians(elem['pose']['orientation'][2] + 90)),
                                                     0.0]


        ET.SubElement(marker_include, "pose", frame="''").text = ' '.join(str(
            e) for e in elem['pose']['position']) + ' ' + ' '.join(str(math.radians(e)) for e in elem['pose']['orientation'])

    # Add walls from json file
    index = 0
    for elem in data['walls']:
        add_wall(world, elem, index)
        if 'gate' in elem:
            continue
        index = index + 1

    # Add signs from json file
    for elem in data['roadsigns']:
        sign_include = ET.SubElement(world, "include")
        ET.SubElement(sign_include, "uri").text = "model://" + \
            'sign_' + str(elem['sign'])
        # Calculate pose
        elem['pose']['orientation'][0] = elem['pose']['orientation'][0] - 90

        ET.SubElement(sign_include, "pose", frame="''").text = ' '.join(str(
            e) for e in elem['pose']['position']) + ' ' + ' '.join(str(math.radians(e)) for e in elem['pose']['orientation'])

    # Add physics properties
    physics = ET.SubElement(
        world, "physics", name='default_physics', default='0', type='ode')
    ET.SubElement(physics, "gravity").text = "0 0 -9.8066"

    ode = ET.SubElement(physics, "ode")

    solver = ET.SubElement(ode, "solver")
    ET.SubElement(solver, "type").text = "quick"
    ET.SubElement(solver, "iters").text = "20"
    ET.SubElement(solver, "sor").text = "1.3"
    ET.SubElement(solver, "use_dynamic_moi_rescaling").text = "0"

    constraints = ET.SubElement(ode, "constraints")
    ET.SubElement(constraints, "cfm").text = "0"
    ET.SubElement(constraints, "erp").text = "0.2"
    ET.SubElement(constraints, "contact_max_correcting_vel").text = "100"
    ET.SubElement(constraints, "contact_surface_layer").text = "0.001"

    ET.SubElement(physics, "max_step_size").text = str(1.0/physics_iterations)
    ET.SubElement(physics, "real_time_factor").text = "1"
    ET.SubElement(physics, "real_time_update_rate").text = str(
        physics_iterations)
    ET.SubElement(physics, "magnetic_field").text = "6e-06 2.3e-05 -4.2e-05"

    tree = ET.ElementTree(sdf)
    indent(sdf)

    print("Saving Gazebo world to:", save_path + filename)
    tree.write(save_path + filename, encoding="utf-8", xml_declaration=True)
