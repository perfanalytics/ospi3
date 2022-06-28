"""
.. module:: opensim parser with pinocchio
       :platform: ubuntu
       :parse OpenSim into pinocchio models
.. moduleauthor:: Galo Maldonado <galo_xav@hotmail.com>
"""
import xml.etree.ElementTree as xml
import numpy as np
import pinocchio as se3
import os 
import ospi.model_builder as builder
import ospi.utils as utils

def readOsim(filename):
    pymodel = {
        'Bodies':[], 
        'Joints':[], 
        'Visuals':[],
        'Forces':[],
        'Point':[],
        'Markers':[]
    }
    tree = xml.parse(filename)
    root = tree.getroot()

    version = int(root.get('Version'))

    if version >= 40000: #name difference between opensim 3 and 4 for parent body
        print('Compatible only with version file = 30000')
    
    # *********************************************************
    #                   Get body set
    # *********************************************************
    for bodies in root.findall('./Model/BodySet/objects/Body'):
        body_data = {'name':[], 
                     'mass':[], 
                     'mass_center':[], 
                     'inertia':[]}
        body_data['name'].append(bodies.get('name'))
        body_data['mass'].append(bodies.find('mass').text)
        body_data['mass_center'].append((bodies.find('mass_center').text).split())

        if version >= 40000:
            Ixx, Iyy, Izz, Ixy, Ixz, Iyz = [float(coord) for coord in bodies.find('inertia').text.split()]
            Y = [[ Ixx, Ixy, Ixz,
                Ixy, Iyy, Iyz,
                Ixz, Iyz, Izz]]
        else:
            Y = [[bodies.find('inertia_xx').text,
                bodies.find('inertia_xy').text,
                bodies.find('inertia_xz').text],
                [bodies.find('inertia_xy').text,
                bodies.find('inertia_yy').text,
                bodies.find('inertia_yz').text],
                [bodies.find('inertia_xz').text,
                bodies.find('inertia_yz').text,
                bodies.find('inertia_zz').text]]
        body_data['inertia'].append(Y)
        pymodel['Bodies'].append(body_data)

       

        
        joints_list =  bodies.iter('CustomJoint')
        for joint in joints_list:
            joint_data = {
                'name':[], 
                'parent_body':[], 
                'location_in_parent':[], 
                'orientation_in_parent':[], 
                'location':[],
                'orientation':[]
            }
            joint_data['name'].append(joint.get('name'))
            joint_data['parent_body'].append(joint.find('parent_body').text)
            joint_data['location_in_parent'].append((joint.find('location_in_parent').text).split())
            joint_data['orientation_in_parent'].append((joint.find('orientation_in_parent').text).split())
            joint_data['location'].append((joint.find('location').text).split())
            joint_data['orientation'].append((joint.find('orientation').text).split())

            # Coordinate Set
            coordinates_list = joint.iter('Coordinate')
            coordinate_data = {'name':[],
                               'motion_type':[],
                               'default_value':[],
                               'default_speed_value':[],
                               'range':[],
                               'clamped':[],
                               'locked':[],
                               'prescribed_function':[]
            }
            for coordinates in coordinates_list:
                coordinate_data['name'].append(coordinates.get('name'))
                coordinate_data['motion_type'].append(coordinates.find('motion_type').text)
                coordinate_data['default_value'].append(coordinates.find('default_value').text)
                coordinate_data['default_speed_value'].append(coordinates.find('default_speed_value').text)
                coordinate_data['range'].append((coordinates.find('range').text).split())
                coordinate_data['clamped'].append(coordinates.find('clamped').text)
                coordinate_data['locked'].append(coordinates.find('locked').text)
                coordinate_data['prescribed_function'].append(coordinates.find('prescribed_function').text)
                               
            #get spatial transform
            spatial_list = joint.iter('TransformAxis')
            spatial_data = {
                'name': [],
                'coordinates': [],
                'axis': []
            }
            for spatial_transform in spatial_list:
                spatial_data['name'].append(spatial_transform.get('name'))
                spatial_data['axis'].append((spatial_transform.find('axis').text).split())

                #checking if there is a spline (knee for e.g.) and converting it in constant translation
                spline = spatial_transform.find('.//SimmSpline') #meaning of ".//" = check recursively 
                if spline !=None: 
                    y_value = spline.find('y').text.split()
                    translation = np.array([ float(y) for y in y_value]).mean()
                    scale = spatial_transform.find("function").find(".//scale")
                    if  scale != None:
                        scale_factor = float(spatial_transform.find("function").find(".//scale").text)
                    else:
                        scale_factor = 1
                    axis = np.array([ float(x) for x in (spatial_transform.find('axis').text).split()])
                    loc = joint_data['location_in_parent'][0]
                    loc = np.array([float(coord) for coord in loc])
                    new_loc = loc + axis * translation * scale_factor
                    new_loc = [ str(coord) for coord in new_loc]
                    joint_data['location_in_parent'][0] = new_loc
                    spatial_data['coordinates'].append(None) #pas de degré de liberté
                else:
                    spatial_data['coordinates'].append(spatial_transform.find('coordinates').text)


            pymodel['Joints'].append([joint_data, coordinate_data, spatial_data])

            

        # Visible Objects
        visible_list =  bodies.iter('VisibleObject')
        for visuals in visible_list:
            visuals_data = {
                'scale_factors':[],
                'show_axes':[],
                'display_preference':[]
            }
            visuals_data['scale_factors'].append((visuals.find('scale_factors').text).split())
            visuals_data['show_axes'].append(visuals.find('show_axes').text)
            visuals_data['display_preference'].append(visuals.find('display_preference').text)
                
            bones_list =  visuals.iter('DisplayGeometry')
            bones_data = {
                'geometry_file': [],
                'color': [],
                'transform': [],
                'scale_factors': [],
                'display_preference': [],
                'opacity': []
            }
            for bones in bones_list:
                bones_data['geometry_file'].append(bones.find('geometry_file').text) 
                bones_data['color'].append((bones.find('color').text).split())
                bones_data['transform'].append((bones.find('transform').text).split())
                bones_data['scale_factors'].append((bones.find('scale_factors').text).split())
                bones_data['display_preference'].append(bones.find('display_preference').text)
                bones_data['opacity'].append(bones.find('opacity').text)
            pymodel['Visuals'].append([visuals_data, bones_data])




    # *********************************************************
    #                 Get force set
    # *********************************************************
    
    # Schutte1993Muscle_Deprecated
    for forces in root.findall('./Model/ForceSet/objects/Schutte1993Muscle_Deprecated'):
        force_data = {
            'force_name':[],
            'type':[]
        }
        force_data['force_name'].append(forces.get('name'))
        force_data['type'].append('Schutte1993Muscle_Deprecated')
        # point set
        points = []
        path_point_list =  forces.iter('PathPoint')
        for point in path_point_list:
            point_data = {'point_name':[], 'location':[], 'body':[]}
            point_data['point_name'].append(point.get('name'))
            point_data['location'].append((point.find('location').text).split())
            if version >= 40000:
                point_data['body'].append(point.find('socket_parent_frame').text)
            else:
                point_data['body'].append(point.find('body').text)    
            points.append(point_data)
        pymodel['Forces'].append([force_data,points]) 
        
    # Thelen2003Muscle
    for forces in root.findall('./Model/ForceSet/objects/Thelen2003Muscle'):
        force_data = {
            'force_name':[],
            'type':[]
        }
        force_data['force_name'].append(forces.get('name'))
        force_data['type'].append('Thelen2003Muscle')
        # point set
        points = []
        path_point_list =  forces.iter('PathPoint')
        for point in path_point_list:
            point_data = {'point_name':[], 'location':[], 'body':[]}
            point_data['point_name'].append(point.get('name'))
            point_data['location'].append((point.find('location').text).split())
            if version >= 40000:
                point_data['body'].append(point.find('socket_parent_frame').text)
            else:
                point_data['body'].append(point.find('body').text)    
            points.append(point_data)
            points.append(point_data)
        pymodel['Forces'].append([force_data,points])

    #add Markers
    for marker in root.iter('Marker'):
        x,y,z = [float(coord) for coord in marker.find('location').text.split()]
        location = [z,x,y] #osim axes rotation
        if (version >= 40000):
            body = marker.find('socket_parent_frame').text.strip()
        else:
            body = marker.find('body').text.strip()
        marker_data = [marker.get('name'), body, location] # [marker_name, parent_body, location [x,y,z]]
        pymodel['Markers'].append(marker_data)
    
    return pymodel        


# *********************************************************************************************

def addExternalMarkerSet(pymodel,markerSet_path):
    if pymodel['Markers'] != []:
        print("Scaled MarketSet already included in .osim file. External MarkerSet file is not used.")
        return

    tree = xml.parse(markerSet_path)
    root = tree.getroot()
    
    for marker in root.iter('Marker'):
        x,y,z = [float(coord) for coord in marker.find('location').text.split()]
        location = [z,x,y] #osim axes rotation
        body = marker.find('body').text.strip()
        marker_data = [marker.get('name'), body, location] # [marker_name, parent_body, location [x,y,z]]
        pymodel['Markers'].append(marker_data)

# *********************************************************************************************
def parseModel(filename, mesh_path, markerSet_path, verbose=False):
    pymodel = readOsim(filename)
    if markerSet_path != None:
        addExternalMarkerSet(pymodel,markerSet_path)
    ms_system = builder.MS("MS system") # TODO get model name
    osMpi = se3.utils.rotate('z', np.pi/2) @ se3.utils.rotate('x', np.pi/2)

    joint_models, joint_transformations = utils._parse2PinocchioJoints(pymodel)    
    ms_system.createJointTransformations(joint_transformations)

    idx = []
    for joint in range(0,len(pymodel['Joints'])):
        parent_name = pymodel['Joints'][joint][0]['parent_body'][0]
        idx.append(parent_name)
        
    jointLimits = []
    for joint in range(0,len(pymodel['Joints'])):
        # don't take into account ground body and visual
        body = joint  + 1
        body_name = pymodel['Bodies'][body]['name'][0]
        joint_name = pymodel['Joints'][joint][0]['name'][0] 
        joint_id = body
        parent = idx.index(pymodel['Joints'][joint][0]['parent_body'][0])
        joint_model = joint_models[joint][2]
        jointLimits += pymodel['Joints'][joint][1]['range']
        
        if (verbose):
            print ('ID: ',joint_id)
            print ('Joint Name: '+joint_name)
            print ('Parent Name: '+pymodel['Joints'][joint][0]['parent_body'][0], parent)
            print ('Joint Model: ',joint_model)
            print ('Joint Limits: ',pymodel['Joints'][joint][1]['range'])
            
        # From OpenSim to Pinocchio
        joint_placement = se3.SE3.Identity()
        r = np.matrix(pymodel['Joints'][joint][0]['orientation_in_parent'][0],dtype = np.float64).T
        # TODO change orientation of joint ***
        joint_placement.rotation = se3.utils.rpyToMatrix(osMpi @ r)
            
        t = pymodel['Joints'][joint][0]['location_in_parent'][0]            
        joint_placement.translation = osMpi @  np.matrix(t,dtype=np.float64).T
            
        mass = np.float64(pymodel['Bodies'][body]['mass'][0])
        mass_center = osMpi @ np.matrix(pymodel['Bodies'][body]['mass_center'][0], dtype = np.float64).T
        inertia_matrix = np.matrix(pymodel['Bodies'][body]['inertia'][0], dtype = np.float64)
        body_inertia = (se3.Inertia( mass, mass_center, inertia_matrix))
        body_placement = se3.SE3.Identity()
            
        # Add to pinocchio model
        ms_system.buildModel(
            parent, 
            joint_model,
            joint_placement,
            joint_name,joint_id, 
            body_inertia,
            body_placement, 
            body_name
        )
    
        scale_factors = osMpi @ (np.matrix(pymodel['Visuals'][body][0]['scale_factors'][0], np.float64)).T
        scale_factors = np.asarray(scale_factors.T)[0]
        scale_factors = [scale_factors[0], scale_factors[1], scale_factors[2]]

        # add to visuals list
        for mesh in range(0, len(pymodel['Visuals'][body][1]['geometry_file']) ):
            visual_name = os.path.splitext(pymodel['Visuals'][body][1]['geometry_file'][mesh])[0]
            filename = mesh_path+'/'+visual_name+'.obj'
            if (verbose): print ('Filename: '+filename)
            transform = np.matrix(pymodel['Visuals'][body][1]['transform'][mesh],dtype=np.float64).T
            transform[3:6] =  osMpi @ transform[3:6]
            transform[0:3] =  osMpi @ transform[0:3]
            visuals = ms_system.createVisuals(parent, joint_name, filename, scale_factors, transform)
        if (verbose): print ('****')
    
    # add markerSet
    ms_system.createMarkerSet(pymodel['Markers'])

    # create data
    ms_system.createData()

    # add constraints
    #ms_system.createConstraints(np.matrix(jointLimits,dtype=np.float64))

    # add forces
    for force in range(0,len(pymodel['Forces'])):
        force_name = pymodel['Forces'][force][0]['force_name'][0]
        force_type = pymodel['Forces'][force][0]['type'][0]
        points = []
        for point in range(0,len(pymodel['Forces'][force][1])):
            parent = pymodel['Forces'][force][1][point]['body'][0]
            point_name = pymodel['Forces'][force][1][point]['point_name'][0]
            location = osMpi @ np.matrix([pymodel['Forces'][force][1][point]['location'][0]],dtype=np.float64).T
            points.append([point_name,parent,location])
        ms_system.createForces(force_name,force_type,parent,points)

    

    return ms_system
