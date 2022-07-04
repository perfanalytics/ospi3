import xml.etree.ElementTree as xml
import pandas as pd

class Force():
    def __init__(self, forceName, applicationBody, force_identifier, torque_identifier, point_identifier):
        self.name = forceName
        self.applicationBody = applicationBody
        self.force_identifier = force_identifier #name in .mot file
        self.torque_identifier = torque_identifier #name in .mot file
        self.point_identifier = point_identifier #name in .mot file
        self.force_data = [] # [ [t, fx,fy,fz, tz,ty,tz, px,py,pz] for t in t_tab] 

    def update_force_data(self,force_data):
        """
        force_data = [ [t, fx,fy,fz, tz,ty,tz, px,py,pz] for t in t_tab] 
        with f forces, t torques et p point of application
        expressed in world frame
        """
        self.force_data = force_data

def load_force_data(force_data_path, L_forces, rotate_axis = False):
    """ 
    Read ground force file .mot and append to each forces of L_forces associated date
    """

    force_df = pd.read_csv(force_data_path, sep="\t", skiprows=6)
    #FILTER DATA

    if rotate_axis:
        column_names = force_df.columns
        for force in (L_forces):
            for id in [force.force_identifier,force.torque_identifier,force.point_identifier]:
                force_df.rename(columns={f"{id}y":f"{id}z",f"{id}z":f"{id}y"},inplace=True)
                force_df.rename(columns={f"{id}x":f"{id}y",f"{id}y":f"{id}x"},inplace=True)
        force_df = force_df[column_names] #reordering columns

    for force in L_forces:
        force_data = force_df[['time',
                force.force_identifier+'x',force.force_identifier+'y',force.force_identifier+'z',
                force.torque_identifier+'x',force.torque_identifier+'y',force.torque_identifier+'z',
                 force.point_identifier+'x',force.point_identifier+'y',force.point_identifier+'z'     ]]
        force.update_force_data(force_data.to_numpy())

    return force_df

def force_parser(path, filename, rotate_axis = False):

    tree = xml.parse(path+"/"+filename)

    external_forces = []

    for force in tree.getroot().iter('ExternalForce'):
        name = force.get('name')
        applicationBody = force.find('applied_to_body').text.strip()
        force_identifier = force.find('force_identifier').text.strip()
        torque_identifier = force.find('torque_identifier').text.strip()
        point_identifier = force.find('point_identifier').text.strip()
        external_forces.append(Force(name, applicationBody, force_identifier, torque_identifier, point_identifier))

    data_file = tree.getroot().find('ExternalLoads').find('datafile').text.strip()

    force_df = load_force_data(path+'/'+data_file,external_forces, rotate_axis)
    return external_forces

