#!/usr/bin/env python

import subprocess
import time

import ospi.motion_parser as mtp
import ospi.viewer_utils as vw
import ospi.wrapper as wr
from ospi.force_data_parser import force_parser
from ospi.trc_parser import read_trc, adapte_to_marker_set
import pandas as pd
import numpy as np

import pinocchio as pin

#Data
path = "./models/Gait2354_Simbody/"
filename_model = 'subject01.osim'
mesh_path = './models/Geometry'
filename_externalMarkerSet = 'gait2354_Scale_MarkerSet.xml' # useless if the model is scaled with openSim since the markerSet is included in .osim. To add external marker set, precise 'markerSet_path = path + filename_externalMarkerSet' when calling Wrapper

filename_trc = "subject01_walk1.trc"
filename_grf = 'subject01_walk1_grf.xml'
filename_mot = "subject01_walk1_ik.mot"

# Create a wrapper specific to the whole-body model
# The wrapper parse the OpenSim model and builds pinocchio model and data
wb = wr.Wrapper(path+filename_model, mesh_path, name='whole-body_model') 

# call the gepetto viewer server
gvs = subprocess.Popen('/home/florian/miniconda3/envs/stage_v1/bin/gepetto-gui', shell=True )
#gvs = subprocess.Popen('gepetto-gui', shell=True ) # if gepetto-gui is installed in a conda env replace by '/home/$USER_NAME$/miniconda3/envs/$ENV_NAME$/bin/gepetto-gui'
print('Loading the viewer ...')
time.sleep(4)

# Init the viewer and add the model to it

q=wb.q0
viewer = vw.Viewer('viewer', wb)
viewer.setVisibility(wb.name + "/floor", "OFF")
viewer.display(q, wb.name)

# See axis
# viewer.JointFrames(wb.name)

# Parsing motion data
time_tab, q_mot, colheaders, qOsim = mtp.parseMotion(wb.model, wb.joint_transformations, path+filename_mot, 'quat')

# Parsing ground force data
external_forces = force_parser(path, filename_grf, rotate_axis = True)
#t_tab = external_forces[0].force_data[:,0]

#Parsing trc data 
trc_df = read_trc(path + filename_trc, filter = True, rotate_axis=True)
trc_df = adapte_to_marker_set(trc_df, wb.markers)
L_marker_name = list(trc_df.columns)
L_frame_id = [wb.model.getFrameId('marker_'+name) for name in L_marker_name]


def playMotions(first=0, last=1, step=3, t=0):
    rep = 0
    while rep < 3 :
        for i in range(first, last, step):
            viewer.display(q_mot[i].T, wb.name,osimref=True)

            frame = i + 1
            current_frame_df = trc_df.iloc[frame-1]

            for marker in current_frame_df.index:
                viewer.place_marker(marker+"_goal",list(current_frame_df.loc[marker]),color_rgba=[0,0,0,1])
            
            for force in external_forces:
                vector = force.force_data[10*i][1:4]
                length = np.linalg.norm(vector)
                position = list(force.force_data[10*i][7:10])
                orientation = list(pin.Quaternion.FromTwoVectors(np.array([1,0,0]),np.array(vector)).coeffs())
                viewer.place_arrow(force.name,position,orientation,0.001*length)

            time.sleep(t)
        rep+=1


playMotions(0, len(time_tab), 1, 0.0025)

time.sleep(2)
gvs.terminate()
