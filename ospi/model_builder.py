import numpy as np
from IPython import embed

import pinocchio as se3
import ospi.utils as utils


class MS:
    def __init__(self, name="whole-body MS model"):
        # Pinocchio Model
        self.model = se3.Model()
        # Skelette Meshes
        self.visuals = []
        self.visuals.append([0, 'ground', 'none'])
        self.forces = []
        self.markers = [] # [ [marker_name, parent_body, location [X,Y,Z]] for marker in marker_set ] 
        self.joint_transformations = []
        self.name = name
        #self.FrameType = se3.FrameType.OP_FRAME

    def buildModel(self, parent, joint_model, joint_placement, joint_name, joint_id, body_inertia, body_placement,
                   body_name):
        ''' Add a model to the kinematic three
      TODO add with bounds, check model.hpp
      '''
        frame_parent = self.model.getFrameId(joint_name)
        #print 'frame_parent: ', frame_parent
        #self.model.addFrame(joint_name, joint_id, frame_parent, joint_placement, se3.FrameType.JOINT)
        self.model.addJoint(parent, joint_model, joint_placement, joint_name)
        self.model.addJointFrame(joint_id, frame_parent)
        ''' Append a body to the given joint in the kinematic tree
      '''
        self.model.appendBodyToJoint(joint_id, body_inertia, body_placement)
        self.model.addBodyFrame(body_name, joint_id, body_placement, parent)
        ''' Add a frame to the frame three i.e. operational points
      '''
        #self.model.addFrame(joint_name, parent, idx_f, body_placement, self.FrameType)
        #return self.model

    def createVisuals(self, parent, joint_name, filename, scale_factors=None, transform=None):
        self.visuals.append([parent, joint_name, filename, scale_factors, transform])

    def createForces(self, force_name, force_type, parent, points):
        #TODO
        self.forces.append([force_name, force_type, parent, points])

    def createConstraints(self, qRoM):
        self.model.lowerPositionLimit = utils.pinocchioCoordinates(self.model, self.joint_transformations, qRoM[:, 0])
        self.model.upperPositionLimit = utils.pinocchioCoordinates(self.model, self.joint_transformations, qRoM[:, 1])
        #TODO
        #self.Model.effortLimit
        #self.Model.velocityLimit

    def createData(self):
        self.data = self.model.createData()

    def createJointTransformations(self, joint_transformations):
        self.joint_transformations = joint_transformations

    def createMarkerSet(self, marker_set):
        self.markers = marker_set # [marker_name, parent_body, location [X,Y,Z] for marker in marker_set]
        #creating frames
        for marker in marker_set:
            marker_name, parent_body, location = marker # marker_name, parent_body, location [X,Y,Z]
            parent_frame_id = self.model.getFrameId(parent_body)
            parent_joint_id = self.model.frames[parent_frame_id].parent
            M = se3.SE3.Identity()
            M.translation = np.array(location)
            self.model.addFrame(se3.Frame("marker_"+marker_name,parent_joint_id, parent_frame_id, M,se3.OP_FRAME))
