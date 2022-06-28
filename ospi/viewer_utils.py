import gepetto.corbaserver
#from first_order_low_pass_filter import FirstOrderLowPassFilter
import numpy as np
import pandas as pd
import pinocchio as se3
import os
from pinocchio.utils import zero, se3ToXYZQUAT
from pinocchio.utils import zero as mat_zeros
from IPython import embed
ENABLE_VIEWER = "ON"

class Viewer(object):
    COM_SPHERE_RADIUS = 0.01;
    COM_SPHERE_COLOR = (1, 0, 0, 1);
    PLAYER_FRAME_RATE = 30;
    
    CAMERA_LOW_PASS_FILTER_CUT_FREQUENCY = 10.0;
    CAMERA_FOLLOW_ROBOT = '';   # name of the robot to follow with the camera
    CAMERA_LOOK_AT_HEIGHT = 0.5;           # height the camera should look at when following a robot
    CAMERA_REL_POS = [4, 2, 1.75];  # distance from robot to camera
    CAMERA_LOOK_AT_OFFSET = [0, 0]; # offset to robot xy position looked by camera
    
    def __init__(self, name, robot, window_name="viewer1"):
        self.name = name
        root_node_name = "world/"+robot.name
        #self.filter = FirstOrderLowPassFilter(0.002, self.CAMERA_LOW_PASS_FILTER_CUT_FREQUENCY, mat_zeros(2))
        print ("Adding robot: "+robot.name)
        # Launch the gepetto corba server for communicating with the viewer
        #self.initDisplay("world/pinocchio")
        self.initDisplay(root_node_name)
        #nodeName = "world/"+self.robot.name
        # Load and display the model
        self.loadDisplayModel(root_node_name, window_name, robot)
        #self.viewer.gui.setLightingMode('world/floor', 'OFF')
        self.robots = {robot.name: robot}


    def initDisplay(self, viewerRootNodeName):#="world/pinocchio"):
        try:
            self.viewer=gepetto.corbaserver.Client()
            print ("Connected to corba server")
            self.viewerRootNodeName = viewerRootNodeName
        except:
            if 'viewer' in self.__dict__:
                del self.viewer
            print ("!! Error while starting the viewer client. ")
            print ("Check whether Gepetto-viewer-corba is properly started")

    # Create the scene displaying the robot meshes in gepetto-viewer      
    def loadDisplayModel(self, nodeName, windowName, robot):
        # Open a window for displaying your model.     
        try:
            # If the window already exists, do not do anything
            self.windowID = self.viewer.gui.getWindowID(windowName)
            print ("Warning: window '"+windowName+"' already created.")
            print ("The previously created objects will not be destroyed and do not have to be created again.")
        except:
            # Otherwise, create the empty window.
            print ("Creating window: "+windowName)
            self.windowID = self.viewer.gui.createWindow(windowName)

        # Start a new "scene" in this window, named "world/robot.name/", with just a floor.
        if nodeName not in self.viewer.gui.getSceneList():
            print ("creating scene: "+nodeName)
            self.viewer.gui.createSceneWithFloor(nodeName)
        else:
            print (nodeName+" already in scene list")

        self.viewer.gui.addSceneToWindow(nodeName, self.windowID)

        self.viewer.gui.createGroup(nodeName)        
        # iterate over visuals and create the meshes in the viewer 
        for i in range (1,len(robot.visuals)):
            try:
                visual_name= os.path.split(robot.visuals[i][2])[1]
                self.viewer.gui.addMesh(nodeName+'/'+
                                        robot.visuals[i][1]+'_'+
                                        visual_name, 
                                        robot.visuals[i][2])
            except:
                visual_name= os.path.split(robot.visuals[i][2])[1]
                print (nodeName+"/"+robot.visuals[i][1]+'_'+visual_name+" already created")
                #print "Node "+visual_name+" already created"
        # iterate for creating nodes for all joint 
        #for i in range(1,robot.model.nbodies):
        #    self.viewer.gui.addXYZaxis(nodeName+'/'+
        #                               robot.model.names[i], [1., 0., 0., .5], 0.02, 1)
        # create a node for the center of mass
        self.viewer.gui.addXYZaxis(nodeName+
                                   '/globalCoM', [0., 1., 0., .5], .015, 0.05)#0.3

        #Add group for markers
        self.viewer.gui.createScene('markers')
        self.viewer.gui.addSceneToWindow('markers', self.windowID)
        self.viewer.gui.createGroup('markers')  

        #Add group for force arrows
        self.viewer.gui.createScene('forces')
        self.viewer.gui.addSceneToWindow('forces', self.windowID)
        self.viewer.gui.createGroup('forces')  

        # Finally, refresh the layout to obtain your first rendering
        self.viewer.gui.refresh()

    def placeObject(self, objName, M, refresh=True):
        '''     
        This function places (ie changes both translation and rotation) of the object names 
        "objName" in place given by the SE3 object "M". By default, immediately refresh the
        layout. If multiple objects have to be placed at the same time, do the refresh only
        at the end of the list
        '''
        pinocchioConf = se3.SE3ToXYZQUATtuple(M) 
        self.viewer.gui.applyConfiguration(objName,pinocchioConf)
        if refresh: self.viewer.gui.refresh()

    def place_marker(self, marker_name, position, color_rgba = [0,0,0,1]):
        if not(self.viewer.gui.nodeExists(f"markers/{marker_name}")):
            self.viewer.gui.addSphere(f"markers/{marker_name}", 0.01, color_rgba)
        self.viewer.gui.applyConfiguration(f"markers/{marker_name}", position + [0,0,0,1]) # position + quaternion
        self.viewer.gui.refresh()
        return
    
    def place_arrow(self, arrow_name, position, direction, length, color_rgba = [0,1,0,1]):
        """ 
        positon x,y,z of the point of application, direction (quat) of the arrow
        """
        if not(self.viewer.gui.nodeExists(f"forces/{arrow_name}")):
            self.viewer.gui.addArrow(f"forces/{arrow_name}", 0.01,length , color_rgba)
        else:
            self.viewer.gui.resizeArrow(f"forces/{arrow_name}", 0.01,length)    
        self.viewer.gui.applyConfiguration(f"forces/{arrow_name}", position + direction) # position + quaternion
        self.viewer.gui.refresh()
        return

    def display(self, q, robotName, com=False, joint_frames=False, osimref=True,show_markers=True):
        robot = self.robots[robotName]
        #Update kinematics
        se3.forwardKinematics(robot.model,robot.data,q)
        se3.updateFramePlacements(robot.model,robot.data)

        oMp = se3.SE3.Identity().rotation
        if osimref is True:
            oMp = se3.utils.rotate('z',np.pi/2) @ se3.utils.rotate('x',np.pi/2)
        # show CoM
        if com is True:
            CoM = se3.SE3.Identity()
            CoM.translation =robot.com(q)
            self.placeObject('world/'+robot.name+"/globalCoM", CoM, True)

        #display skelette visuals
        for i in range (1,len(robot.visuals)):
            # get the parent joint
            idx = robot.model.getJointId(robot.visuals[i][1])
            pose = se3.SE3.Identity()
            if osimref is True:
                # convert bones pose
                pose.translation =  robot.data.oMi[idx].translation
                pose.rotation= robot.data.oMi[idx].rotation @ oMp

            else:
                pose.translation= robot.data.oMi[idx].translation+robot.visuals[i][4][3:6].A1
                pose.rotation= robot.data.oMi[idx].rotation

            # place objects
            if i != len(robot.visuals)-1:
                self.viewer.gui.setScale('world/'+robot.name+'/'+robot.visuals[i][1]+'_'+os.path.split(robot.visuals[i][2])[1], robot.visuals[i][3])
                self.placeObject('world/'+robot.name+'/'+robot.visuals[i][1]+'_'+os.path.split(robot.visuals[i][2])[1], pose, False)
            else:
                self.viewer.gui.setScale('world/'+robot.name+'/'+ robot.visuals[i][1]+'_'+ os.path.split(robot.visuals[i][2])[1],  robot.visuals[i][3])
                self.placeObject('world/'+robot.name+'/'+robot.visuals[i][1]+'_'+os.path.split(robot.visuals[i][2])[1], pose, True)
            if joint_frames is True:
                self.JointFrames(robot.name)
        
        if show_markers:
            for marker in robot.markers:
                marker_name, parent_body, location = marker # marker_name, parent_body, location [X,Y,Z]
                position = robot.data.oMf[robot.model.getFrameId("marker_"+marker_name)].translation
                self.place_marker('marker_'+marker_name, list(position),color_rgba=[1,1,1,1])
        
    def updateRobotConfig(self, q, robotName, osimref=True):
        #self.robots[robotName].display(q, robotName)
        
        se3.computeAllTerms(self.robots[robotName].model, 
                            self.robots[robotName].data, 
                            q, 
                            self.robots[robotName].v)
        
        #se3.forwardKinematics(self.robots[robotName].model, 
        #                      self.robots[robotName].data, 
        #                      q)
        #se3.framesKinematics(self.robots[robotName].model, 
        #                     self.robots[robotName].data, 
        #                     q)
        self.display(q,robotName)

    def addRobot(self, robot):
        if(ENABLE_VIEWER):
            self.robots[robot.name] = robot
            nodeName = "world/"+robot.name
            print ("adding robot: "+robot.name)
            self.loadDisplayModel(nodeName, "pinocchio", self.robots[robot.name]);
            se3.framesKinematics(self.robots[robot.name].model, 
                                 self.robots[robot.name].data, 
                                 self.robots[robot.name].q0)
            self.updateRobotConfig(self.robots[robot.name].q0,robot.name)
            
            

    def addSphere(self,name, radius, xyz, rpy=mat_zeros(3), color=(0,0,0,1.0), lightingMode='ON'):
        #if(ENABLE_VIEWER):
        position = xyz + [0,0,0,1] # MODIF xyzRpyToViewerConfig(xyz, rpy);
        self.viewer.gui.addSphere('world/'+name, radius, color)
        self.viewer.gui.applyConfiguration('world/'+name, position)
        self.viewer.gui.setLightingMode('world/'+name, lightingMode)
        self.viewer.gui.refresh()

    def addLine(self,name, pos1, pos2, color=(0,0,0,1.0), lightingMode='ON'):
        if(ENABLE_VIEWER):
            if(len(pos1.shape)==1):
                self.viewer.gui.addLine(name, 
                                         (pos1[0], pos1[1], pos1[2]), 
                                         (pos2[0], pos2[1], pos2[2]), 
                                         color)
            else:
                self.viewer.gui.addLine(name, 
                                        (pos1[0,0], pos1[1,0], pos1[2,0]), 
                                        (pos2[0,0], pos2[1,0], pos2[2,0]), 
                                        color);
            self.viewer.gui.setLightingMode(name, lightingMode);

    def setVisibility(self, name, mode='OFF'):
        if(ENABLE_VIEWER):
            self.viewer.gui.setVisibility('world/'+name, mode);
    
    def addLandmark(self, nodeName, size):
        self.viewer.gui.addLandmark(nodeName, size)
        
    def JointFrames(self, robotName, ON=True):
        robot = self.robots[robotName]
        if ON is True:
            #create nodes
            for i in range(1, robot.model.nbodies):
                pose =  robot.data.oMi[i] 
                root_node_name = 'world/'+robot.name+'/'+robot.model.names[i]
                if self.viewer.gui.nodeExists(root_node_name+'_frame') is False:
                    self.viewer.gui.addXYZaxis(root_node_name+'_frame', [.7, .7, .7, .5], 0.015, 0.02)
                    #self.viewer.gui.addArrow(root_node_name+'_xaxis', .005, .03, [1., 0. ,0. ,.5])
                self.placeObject(root_node_name+'_frame', pose, True)
                #self.addLine('line1', pos1, pos2)
        else:
            for i in range(1, robot.model.nbodies):
                node_name = 'world/'+robot.name+'/'+robot.model.names[i]+'_frame'
                self.viewer.gui.deleteNode(node_name, False)
            
        #embed()
    def displayLowerJointLimits(self, robotName, ff=False):
        pass
        #viewer.display(self.robot.model.lowerPositionLimit, robotName)
        #for i in xrange(0, wb_model.model.nbodies):
        #    q = self.robot.model.lowerPositionLimit
        #    q[:7] = np.zeros((7,1))
        #    viewer.display(q, robotName)

    def getNodeList(self):
        return self.viewer.gui.getNodeList()

