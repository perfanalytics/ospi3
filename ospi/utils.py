"""
.. module:: opensim parser with pinocchio
       :platform: ubuntu
       :parse OpenSim into pinocchio models
.. moduleauthor:: Galo Maldonado <galo_xav@hotmail.com>
"""

import numpy as np
import pinocchio as se3
import ospi.algebra as alg 


def _parse2PinocchioJoints(pymodel):
    jts = 0 
    # save pinocchio like joint models
    joint_models = []
    # axis transformations (e.g. for inversing sign with left and right)
    joint_transformations = []
    for joints in pymodel['Joints']:
        dof_in_joint = 6 - (joints[2]['coordinates']).count(None)
        if dof_in_joint == 6:
            joint_transformations.append(np.matrix(np.float64(joints[2]['axis'])))
            joint_models.append([jts, pymodel['Joints'][jts][0]['name'][0], se3.JointModelFreeFlyer()])
        elif dof_in_joint == 3:
            joint_transformations.append(np.matrix(np.float64(joints[2]['axis']))[0:3,:])
            joint_models.append([jts, pymodel['Joints'][jts][0]['name'][0], se3.JointModelSpherical()])
        elif dof_in_joint == 2:
            print ('2 dof not supported')
        elif dof_in_joint == 1:
            for dof in range(0, len(joints[2]['coordinates'])):
                if joints[2]['coordinates'][dof] != None:
                    if joints[2]['name'][dof][0:8] == 'rotation':
                        if joints[2]['axis'][dof] == ['1', '0', '0'] or joints[2]['axis'][dof] == ['-1', '0', '0']:
                            #Y
                            joint_models.append([jts,pymodel['Joints'][jts][0]['name'][0],se3.JointModelRY()])
                            joint_transformations.append(np.matrix(np.float64(joints[2]['axis']))[dof])
                            
                        elif joints[2]['axis'][dof] == ['0', '1', '0']:
                            #Z
                            joint_models.append([jts,pymodel['Joints'][jts][0]['name'][0],se3.JointModelRZ()])
                            joint_transformations.append(np.matrix(np.float64(joints[2]['axis']))[dof])

                        elif joints[2]['axis'][dof] == ['0', '0', '1']:
                            #X
                            joint_models.append([jts,pymodel['Joints'][jts][0]['name'][0],se3.JointModelRX()])
                            joint_transformations.append(np.matrix(np.float64(joints[2]['axis']))[dof])

                        else:
                            joint_transformations.append(np.matrix(np.float64(joints[2]['axis']))[dof])
                            v=np.matrix( [np.float64(joints[2]['axis'][dof][0]),
                                          np.float64(joints[2]['axis'][dof][1]), 
                                          np.float64(joints[2]['axis'][dof][2])] )
                            #2,0,1
                            joint_models.append([jts,
                                                 pymodel['Joints'][jts][0]['name'][0],
                                                 se3.JointModelRevoluteUnaligned(v[0,2], v[0,0], v[0,1])])
        jts += 1                    
    return joint_models, joint_transformations

def pinocchioCoordinates(model, joint_transformations, dof, representation="quat"):
    jt = joint_transformations
    oMp = se3.utils.rotate('z', np.pi/2) @ se3.utils.rotate('x', np.pi/2)
    q = np.matrix(np.zeros((model.nq, 1)))
    qo_idx = 0 #osim index
    qp_idx = 0 #pinocchio index

    
    # for quaternions
    def orderQuat(quat):
    	return [quat[1], quat[2], quat[3], quat[0]]
    
    for i in range(1,len(model.joints)):
        if (model.joints[i].shortname() == "JointModelFreeFlyer"):
            q[qp_idx+0:qp_idx+3,0] = ( oMp @ dof[qo_idx+3:qo_idx+6,0] ).A  #tx,ty,tz
            q[qp_idx+3:qp_idx+7,0] = np.matrix(orderQuat(alg.quaternion_from_matrix(alg.euler_matrix((dof[qo_idx,0]), dof[qo_idx+1,0], dof[qo_idx+2,0], 'rxyz')))).T
            qo_idx += 6
            qp_idx += 7
        elif (model.joints[i].shortname() == "JointModelSpherical"):
            jt_sph = jt[i-1]
            tup = np.nonzero(jt_sph)
            row=tup[0]; col=tup[1];
            q[qp_idx:qp_idx+4,0] = np.matrix(orderQuat(alg.quaternion_from_matrix(alg.euler_matrix((dof[qo_idx,0])*jt_sph[row[0],col[0]], dof[qo_idx+1,0]*jt_sph[row[1],col[1]], dof[qo_idx+2,0]*jt_sph[row[2],col[2]], 'rxyz')))).T
            qo_idx += 3
            qp_idx += 4
        elif (model.joints[i].shortname() == "JointModelSphericalZYX"):
            # TODO joint transformation
            q[qp_idx:qp_idx+4,0] = np.matrix(orderQuat(alg.quaternion_from_matrix(alg.euler_matrix((dof[qo_idx,0]), dof[qo_idx+1,0], dof[qo_idx+2,0], 'rzyx')))).T
            qo_idx += 3
            qp_idx += 4
        elif (model.joints[i].shortname() == "JointModelRevoluteUnaligned"):
            #joint transformations are handled when creating joint model?
            q[qp_idx,0] = dof[qo_idx,0]
            qo_idx += 1
            qp_idx += 1
        elif (model.joints[i].shortname() == "JointModelRX" or 
              model.joints[i].shortname() == "JointModelRY" or 
              model.joints[i].shortname() == "JointModelRZ"):
            jt_rev = jt[i-1]
            jt_idx = np.nonzero(jt_rev.A1)[0][0]
            q[qp_idx,0] = dof[qo_idx,0]*jt_rev[0,jt_idx]
            qo_idx += 1
            qp_idx += 1
            
    return q



