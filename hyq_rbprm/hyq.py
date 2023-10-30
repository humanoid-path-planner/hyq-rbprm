#!/usr/bin/env python
# Copyright (c) 2019 CNRS
# Author: Pierre Fernbach
#
# This file is part of hpp-rbprm-robot-data.
# hpp_tutorial is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp_tutorial is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp_tutorial.  If not, see
# <http://www.gnu.org/licenses/>.

import numpy as np
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody as Parent
from pinocchio import SE3


class Robot(Parent):
    #  Information to retrieve urdf and srdf files.
    name = "hyq"
    packageName = "example-robot-data/robots/hyq_description"
    meshPackageName = "hyq_description"
    rootJointType = "freeflyer"
    urdfName = "hyq"
    urdfSuffix = "_no_sensors"
    srdfSuffix = ""
    urdfDir = "robots"

    # Information about the names of thes joints defining the limbs of the robot
    rLegId = "rfleg"
    rleg = "rf_haa_joint"
    rfoot = "rf_foot_joint"
    lLegId = "lfleg"
    lleg = "lf_haa_joint"
    lfoot = "lf_foot_joint"
    lArmId = "lhleg"
    larm = "lh_haa_joint"
    lhand = "lh_foot_joint"
    rArmId = "rhleg"
    rarm = "rh_haa_joint"
    rhand = "rh_foot_joint"

    referenceConfig = [
        0.0,
        0.0,
        0.6,
        0.0,
        0.0,
        0.0,
        1.0,
        0,  # LF
        0.7853981633974483,
        -1.5707963267948966,
        0,  # LH
        -0.7853981633974483,
        1.5707963267948966,
        0,  # RF
        0.7853981633974483,
        -1.5707963267948966,
        0,  # RH
        -0.7853981633974483,
        1.5707963267948966,
    ]

    postureWeights = [
        0,
        0,
        0,
        0,
        0,
        0,  # FF
        100.0,
        1.0,
        20.0,  # LF
        100.0,
        1.0,
        20.0,  # LH
        100.0,
        1.0,
        20.0,  # RF
        100.0,
        1.0,
        20.0,
    ]  # RH

    # informations required to generate the limbs databases the limbs :
    nbSamples = 50000
    octreeSize = 0.05
    cType = "_3_DOF"
    offset = [0.0, 0.0, -0.021]
    normal = [0, 0, 1]
    legx = 0.02
    legy = 0.02
    kinematicConstraintsPath = "package://hyq-rbprm/com_inequalities/"
    rLegKinematicConstraints = kinematicConstraintsPath + rleg + "_com_constraints.obj"
    lLegKinematicConstraints = kinematicConstraintsPath + lleg + "_com_constraints.obj"
    rArmKinematicConstraints = kinematicConstraintsPath + rarm + "_com_constraints.obj"
    lArmKinematicConstraints = kinematicConstraintsPath + larm + "_com_constraints.obj"
    minDist = 0.1
    # data used by scripts :
    limbs_names = [rLegId, lLegId, rArmId, lArmId]
    dict_limb_joint = {rLegId: rfoot, lLegId: lfoot, rArmId: rhand, lArmId: lhand}
    dict_limb_rootJoint = {rLegId: rleg, lLegId: lleg, rArmId: rarm, lArmId: larm}
    dict_limb_color_traj = {
        rfoot: [0, 1, 0, 1],
        lfoot: [1, 0, 0, 1],
        rhand: [0, 0, 1, 1],
        lhand: [0.9, 0.5, 0, 1],
    }
    FOOT_SAFETY_SIZE = 0.01
    # size of the contact surface (x,y)
    dict_size = {
        rfoot: [0.02, 0.02],
        lfoot: [0.02, 0.02],
        rhand: [0.02, 0.02],
        lhand: [0.02, 0.02],
    }
    dict_normal = {rfoot: normal, lfoot: normal, rhand: normal, lhand: normal}
    # various offset used by scripts
    MRsole_offset = SE3.Identity()
    MRsole_offset.translation = np.matrix(offset).T
    MLsole_offset = MRsole_offset.copy()
    MRhand_offset = MRsole_offset.copy()
    MLhand_offset = MRsole_offset.copy()
    dict_offset = {
        rfoot: MRsole_offset,
        lfoot: MLsole_offset,
        rhand: MRhand_offset,
        lhand: MLhand_offset,
    }

    # display transform :
    MRsole_display = SE3.Identity()
    MLsole_display = SE3.Identity()
    MRhand_display = SE3.Identity()
    MLhand_display = SE3.Identity()
    dict_display_offset = {
        rfoot: MRsole_display,
        lfoot: MLsole_display,
        rhand: MRhand_display,
        lhand: MLhand_display,
    }

    def __init__(self, name=None, load=True, client=None, clientRbprm=None):
        if name is not None:
            self.name = name
        Parent.__init__(
            self, self.name, self.rootJointType, load, client, None, clientRbprm
        )

    def loadAllLimbs(
        self, heuristic, analysis=None, nbSamples=nbSamples, octreeSize=octreeSize
    ):
        if isinstance(
            heuristic, str
        ):  # aonly one heuristic name given assign it to all the limbs
            dict_heuristic = {}
            for id in self.limbs_names:
                dict_heuristic.update({id: heuristic})
        elif isinstance(heuristic, dict):
            dict_heuristic = heuristic
        else:
            raise Exception(
                "heuristic should be either a string or a map limbId:string"
            )
        for id in self.limbs_names:
            eff = self.dict_limb_joint[id]
            self.addLimb(
                id,
                self.dict_limb_rootJoint[id],
                eff,
                self.dict_offset[eff].translation.tolist(),
                self.dict_normal[eff],
                self.dict_size[eff][0] / 2.0,
                self.dict_size[eff][1] / 2.0,
                nbSamples,
                dict_heuristic[id],
                octreeSize,
                self.cType,
                kinematicConstraintsPath=self.kinematicConstraintsPath
                + self.dict_limb_rootJoint[id]
                + "_com_constraints.obj",
                kinematicConstraintsMin=self.minDist,
            )
            if analysis:
                self.runLimbSampleAnalysis(id, analysis, True)
