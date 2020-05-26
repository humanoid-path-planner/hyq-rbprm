from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.tools.plot_analytics  import plotOctreeValues

packageName = "hyq_description"
meshPackageName = "hyq_description"
rootJointType = "freeflyer"

#  Information to retrieve urdf and srdf files.
urdfName = "hyq"
urdfSuffix = ""
srdfSuffix = ""

#  This time we load the full body model of HyQ
fullBody = FullBody () 
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-2,5, -1, 1, 0.3, 4])

#  Setting a number of sample configurations used
nbSamples = 50000



rootName = 'base_joint_xyz'

#  Creating limbs
# cType is "_3_DOF": positional constraint, but no rotation (contacts are punctual)
cType = "_3_DOF"
# string identifying the limb
rfLegId = 'rfleg'
# First joint of the limb, as in urdf file
rfLeg = 'rf_haa_joint'
# Last joint of the limb, as in urdf file
rfFoot = 'rf_foot_joint'
# Specifying the distance between last joint and contact surface
offset = [0.,-0.021,0.]
# Specifying the contact surface direction when the limb is in rest pose
normal = [0,1,0]
# Specifying the rectangular contact surface length
legx = 0.02; legy = 0.02
# remaining parameters are the chosen heuristic (here, manipulability), and the resolution of the octree (here, 10 cm).
fullBody.addLimb(rfLegId,rfLeg,rfFoot,offset,normal, legx, legy, nbSamples, "manipulability", 0.05, cType)

lhLegId = 'lhleg'
lhLeg = 'lh_haa_joint'
lhFoot = 'lh_foot_joint'
fullBody.addLimb(lhLegId,lhLeg,lhFoot,offset,normal, legx, legy, nbSamples, "manipulability", 0.05, cType)

rhLegId = 'rhleg'
rhLeg = 'rh_haa_joint'
rhFoot = 'rh_foot_joint'
fullBody.addLimb(rhLegId,rhLeg,rhFoot,offset,normal, legx, legy, nbSamples, "manipulability", 0.05, cType)

lfLegId = 'lfleg'
lfLeg = 'lf_haa_joint'
lfFoot = 'lf_foot_joint'
fullBody.addLimb(lfLegId,lfLeg,lfFoot,offset,normal, legx, legy, nbSamples, "manipulability", 0.05, cType)


q_0 = fullBody.getCurrentConfig(); 


def runall(lid, dbName):
	fullBody.runLimbSampleAnalysis(lid, "isotropy", True)
	fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
	#~ fullBody.runLimbSampleAnalysis(lid, "selfCollisionProbability", False)
	fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
	fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", True)
	fullBody.saveLimbDatabase(lid, dbName)

runall(rfLegId, './hyq_'+rfLegId+'.db')
runall(lhLegId, './hyq_'+lhLegId+'.db')
runall(rhLegId, './hyq_'+rhLegId+'.db')
runall(lfLegId, './hyq_'+lfLegId+'.db')
#plotOctreeValues(fullBody, "minimumSingularValue", rarmId)

