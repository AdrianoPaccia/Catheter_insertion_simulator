

from lzma import MODE_FAST, MODE_NORMAL
import sys
import Sofa
import SofaRuntime
import Sofa.Gui
from splib3.numerics import RigidDof
from splib3.numerics.vec3 import Vec3
from splib3.numerics.quat import Quat
import udpComms

import time

import numpy as np
from extractions import *

force_threshold = 1e-6
mode = 3 #modality of force approximation
grouping_model = 1 # mode for the group average of the clusters
force_render_vector = [0.0,0.0,0.0]
force_scale = 10
rotations_sensor = [0.0,0.0,0.0]
translations_sensor = [0.0,0.0,0.0]
old_positions = [0.0,0.0,0.0]
offset_positions = [0.0,-65.51,-88.11]
old_rotations = [0.0,0.0,0.0]

rotations_scale = [1/6, 1, 1]
translations_scale = [1,1,1/20]

force_indicator_pose = [[0.0,0.0,0.0],[0.0,0.0,0.0]]



oldt = 0
newt = 0
flagDisableMovement = True
flagFirstPacketDiscarded = False
catheter_material = 'platine'

catheter_vars = {
    "length":200,   #0.45 m
    "controlPoints":100,
    "nodePoses":[],
    "meshLines":[],
    "radius":1.0,
    "FEMradius":2.5,
    "innerRadius":0.8,
    #"massDensity": 0.002145,    #21450 Kg/m3 (platine)
    "massDensity":  0.000155,    # 1550 Kg/m3  (silicone), 
    "airDensity":   0.0000001,   # 1 Kg/m3  (silicone)
    #"youngModulus":16.4e4, #16400 MPa (platine)
    "youngModulus":1e4,   # 10000 MPa (silicone)
    "mass":0.01,
    "poissonRatio":0.49
}

mass = catheter_vars['massDensity'] * catheter_vars['length'] * ((catheter_vars['radius'] - catheter_vars['innerRadius'])**2) *3.14

object_pose = {'dx':290.0, 'dy':-23.0, 'dz':-40.0, 'rx':90.0, 'ry':110.0, 'rz':-27.0}     #terzo canale - scale=1.5
controller_vars = {'movement_incr_mm':0.5, 'rotation_incr_degrees':10, 'dt':0.001}

def generateCatheterNodes():
    distanceBetweenNodes = catheter_vars['length'] / (catheter_vars['controlPoints']-1)
    for i in range(catheter_vars['controlPoints']):
        nodePose = [distanceBetweenNodes * i, 0, 0, 0, 0, 0, 1]
        catheter_vars['nodePoses'] += nodePose
    
    halfIndex = 0
    for i in range(catheter_vars['controlPoints'] * 2):
        if (i % 2) == 0: #num pari
            catheter_vars['meshLines'] += [halfIndex]
        else: #num dispari
            halfIndex += 1
            if(halfIndex >= catheter_vars['controlPoints']):
                print("LAST INDEX REACHED")
            else: catheter_vars['meshLines'] += [halfIndex] #comment with else: maybe
        

def createScene(root):
    generateCatheterNodes()
    root.findData('dt').value=controller_vars['dt']
    root.findData('gravity').value=[0, -9806, 0]#[0, -9806, 0]
    #root.findData('gravity').value=[0, 0, 0]

    controlNode = root.addChild('RequiredPlugin')
    controlNode.addObject('RequiredPlugin', name="SofaConstraint")
    controlNode.addObject('RequiredPlugin', name="SofaImplicitOdeSolver")
    controlNode.addObject('RequiredPlugin', name="SofaLoader")
    controlNode.addObject('RequiredPlugin', name="SofaOpenglVisual")
    controlNode.addObject('RequiredPlugin', name="SofaMeshCollision")

    controlNode.addObject('RequiredPlugin', name="SofaGeneralLinearSolver")
    controlNode.addObject('RequiredPlugin', name="SofaGeneralObjectInteraction")
    controlNode.addObject('RequiredPlugin', name="SofaGeneralSimpleFem")
    controlNode.addObject('RequiredPlugin', name="SofaGeneralTopology")
    controlNode.addObject('RequiredPlugin', name="SofaMiscMapping")

    root.addObject('VisualStyle', displayFlags='showInteractionForceFields hideBoundingCollisionModels showForceFields')

    #showCollisionModels
    root.addObject('FreeMotionAnimationLoop',parallelCollisionDetectionAndFreeMotion="True", parallelODESolving="True")
    root.addObject('GenericConstraintSolver', name="ConstraintSolver", multithreading="True", tolerance="1e-3", maxIterations="100", computeConstraintForces="true")

    root.addObject('DefaultPipeline', depth="6", verbose="0", draw="0")
    root.addObject('BruteForceBroadPhase')
    root.addObject('BVHNarrowPhase')
    root.addObject('MinProximityIntersection', name="Proximity",alarmDistance= 1.5, contactDistance=1)
    # alarmDistance= 0.5+catheter_vars['radius'], contactDistance=catheter_vars['radius'])
    #root.addObject('LocalMinDistance', name="Proximity", alarmDistance= 1.5, contactDistance=1, angleCone="0.5")

    root.addObject('CollisionResponse', name="Response", response="FrictionContactConstraint")
   
    root.addObject( MyCatheterController(name="MyCatheterController") )

    forceIndicator = root.addChild('ForceIndicator')
    inputMsh = "mesh/arrow.obj"

    #forceIndicator.addObject('EulerImplicitSolver', rayleighStiffness=0, rayleighMass=0, printLog='false')
    #forceIndicator.addObject('CGLinearSolver', iterations="25", tolerance="1.0e-3", threshold="1.0e-3")

    forceIndicator.addObject('MeshObjLoader', name="loader",
                        filename=inputMsh, triangulate=True,
                        scale=0.5)
    forceIndicator.addObject('Mesh', src="@loader")
    forceIndicator.addObject('MechanicalObject', name="forceObject",template="Rigid3d")
    forceVisu = forceIndicator.addChild('forceVisu')
    forceVisu.addObject('OglModel', name='Visual', src='@../loader', color=[0.0,1.0,0.0,1.0])
    forceVisu.addObject('RigidMapping',  input="@../forceObject", output="@Visual")



    gripperNode = root.addChild('gripperNode')
    gripperNode.addObject('MechanicalObject', name="Gripper",template="Rigid3d", position="0 0 0 0 0 0 1")

    Catheter = root.addChild('Catheter')
    Catheter.addObject('EulerImplicitSolver', rayleighStiffness=0, rayleighMass=0, printLog='false')
    #Catheter.addObject('CGLinearSolver', iterations="25", tolerance="1.0e-9", threshold="1.0e-9")
    Catheter.addObject('BTDLinearSolver', template="BTDMatrix6d")

    Catheter.addObject('MechanicalObject', template='Rigid3d', name='DOFs', position=
    catheter_vars['nodePoses'])
    Catheter.addObject('MeshTopology', name='lines', lines=catheter_vars['meshLines'])
    Catheter.addObject("UniformMass",totalMass=mass)
    Catheter.addObject("BeamFEMForceField", name="FEM", template="Rigid3d", radius=catheter_vars['FEMradius'] , youngModulus=catheter_vars['youngModulus'], poissonRatio=catheter_vars['poissonRatio'])
    #Catheter.addObject('PartialFixedConstraint', indices="0", fixedDirections="0 1 0 1 1 1")

    Catheter.addObject("UncoupledConstraintCorrection")
    
    Catheter_collis = Catheter.addChild('Collision')
    Catheter_collis.addObject("CylinderGridTopology", name="cylContainer", nx="4", ny="4", nz=catheter_vars['controlPoints'], radius=catheter_vars['radius'],length=catheter_vars['length'],axis="1 0 0")
    Catheter_collis.addObject("MechanicalObject", name="collisionDOFs")
    Catheter_collis.addObject("BeamLinearMapping", isMechanical="true", template="Rigid3d,Vec3d", input="@../DOFs" ,output="@collisionDOFs", localCoord="false")
    Catheter_collis.addObject("TriangleCollisionModel")
    Catheter_collis.addObject("LineCollisionModel") 
    Catheter_collis.addObject("PointCollisionModel")  

    visu = Catheter_collis.addChild("Visual")
    visu.addObject('OglModel', name="VisualModel", material="Default Diffuse 1 0.8 0.8 0.8 1 Ambient 1 0.2 0.2 0.2 1 Specular 0 0.8 0.8 0.8 1 Emissive 0 0.8 0.8 0.8 1 Shininess 0 45")
    visu.addObject('IdentityMapping',  input="@..", output="@Visual")

    root.addObject('AttachConstraint', object1="@gripperNode/Gripper", object2="@Catheter/DOFs", indices1="0", indices2="0", constraintFactor="1", twoWay="false")
    
    
    veinsNode = root.addChild('Veins')
    veinsNode.addObject('MeshSTLLoader', name='loader', filename='mesh/carotids.stl',  flipNormals=0, triangulate="1")
    veinsNode.addObject('MeshTopology',src = '@loader', triangles='@loader.triangles' )
    veinsNode.addObject('MechanicalObject', showObject="1", name='dofs', template='Vec3d', 
    dx=object_pose['dx'], dy=object_pose['dy'], dz=object_pose['dz'], 
    rx=object_pose['rx'], ry=object_pose['ry'],rz=object_pose['rz'], scale = 1.5)
    veinsNode.addObject('TriangleCollisionModel',contactStiffness=5)
    veinsNode.addObject('LineCollisionModel',contactStiffness=5)
    veinsNode.addObject('PointCollisionModel',contactStiffness=5)
    veinsNode.addObject('OglModel', name='Visual', src='@loader', color='1 0 0 0.8',
    dx=object_pose['dx'], dy=object_pose['dy'], dz=object_pose['dz'], 
    rx=object_pose['rx'], ry=object_pose['ry'],rz=object_pose['rz'],  scale = 1.5)

    

class MyCatheterController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, *kwargs)
        self.movementSpeedCmd = "N"
        self.rotationSpeedCmd = "N"
        self.directionCmd = "N"
        self.printLog = 0

    def onKeypressedEvent(self, event):
        key = event['key']
        print(ord(key))
        
        if ord(key) == 77:  # control M
            self.printLog = 1
            print("press step to print next collision data")

        if ord(key) ==57: #control 9
            self.movementSpeedCmd = "U"
            print("forward movement")
        if ord(key) ==  55: #control 7
            self.movementSpeedCmd = "D"
            print("backward movement")
        if ord(key) == 56: # control 8
            self.movementSpeedCmd = "S"
            print("stop mvspd")
        if ord(key) == 54: #control 6
            self.rotationSpeedCmd = "U"
            print("rotation clockwise")
        if ord(key) == 52: #control 4
            self.rotationSpeedCmd = "D"
            print("decrease counterclockwise")
        if ord(key) == 53: #control 5
            self.rotationSpeedCmd = "S"
            print("zero rotation")
        
        if ord(key) == 49: #control 1
            print("move x")
            self.directionCmd = "X"
        if ord(key) == 50: #control 2
            print("move y")
            self.directionCmd = "Y"
        if ord(key) == 51: #control 3
            print("move z")
            self.directionCmd = "Z"
        
            

    def createDictEntry(self,i, directions,force_num,force_value, force_components):
            dict_entry = {'point':i,
            'node_ID': i // self.deltaGroups, 
            'directions': directions, 
            'force num': force_num,
            'value': force_value,
            'components': force_components}
            return dict_entry

    def createDictEntryQuat(self,i, directions,quat,force_num,force_value, force_components):
            dict_entry = {'node':i,
            'directions': directions, 
            'force num': force_num,
            'quaternions': quat,
            'value': force_value,
            'components': force_components}
            return dict_entry
                
    def onAnimateEndEvent(self, params):
        global force_render_vector, force_scale
        global translations_sensor , old_positions, offset_positions, translations_scale
        global rotations_sensor, old_rotations, rotations_scale
        global oldt, newt
        global flagDisableMovement, flagFirstPacketDiscarded
        global force_indicator_pose
        global mode, grouping_model

        AUTOMATIC_MOVEMENT = True
 
        root = self.getContext()

        #print(flagDisableMovement)
        if(flagDisableMovement==False or AUTOMATIC_MOVEMENT == True):
            delta_translation = np.array(translations_sensor) - np.array(old_positions)
            rigidBody = RigidDof(root.gripperNode.Gripper)
            translationDirection = [-delta_translation[2] * translations_scale[2],0,0]
            if(AUTOMATIC_MOVEMENT==True):
                translationDirection[0] = 0.1
            rigidBody.translate(translationDirection)
            old_positions = translations_sensor

            rigidBody = RigidDof(root.gripperNode.Gripper)
            delta_rot =  np.array(rotations_sensor) - np.array(old_rotations)
            rigidBody.rotateAround( [ 1, 0, 0], delta_rot[2] * rotations_scale[2])
            rigidBody.rotateAround( [ 0, 0, -1], delta_rot[0] * rotations_scale[0])
            #rigidBody.rotateAround( [ 0, -1, 0], delta_rot[1] * rotations_scale[1] )
            old_rotations = rotations_sensor
        else:
            old_rotations = rotations_sensor
            old_positions = translations_sensor
    

        # if(self.printLog.value == 1):
        if(True):
            self.printLog = 0
            
            USE_COLLISION_MODEL_FOR_FORCE_EVAL = False

            if(USE_COLLISION_MODEL_FOR_FORCE_EVAL):
                force_directions = root.Catheter.Collision.collisionDOFs.constraint
                
                # print("-----FORCE DIRECTIONS RAW-----")
                # print(force_directions.value)
                # print(type(force_directions.value))
                # print("------------------------------")

                self.collisionMatrix = force_directions.value.splitlines()
                self.collisionMatrix = [line.split() for line in self.collisionMatrix]
                self.collisionMatrix = [[float(Value) for Value in self.collisionMatrix[i]] for i in range(len(self.collisionMatrix))]
                # print("----EXTRACTED LINES FROM FORCE DIRECTIONS----")
                # for line in self.collisionMatrix:
                #     print(line)
                # print("---------------------------------------------")



                force_values = root.ConstraintSolver.constraintForces.value.tolist()
                # print("----FORCE VALUES RAW----")
                # print(force_values)
                # print(type(force_values))
                # print("------------------------")


                self.numberDOFs = 3

                self.extractedCollisionMatrix = []
            
                self.cylinderContainer = root.Catheter.Collision.cylContainer.n.value
                self.collisionPoints = self.cylinderContainer[0] * self.cylinderContainer[1] * self.cylinderContainer[2]
                self.collisionGroups = catheter_vars['controlPoints']
                self.deltaGroups = self.cylinderContainer[0] * self.cylinderContainer[1]

                self.collisionDict = [ [] for i in range(self.collisionPoints)]

                for i in range(len(self.collisionMatrix)):
                    numberElements = int(self.collisionMatrix[i][1])

                    if numberElements == 1:
                        element1 = int(self.collisionMatrix[i][2])
                        direction1 = [self.collisionMatrix[i][3+j] for j in range(3)]
                        forcenum = int(self.collisionMatrix[i][0])
                        forceval = force_values[forcenum]
                        
                        forcecomps = [ (direction1[j] * forceval) for j in range(3)]

                        if(forceval > 0):
                            self.collisionDict[element1].append(self.createDictEntry(element1,direction1,forcenum,forceval,forcecomps))


                    if numberElements == 2:
                        
                        element1 = int(self.collisionMatrix[i][2])
                        direction1 = [self.collisionMatrix[i][3+j] for j in range(3)]
                        forcenum = int(self.collisionMatrix[i][0])
                        forceval = force_values[forcenum]
                        
                        forcecomps = [ (direction1[j] * forceval) for j in range(3)]
                        if(forceval > 0):
                            self.collisionDict[element1].append(self.createDictEntry(element1,direction1,forcenum,forceval,forcecomps))
            
                        element2 = int(self.collisionMatrix[i][6])
                        direction2 = [self.collisionMatrix[i][7+j] for j in range(3)]
                        forcenum = int(self.collisionMatrix[i][0])
                        forceval = force_values[forcenum]
                        forcecomps = [ (direction2[j] * forceval) for j in range(3)]
                        if(forceval > 0):
                            self.collisionDict[element2].append(self.createDictEntry(element2,direction2,forcenum,forceval,forcecomps))
                        



                    if numberElements == 3:
                        
                        
                        element1 = int(self.collisionMatrix[i][2])
                        direction1 = [self.collisionMatrix[i][3+j] for j in range(3)]
                        forcenum = int(self.collisionMatrix[i][0])
                        forceval = force_values[forcenum]
                        forcecomps = [ (direction1[j] * forceval) for j in range(3)]
                        if(forceval > 0):
                            self.collisionDict[element1].append(self.createDictEntry(element1,direction1,forcenum,forceval,forcecomps))
            
                        element2 = int(self.collisionMatrix[i][6])
                        direction2 = [self.collisionMatrix[i][7+j] for j in range(3)]
                        forcenum = int(self.collisionMatrix[i][0])
                        forceval = force_values[forcenum]
                        forcecomps = [ (direction2[j] * forceval) for j in range(3)]
                        if(forceval > 0):
                            self.collisionDict[element2].append(self.createDictEntry(element2,direction2,forcenum,forceval,forcecomps))

                        element3 = int(self.collisionMatrix[i][10])
                        direction3 = [self.collisionMatrix[i][11 +j] for j in range(3)]
                        forcenum = int(self.collisionMatrix[i][0])
                        forceval = force_values[forcenum]
                        forcecomps = [ (direction3[j] * forceval) for j in range(3)]
                        
                        if(forceval > 0):
                            self.collisionDict[element3].append(self.createDictEntry(element3,direction3,forcenum,forceval,forcecomps))
            else:
                force_directions = root.Catheter.DOFs.constraint
                """
                print("-----FORCE DIRECTIONS RAW-----")
                print(force_directions.value)
                print(type(force_directions.value))
                print("------------------------------")
                """

                self.collisionMatrix = force_directions.value.splitlines()
                self.collisionMatrix = [line.split() for line in self.collisionMatrix]
                self.collisionMatrix = [[float(Value) for Value in self.collisionMatrix[i]] for i in range(len(self.collisionMatrix))]
                """
                print("----EXTRACTED LINES FROM FORCE DIRECTIONS----")
                for line in self.collisionMatrix:
                    print(line)
                print("---------------------------------------------")
                """



                force_values = root.ConstraintSolver.constraintForces.value.tolist()
                """
                print("----FORCE VALUES RAW----")
                print(force_values)
                print(type(force_values))
                print("------------------------")
                """


                self.numberDOFs = 3

                self.extractedCollisionMatrix = []
            
                self.cylinderContainer = root.Catheter.Collision.cylContainer.n.value
                self.collisionPoints = self.cylinderContainer[0] * self.cylinderContainer[1] * self.cylinderContainer[2]
                self.collisionGroups = catheter_vars['controlPoints']
                self.deltaGroups = self.cylinderContainer[0] * self.cylinderContainer[1]

                self.collisionDict = [ [] for i in range(self.collisionGroups)]

                for i in range(len(self.collisionMatrix)):
                    numberElements = int(self.collisionMatrix[i][1])

                    if numberElements == 1:
                        element1 = int(self.collisionMatrix[i][2])
                        direction1 = [self.collisionMatrix[i][3+j] for j in range(3)]
                        quaternion1 = [self.collisionMatrix[i][6+j] for j in range(3)]
                        forcenum = int(self.collisionMatrix[i][0])
                        forceval = force_values[forcenum]
                        
                        forcecomps = [ (direction1[j] * forceval) for j in range(3)]

                        if(forceval > 0):
                            self.collisionDict[element1].append(self.createDictEntryQuat(element1,direction1,quaternion1,forcenum,forceval,forcecomps))


                    if numberElements == 2:
                        
                        element1 = int(self.collisionMatrix[i][2])
                        direction1 = [self.collisionMatrix[i][3+j] for j in range(3)]
                        quaternion1 = [self.collisionMatrix[i][6+j] for j in range(3)]
                        forcenum = int(self.collisionMatrix[i][0])
                        forceval = force_values[forcenum]
                        
                        forcecomps = [ (direction1[j] * forceval) for j in range(3)]
                        if(forceval > 0):
                            self.collisionDict[element1].append(self.createDictEntryQuat(element1,direction1,quaternion1,forcenum,forceval,forcecomps))
            
                        element2 = int(self.collisionMatrix[i][9])
                        direction2 = [self.collisionMatrix[i][10+j] for j in range(3)]
                        quaternion2 = [self.collisionMatrix[i][13+j] for j in range(3)]
                        forcenum = int(self.collisionMatrix[i][0])
                        forceval = force_values[forcenum]
                        forcecomps = [ (direction2[j] * forceval) for j in range(3)]
                        if(forceval > 0):
                            self.collisionDict[element2].append(self.createDictEntryQuat(element2,direction2,quaternion2,forcenum,forceval,forcecomps))

                    if numberElements == 3:
                        element1 = int(self.collisionMatrix[i][2])
                        direction1 = [self.collisionMatrix[i][3+j] for j in range(3)]
                        quaternion1 = [self.collisionMatrix[i][6+j] for j in range(3)]
                        forcenum = int(self.collisionMatrix[i][0])
                        forceval = force_values[forcenum]
                        
                        forcecomps = [ (direction1[j] * forceval) for j in range(3)]
                        if(forceval > 0):
                            self.collisionDict[element1].append(self.createDictEntryQuat(element1,direction1,quaternion1,forcenum,forceval,forcecomps))
            
                        element2 = int(self.collisionMatrix[i][9])
                        direction2 = [self.collisionMatrix[i][10+j] for j in range(3)]
                        quaternion2 = [self.collisionMatrix[i][13+j] for j in range(3)]
                        forcenum = int(self.collisionMatrix[i][0])
                        forceval = force_values[forcenum]
                        forcecomps = [ (direction2[j] * forceval) for j in range(3)]
                        if(forceval > 0):
                            self.collisionDict[element2].append(self.createDictEntryQuat(element2,direction2,quaternion2,forcenum,forceval,forcecomps))
                        
                        element3 = int(self.collisionMatrix[i][16])
                        direction3 = [self.collisionMatrix[i][17+j] for j in range(3)]
                        quaternion3 = [self.collisionMatrix[i][20+j] for j in range(3)]
                        forcenum = int(self.collisionMatrix[i][0])
                        forceval = force_values[forcenum]
                        forcecomps = [ (direction3[j] * forceval) for j in range(3)]
                        if(forceval > 0):
                            self.collisionDict[element3].append(self.createDictEntryQuat(element3,direction3,quaternion3,forcenum,forceval,forcecomps))
                        

                
            force_plot = handle_forces(self.collisionDict,mode,grouping_model,force_threshold, 0)

            for i in range(len(force_plot)):
                visualIndicator = RigidDof(root.ForceIndicator.forceObject)
                node_number = force_plot[i][0]
                node_position = root.Catheter.DOFs.position[node_number]
                #print(f'move indicator to node: {node_number} at position {node_position}')
                rest_position = root.ForceIndicator.forceObject.rest_position.value
                #print(f'rest pos: {rest_position}')
                root.ForceIndicator.forceObject.position = rest_position
                if(len(force_plot) > 0):
                    node_position_new = np.array(node_position)
                    node_position_new[1] -= 5 #shifto indicatore sulla y un po' in basso
                    visualIndicator.translate(node_position_new[0:3])

                    node_orientation_new = np.array(force_plot[i][1])/np.linalg.norm(force_plot[i][1])
                    zero_vec = Vec3(visualIndicator.getUp())
                    new_orientation = Vec3(node_orientation_new)
                    q = Quat.createFromVectors(new_orientation,zero_vec)
                    p = root.ForceIndicator.forceObject.position
                    pq = p.value[0]
                    p.value = [list(pq[:3]) + list(q)]
                   

            if ( (mode == 1 or mode == 2 or mode == 3) and len(force_plot) > 0):
                force_render_vector = np.round_(((force_plot[0][1] * 0.001) / controller_vars['dt']) * force_scale , 5)
                temp_vector = np.copy(force_render_vector)
                force_render_vector[0] = temp_vector[2]
                force_render_vector[1] = temp_vector[1]
                force_render_vector[2] = -temp_vector[0]
                print(force_render_vector)
           

            else:
                force_render_vector = [0.0,0.0,0.0]
                


        stats,address = udpComms.receiveUDPStats()
        
        if(stats != None and address != None):
            if(flagFirstPacketDiscarded == True):
                newt = time.time()
                print(f'received {stats} from {address} with dt {newt-oldt}')
                oldt = time.time()
                print(f'sending {force_render_vector}')
                translations_sensor = stats[0:3]
                translations_sensor[0] -= offset_positions[0]
                translations_sensor[1] -= offset_positions[1]
                translations_sensor[2] -= offset_positions[2]
                
                rotations_sensor = stats[3:6]

                if(stats[6]>0):
                    flagDisableMovement=False
                else:
                    flagDisableMovement=True
                #print(f'eepos: {ee_position} , rots:{rotations_sensor}')
                udpComms.sendUDPForce(force_render_vector,address)

                if(stats[6]==2):
                    if(mode==3):
                        mode = 1
                    else:
                        mode += 1
                    if(mode==1):
                        print('Mode 1 activated: only the interaction force acting on the tip is mapped into the Geomagic.')
                    elif(mode==2):
                        print('Mode 2 activated: only the strongest interaction force is mapped into the Geomagic.')
                    elif(mode==3):
                        print('Mode 3 activated: the sum of all forces acting on the cathter is mapped into the Geomagic')
                    else:
                        print('NO compatible mode has been chosen')

                        
                
                if(stats[6]==3):
                    grouping_model = not grouping_model
                    if(grouping_model==False):
                        print('The grouping force of the cluster would be the highest in absolute value')
                    else:
                        print('The grouping force of the cluster would be theone in the centre of gravity of the catheter')

            else:
                flagFirstPacketDiscarded = True
                udpComms.sendUDPForce(force_render_vector,address)
                print("Skipping first packet!")
        
        
            

def main():
    root = Sofa.Core.Node('root')

    createScene(root)

    Sofa.Simulation.init(root)
    Sofa.Gui.GUIManager.Init('myscene', 'qglviewer')
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 1080)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()