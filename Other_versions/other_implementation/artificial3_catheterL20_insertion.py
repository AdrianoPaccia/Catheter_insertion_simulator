import sys
import Sofa
import SofaRuntime
import Sofa.Gui
from splib3.numerics import RigidDof

 
catheter_vars = {
    "length":200,   #0.45 m
    "controlPoints":100,
    "nodePoses":[],
    "meshLines":[],
    "radius":2.5,
    "FEMradius":2.5,
    "innerRadius":2.0,
    #"massDensity": 0.002145,    #21450 Kg/m3 (platine)
    "massDensity":  0.000155,    # 1550 Kg/m3  (silicone), 
    "airDensity":   0.0000001,   # 1 Kg/m3  (silicone)
    #"youngModulus":16.4e4, #16400 MPa (platine)
    "youngModulus":1e4,   # 10000 MPa (silicone)
    "mass":0.01,
    "poissonRatio":0.49
}
volume = catheter_vars['length'] * (catheter_vars['radius']**2) *3.14
massMaterial = catheter_vars['massDensity'] * catheter_vars['length'] * ((catheter_vars['radius'] - catheter_vars['innerRadius'])**2) *3.14
massAir = catheter_vars['airDensity'] *catheter_vars['length'] * (catheter_vars['innerRadius']**2) *3.14
realDensity = (massMaterial+massAir)/volume

object_pose = {'dx':300.0, 'dy':0.0, 'dz':-4.0, 'rx':90.0, 'ry':0.0, 'rz':0.0}    

controller_vars = {'movement_incr_mm':0.2, 'rotation_incr_degrees':10, 'dt':0.01}

        

def createScene(root):
    root.findData('dt').value=controller_vars['dt']
    root.findData('gravity').value=[0, -9806,0]#[0, -9806, 0]
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
    controlNode.addObject('RequiredPlugin', name="SofaBoundaryCondition")

    controlNode.addObject('RequiredPlugin', name="SofaSimpleFem")
    controlNode.addObject('RequiredPlugin', name="SofaRigid")
    controlNode.addObject('RequiredPlugin', name="SofaMiscForceField")

    root.addObject('VisualStyle', displayFlags='showInteractionForceFields hideBoundingCollisionModels showForceFields')

    #showCollisionModels
    root.addObject('FreeMotionAnimationLoop',parallelCollisionDetectionAndFreeMotion="True", parallelODESolving="True")
    root.addObject('GenericConstraintSolver', name="ConstraintSolver", multithreading="True", tolerance="1e-3", maxIterations="100", computeConstraintForces="true")

    root.addObject('DefaultPipeline', depth="6", verbose="0", draw="0")
    root.addObject('BruteForceBroadPhase')
    root.addObject('BVHNarrowPhase')
    #root.addObject('MinProximityIntersection', name="Proximity",alarmDistance= 1.5, contactDistance=1)
    # alarmDistance= 0.5+catheter_vars['radius'], contactDistance=catheter_vars['radius'])
    root.addObject('LocalMinDistance', name="Proximity", alarmDistance= 1.5, contactDistance=1.0, angleCone="0.5")

    root.addObject('CollisionResponse', name="Response", response="FrictionContactConstraint")
   
    root.addObject( MyCatheterController(name="MyCatheterController") )

    gripperNode = root.addChild('gripperNode')
    gripperNode.addObject('MechanicalObject', name="Gripper",template="Rigid3d", position="0 0 0 0 0 0 1")

    Catheter = root.addChild('Catheter')
    Catheter.addObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1, printLog='false')
    Catheter.addObject('CGLinearSolver', iterations="25", tolerance="1.0e-5", threshold="1.0e-5")

    Catheter.addObject('RegularGridTopology', name="grid", nx=catheter_vars['controlPoints'], ny="1", nz="1", xmin="0", xmax=catheter_vars['length'], ymin="0", ymax="0", zmin="0", zmax="0")
    Catheter.addObject('MechanicalObject', template="Rigid3d", name="DOFs", showObject="true", showObjectScale="1", position="@grid.position")
    Catheter.addObject('PartialFixedConstraint', indices="0", fixedDirections="0 1 0 1 1 1")

    Catheter.addObject("UncoupledConstraintCorrection")
    FEMNode = Catheter.addChild("FEM")
    
    FEMNode.addObject("CylinderGridTopology", name="FEM_grid", nx="4", ny="4", nz=catheter_vars['controlPoints'], radius=1,length=catheter_vars['length'],axis="1 0 0")
    FEMNode.addObject('MechanicalObject', template="Vec3d", name="FEM_DOFs", position="@FEM_grid.position")
    FEMNode.addObject('HexahedronSetGeometryAlgorithms')
    FEMNode.addObject('MeshMatrixMass',name='mass', massDensity=realDensity)
    FEMNode.addObject('HexahedronFEMForceField', name="FEM", youngModulus= catheter_vars['youngModulus'], poissonRatio= catheter_vars['poissonRatio'], method="large", printLog="false")
    
    
    cylN = root.Catheter.FEM.FEM_grid.n.value
    print(cylN)
    rigidIndexPerPointString = ""
    for i in range(0, cylN[2]):
        for j in range(0, cylN[0] * cylN[1]):
            rigidIndexPerPointString += f" {i}"
    #print(rigidIndexPerPointString)

    
    
    FEMNode.addObject('RigidMapping', globalToLocalCoords="true", rigidIndexPerPoint=rigidIndexPerPointString)
    FEMNode.addObject('TriangleCollisionModel')
    FEMNode.addObject('PointCollisionModel')

    visu = FEMNode.addChild("Visual")
    visu.addObject('OglModel', name="VisualModel", material="Default Diffuse 1 0.8 0.8 0.8 1 Ambient 1 0.2 0.2 0.2 1 Specular 0 0.8 0.8 0.8 1 Emissive 0 0.8 0.8 0.8 1 Shininess 0 45")
    visu.addObject('IdentityMapping',  input="@..", output="@Visual")
    
    root.addObject('AttachConstraint', object1="@gripperNode/Gripper", object2="@Catheter/DOFs", indices1="0", indices2="0", constraintFactor="0", twoWay="false")
    
    veinsNode = root.addChild('Veins')
    veinsNode.addObject('MeshOBJLoader', name='loader', filename='mesh/vessel3.obj',  flipNormals="1", triangulate="1")
    veinsNode.addObject('MeshTopology',src = '@loader', triangles='@loader.triangles' )
    veinsNode.addObject('MechanicalObject', showObject="1", name='dofs', template='Vec3d', 
    dx=object_pose['dx'], dy=object_pose['dy'], dz=object_pose['dz'], 
    rx=object_pose['rx'], ry=object_pose['ry'],rz=object_pose['rz'], scale = 10)
    veinsNode.addObject('TriangleCollisionModel',contactStiffness=5)
    veinsNode.addObject('LineCollisionModel',contactStiffness=5)
    veinsNode.addObject('PointCollisionModel',contactStiffness=5)
    veinsNode.addObject('OglModel', name='Visual', src='@loader', color='1 0 0 0.8',
    dx=object_pose['dx'], dy=object_pose['dy'], dz=object_pose['dz'], 
    rx=object_pose['rx'], ry=object_pose['ry'],rz=object_pose['rz'],  scale = 10)



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
        root = self.getContext()
        
        if(self.movementSpeedCmd != "N"):
            speedDirection = 1 
            if(self.movementSpeedCmd == "D"):
                speedDirection = -1
            if(self.movementSpeedCmd == "S"):
                speedDirection = 0
            
            rigidBody = RigidDof(root.Catheter.DOFs)
            rigidBody = RigidDof(root.gripperNode.Gripper)
            translationDirection = [speedDirection * controller_vars['movement_incr_mm'],0,0]
            rigidBody.translate(translationDirection)
            self.movementSpeedCmd = "N"
        
        if(self.rotationSpeedCmd != "N"):
            rotDirection = 1
            if(self.rotationSpeedCmd == "D"):
                rotDirection = -1
            if(self.rotationSpeedCmd == "S"):
                rotDirection = 0
            
            rigidBody = RigidDof(root.gripperNode.Gripper)
             
            rigidBody.rotateAround( [1,0,0], rotDirection * controller_vars['rotation_incr_degrees'] * 3.14 / 180)
            self.rotationSpeedCmd = "N"

    
        
        
            

def main():
    root = Sofa.Core.Node('root')

    createScene(root)

    Sofa.Simulation.init(root)
    Sofa.Gui.GUIManager.Init('myscene', 'qglviewer')
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 1080)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()