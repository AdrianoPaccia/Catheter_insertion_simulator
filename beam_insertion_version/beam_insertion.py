import sys
import Sofa
import SofaRuntime
import Sofa.Gui
import numpy

from splib3.numerics import RigidDof, Quat


# decomment this 2 rows for a straight down simulation
#object_pose = {'dx':50, 'dy':0.0, 'dz':50.0, 'rx':0.0, 'ry':0.0, 'rz':180.0}
#catheter_vars = {'dx':0.0, 'dy':50.0, 'dz':0.0, 'rx':0.0, 'ry':0.0, 'rz':-90.0, 'radius': 1.0, 'length': 200, 'numnodes':100}

# decomment this 2 rows for a slightly angled beam insertion on a dummy model
object_pose = {'dx':100, 'dy':0.0, 'dz':100.0, 'rx':0.0, 'ry':90.0, 'rz':0.0}
catheter_vars = {'dx':0.0, 'dy':7.0, 'dz':65.0, 'rx':0.0, 'ry':-40.0, 'rz':0.0, 'radius': 1.0, 'length': 200, 'numnodes':100}


controller_vars = {'step':0.01, 'speed':4, 'xtip':0.1}
gravity = '0 0 0'

class MyController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, *kwargs)
        self.collisionMatrix = []
        self.sortedCollisionMatrix = []
        self.printLog.value = 0
        self.printspeeds = -1
        self.movementSpeedCmd = "N"
        self.rotationSpeedCmd = "N"
        self.directionCmd = "N"
        self.movementSpeedAbsIncrease = 1.0
        self.rotationSpeedAbsIncrease = 0.1

    def onKeypressedEvent(self, event):
        key = event['key']
        print(ord(key))
        if ord(key) == 77:  # control punto
            self.printLog = 1
            print("press step to print next collision data")
        if ord(key) == 48: #control 0
            print("print controller data toggle")
            self.printspeeds = -self.printspeeds

        if ord(key) == 55: #control 7
            self.movementSpeedCmd = "U"
            print("increase mvspd")
        if ord(key) == 57: #control 9
            self.movementSpeedCmd = "D"
            print("decrease mvspd")
        if ord(key) == 56: # control 8
            self.movementSpeedCmd = "S"
            print("stop mvspd")

        if ord(key) == 52: #control 4
            self.rotationSpeedCmd = "U"
            print("increase rspd")
        if ord(key) == 54: #control 6
            self.rotationSpeedCmd = "D"
            print("decrease rspd")
        if ord(key) == 53: #control 5
            self.rotationSpeedCmd = "S"
            print("decrease rspd")
        
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
            dict_entry = {'node':i, 
            'directions': directions, 
            'force num': force_num,
            'value': force_value,
            'components': force_components,
            'norm': numpy.linalg.norm( numpy.array(force_components))}
            return dict_entry
                
    def onAnimateEndEvent(self, params):
        root = self.getContext()

        if(self.printspeeds > 0):
            controller = root.InstrumentCombined.m_ircontroller
            speed = controller.speed.value
            direction = controller.mainDirection.value
            rotation = controller.rotationInstrument.value
            print("speed: " ,speed)
            print("dir: ", direction)
            print("rotation: ", rotation)
            refpos = root.RefStartingPos.ReferencePos
            refpos_position = refpos.position.value
            print("startpos_pose: ", refpos_position)
            
        
        if(self.rotationSpeedCmd != "N"):
            rotationDirection = 1
            if(self.rotationSpeedCmd == "D"):
                rotationDirection = -1
            controller = root.InstrumentCombined.m_ircontroller
            oldspeed = controller.rotationInstrument.value
            controller.rotationInstrument.value = [controller.rotationInstrument.value[0] + self.rotationSpeedAbsIncrease * rotationDirection]
            if(self.rotationSpeedCmd == "S"):
                controller.rotationInstrument.value = [0]

            print("old rspeed: ",oldspeed, "new rspeed:",controller.rotationInstrument.value)
            self.rotationSpeedCmd = "N"
        
        if(self.movementSpeedCmd != "N"):
            speedDirection = 1
            if(self.movementSpeedCmd == "D"):
                speedDirection = -1
            controller = root.InstrumentCombined.m_ircontroller
            oldspeed = controller.speed.value
            controller.speed.value += self.movementSpeedAbsIncrease * speedDirection
            if(self.movementSpeedCmd == "S"):
                controller.speed.value = 0
            print("old speed: ",oldspeed, "new speed:",controller.speed.value)
            self.movementSpeedCmd = "N"
        
        if(self.directionCmd != "N"):
            rigid = RigidDof(root.RefStartingPos.ReferencePos)
            print(dir(rigid))
            positionIncrease = 5
            displacementVector = [0,0,0]
            if(self.directionCmd == "X"):

                displacementVector[0] = positionIncrease
            if(self.directionCmd == "Y"):
                displacementVector[1] = positionIncrease
            if(self.directionCmd == "Z"):
                displacementVector[2] = positionIncrease
            rigid.translate(displacementVector)

            """
            controller = root.InstrumentCombined.m_ircontroller
            directionVersor = controller.mainDirection.value.copy()
            oldDirectionVersor = controller.mainDirection.value.copy()
            if(self.directionCmd == "X"):

                directionVersor[0] = -directionVersor[0]
            if(self.directionCmd == "Y"):
                directionVersor[1] = -directionVersor[1]
            if(self.directionCmd == "Z"):
                directionVersor[2] = -directionVersor[2]
            
            controller.mainDirection.value = directionVersor
            print("old directionVersor: ", oldDirectionVersor, "new directionVersor: ", directionVersor)
            """
       
            self.directionCmd = "N"



        

        if(self.printLog.value == 1):
            self.printLog = 0
            force_directions = root.InstrumentCombined.CollisInstrumentCombined.CollisionDOFs.constraint
            print("-----FORCE DIRECTIONS RAW-----")
            print(force_directions.value)
            print(type(force_directions.value))
            print("------------------------------")

            self.collisionMatrix = force_directions.value.splitlines()
            self.collisionMatrix = [line.split() for line in self.collisionMatrix]
            self.collisionMatrix = [[float(Value) for Value in self.collisionMatrix[i]] for i in range(len(self.collisionMatrix))]
            print("----EXTRACTED LINES FROM FORCE DIRECTIONS----")
            for line in self.collisionMatrix:
                print(line)
            print("---------------------------------------------")



            force_values = root.constsolver.constraintForces.value.tolist()
            print("----FORCE VALUES RAW----")
            print(force_values)
            print(type(force_values))
            print("------------------------")


            self.numberDOFs = 3
            self.numberNodes = catheter_vars['numnodes'] +1

            self.extractedCollisionMatrix = []
            self.collisionDict = [ [] for i in range(self.numberNodes)]

            for i in range(len(self.collisionMatrix)):
                numberElements = int(self.collisionMatrix[i][1])

                if numberElements == 1:
                    element1 = int(self.collisionMatrix[i][2])
                    direction1 = [self.collisionMatrix[i][3+j] for j in range(3)]
                    forcenum = int(self.collisionMatrix[i][0])
                    forceval = force_values[forcenum]
                    forcecomps = [ (direction1[j] * forceval) for j in range(3)]

                    self.collisionDict[element1].append(self.createDictEntry(element1,direction1,forcenum,forceval,forcecomps))


                if numberElements == 2:
                    
                    element1 = int(self.collisionMatrix[i][2])
    

                    element1 = int(self.collisionMatrix[i][2])
                    direction1 = [self.collisionMatrix[i][3+j] for j in range(3)]
                    forcenum = int(self.collisionMatrix[i][0])
                    forceval = force_values[forcenum]
                    forcecomps = [ (direction1[j] * forceval) for j in range(3)]

                    self.collisionDict[element1].append(self.createDictEntry(element1,direction1,forcenum,forceval,forcecomps))
        
                    element2 = int(self.collisionMatrix[i][3 + self.numberDOFs ])
                    direction2 = [self.collisionMatrix[i][4+ self.numberDOFs +j] for j in range(3)]
                    forcenum = int(self.collisionMatrix[i][0])
                    forceval = force_values[forcenum]
                    forcecomps = [ (direction2[j] * forceval) for j in range(3)]
                    
                    self.collisionDict[element2].append(self.createDictEntry(element2,direction2,forcenum,forceval,forcecomps))
                    



                if numberElements == 3:
                    """
                    element1 = int(self.collisionMatrix[i][2])
                    element2 = int(self.collisionMatrix[i][3+ self.numberDOFs ])
                    element3 = int(self.collisionMatrix[i][4 + 2*self.numberDOFs ])
                    self.collisionDict[element1]['dirs'].append([self.collisionMatrix[i][3+j] for j in range(3)])
                    self.collisionDict[element1]['forcenum'].append(int(self.collisionMatrix[i][0]))
                    self.collisionDict[element2]['dirs'].append([self.collisionMatrix[i][4+ self.numberDOFs +j] for j in range(3)] )
                    self.collisionDict[element2]['forcenum'].append(int(self.collisionMatrix[i][0]))
                    self.collisionDict[element3]['dirs'].append([self.collisionMatrix[i][5+ 2*self.numberDOFs +j] for j in range(3)] )
                    self.collisionDict[element3]['forcenum'].append(int(self.collisionMatrix[i][0]))
                    """
                
            print("------EXTRACTED VALUES FINAL------")
            for i in range (len(self.collisionDict)):
                elm = self.collisionDict[i]
                print("---")
                print("node:",i)
                for subelm in elm:
                    print(subelm)
                print("---")
            print("----------------------------------")


        
     



def createScene(rootNode):
    rootNode.findData('dt').value=0.01
    rootNode.findData('gravity').value=gravity

    rootNode.addObject( MyController(name="MyEmptyController") )
    rootNode.addObject('RequiredPlugin', pluginName='BeamAdapter SofaMeshCollision SofaBoundaryCondition SofaConstraint SofaMiscCollision SofaDeformable SofaGeneralLinearSolver SofaImplicitOdeSolver')
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideMappings hideForceFields showInteractionForceFields hideWireframe')
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')

    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', name="constsolver", tolerance="1e-3", maxIterations="5000", unbuilt="false", computeConstraintForces="true")
    rootNode.addObject('DefaultPipeline', depth="6", verbose="0", draw="1")
    rootNode.addObject('BruteForceBroadPhase', name='N2')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', response="FrictionContactConstraint", responseParams="mu=0.65")
    rootNode.addObject('LocalMinDistance', name="Proximity", alarmDistance=catheter_vars['radius']*1.36, contactDistance=catheter_vars['radius'], angleCone="0.01")

    carotidsNode = rootNode.addChild('Carotids')
    carotidsNode.addObject('MeshOBJLoader', name='loader', filename='body2.obj', flipNormals="false")
    carotidsNode.addObject('MeshTopology',src = '@loader')
    carotidsNode.addObject('MechanicalObject', name='dofs', template='Vec3d', dx=object_pose['dx'], dy=object_pose['dy'], dz=object_pose['dz'], rx=object_pose['rx'], ry=object_pose['ry'],rz=object_pose['rz'])
    carotidsNode.addObject('TriangleCollisionModel', group='1')
    carotidsNode.addObject('LineCollisionModel', group='1')
    carotidsNode.addObject('PointCollisionModel', group='1')

    visuCarotidsNode = carotidsNode.addChild('visuCarotids')
    visuCarotidsNode.addObject('OglModel', name="VisualModel", color="1.0 1.0 1.0 1.0")
    visuCarotidsNode.addObject('IdentityMapping', input="@../dofs", output="@VisualModel")

    topoLines_cath = rootNode.addChild('topoLines_cath')
    topoLines_cath.addObject('WireRestShape', template="Rigid3d", printLog=False, name="catheterRestShape", length=catheter_vars['length'], straightLength=catheter_vars['length'], spireDiameter="0", spireHeight="0.0", densityOfBeams="40", numEdges="20", numEdgesCollis="20", youngModulus="2.5", youngModulusExtremity="2.5", radius="@../Proximity.contactDistance")
    topoLines_cath.addObject('EdgeSetTopologyContainer', name="meshLinesCath")
    topoLines_cath.addObject('EdgeSetTopologyModifier', name="Modifier")
    topoLines_cath.addObject('EdgeSetGeometryAlgorithms', name="GeomAlgo", template="Rigid3d")
    topoLines_cath.addObject('MechanicalObject', template="Rigid3d", name="dofTopo1")

    RefStartingPos = rootNode.addChild('RefStartingPos')
    RefStartingPos.addObject('MechanicalObject', name="ReferencePos", template="Rigid3d", dx=catheter_vars["dx"], dy=catheter_vars["dy"], dz=catheter_vars["dz"], rx=catheter_vars["rx"], ry=catheter_vars["ry"], rz=catheter_vars["rz"])


    InstrumentCombined = rootNode.addChild('InstrumentCombined')
    InstrumentCombined.addObject('EulerImplicitSolver', rayleighStiffness="0.01", rayleighMass="0.03", printLog=False )
    InstrumentCombined.addObject('BTDLinearSolver')
    InstrumentCombined.addObject('RegularGridTopology', name="meshLinesCombined", nx=catheter_vars['numnodes'], ny="1", nz="1")

    InstrumentCombined.addObject('MechanicalObject', template="Rigid3d", name="DOFs" )
    InstrumentCombined.addObject('InterventionalRadiologyController', template="Rigid3d", name="m_ircontroller", printLog=False, xtip=controller_vars['xtip'],speed =controller_vars['speed'],   step=controller_vars['step'], rotationInstrument="0", controlledInstrument="0", startingPos="@../RefStartingPos/ReferencePos.position", instruments="InterpolCatheter")
    InstrumentCombined.addObject('WireBeamInterpolation', name="InterpolCatheter", WireRestShape="@../topoLines_cath/catheterRestShape", radius="3.0", printLog=False)
    InstrumentCombined.addObject('AdaptiveBeamForceFieldAndMass', name="CatheterForceField", massDensity="0.000000920", interpolation="@InterpolCatheter", printLog=False)
    InstrumentCombined.addObject('LinearSolverConstraintCorrection', printLog=False, wire_optimization="true")
    InstrumentCombined.addObject("FixedConstraint", indices="0")
    InstrumentCombined.addObject('RestShapeSpringsForceField', name="MeasurementFF", points="@m_ircontroller.indexFirstNode",  stiffness="1e10", recompute_indices="1", angularStiffness="1e10", external_rest_shape="@../RefStartingPos/ReferencePos", external_points="0", drawSpring="1", springColor="1 0 0 1")

    CollisInstrumentCombined = InstrumentCombined.addChild('CollisInstrumentCombined')
    CollisInstrumentCombined.addObject('EdgeSetTopologyContainer', name="collisEdgeSet")
    CollisInstrumentCombined.addObject('EdgeSetTopologyModifier', name="colliseEdgeModifier")
    CollisInstrumentCombined.addObject('MechanicalObject', name="CollisionDOFs")
    CollisInstrumentCombined.addObject('MultiAdaptiveBeamMapping', name="multimapp", ircontroller="../m_ircontroller", useCurvAbs="1", printLog="false")
    CollisInstrumentCombined.addObject('LineCollisionModel' )
    CollisInstrumentCombined.addObject('PointCollisionModel')

    visuInstrumentCombined = InstrumentCombined.addChild('visuInstrumentCombined')
    visuInstrumentCombined.addObject('MechanicalObject', name="Quads")
    visuInstrumentCombined.addObject('QuadSetTopologyContainer', name="ContainerCath")
    visuInstrumentCombined.addObject('QuadSetTopologyModifier', name="Modifier" )
    visuInstrumentCombined.addObject('QuadSetGeometryAlgorithms', name="GeomAlgo", template="Vec3d")
    visuInstrumentCombined.addObject('Edge2QuadTopologicalMapping', nbPointsOnEachCircle="10", radius="@../../Proximity.contactDistance", input="@../../topoLines_cath/meshLinesCath", output="@ContainerCath", flipNormals="true",printLog=False)
    visuInstrumentCombined.addObject('AdaptiveBeamMapping', name="VisuMapCath", useCurvAbs="1", printLog=False, isMechanical="false",  interpolation="@../InterpolCatheter")


    realVisuInstrumentCombined = visuInstrumentCombined.addChild('realVisuInstrumentCombined')
    realVisuInstrumentCombined.addObject('OglModel',name="VisualCathOGL", src="@../ContainerCath", color='white')
    realVisuInstrumentCombined.addObject('IdentityMapping', input="@../Quads", output="@VisualCathOGL")




    
    

def main():


    root = Sofa.Core.Node('root')

    createScene(root)

    Sofa.Simulation.init(root)


    Sofa.Gui.GUIManager.Init('myscene', 'qglviewer')
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 1080)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()
