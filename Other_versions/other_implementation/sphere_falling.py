import sys, os
import Sofa.Core
import SofaRuntime
import numpy
from stlib3.physics.rigid import Cube, Sphere, Floor, RigidObject

sphere_vars = {'trasl':[0.0, 50.0, 0.0],'rot':[0.0, 0.0, 0.0], 'numnodes':1}

dt = 0.0001

def createScene(rootNode):
    #rootNode.addObject("OglGrid", nbSubdiv=10, size=1000)

    rootNode.findData('gravity').value=[0.0,-9806,0.0]
    rootNode.findData('dt').value=dt


    #Collision function
    rootNode.addObject( MyController(name="MyEmptyController") )
    rootNode.addObject('RequiredPlugin', name="SofaMiscCollision", printLog=False)
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideMappings hideForceFields showInteractionForceFields hideWireframe')

    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', name="constsolver", tolerance="1e-3", maxIterations="5000", unbuilt="false", computeConstraintForces="true")
    rootNode.addObject('DefaultPipeline', depth="6", verbose="0", draw="1")
    rootNode.addObject('BruteForceBroadPhase', name='N2')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', response="FrictionContactConstraint", responseParams="mu=0.65")
    rootNode.addObject('LocalMinDistance', name="Proximity",alarmDistance=10, contactDistance=1, angleCone=0.01)


    #Creating the floor
    floor = rootNode.addChild("floor")

    floor.addObject('MechanicalObject', name="mstate", template="Rigid3d")

    floorCollis = floor.addChild('collision')
    floorCollis.addObject('MeshObjLoader', name="loader", filename="mesh/floor.obj",
            triangulate="true")
    floorCollis.addObject('MeshTopology', src="@loader")
    floorCollis.addObject('MechanicalObject')
    floorCollis.addObject('TriangleCollisionModel', moving=False, simulated=False)
    floorCollis.addObject('LineCollisionModel', moving=False, simulated=False)
    floorCollis.addObject('PointCollisionModel', moving=False, simulated=False)
    floorCollis.addObject('RigidMapping')


    Sphere = rootNode.addChild("mySphere")

    inputMsh = "mesh/ball.obj"

    Sphere.addObject('MechanicalObject',
                      name="mState", template="Rigid3",
                      translation2=[0,50,0])

    Sphere.addObject('UniformMass', name="mass", totalMass=2.0)
    Sphere.addObject('EulerImplicitSolver')
    Sphere.addObject('CGLinearSolver')
    
    Sphere.addObject("UncoupledConstraintCorrection")

    objectCollis = Sphere.addChild('collision')
    objectCollis.addObject('MeshObjLoader', name="loader",
                        filename=inputMsh, triangulate=True,
                        scale=1)

    objectCollis.addObject('MeshTopology', src="@loader")
    objectCollis.addObject('MechanicalObject', name="collisionObj")


    objectCollis.addObject('TriangleCollisionModel')
    objectCollis.addObject('LineCollisionModel')
    objectCollis.addObject('PointCollisionModel')
    objectCollis.addObject('RigidMapping')



class MyController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, *kwargs)
        self.collisionMatrix = []
        self.sortedCollisionMatrix = []
        self.printLog.value = 0

    def onKeypressedEvent(self, event):
        key = event['key']
        if ord(key) == 77:  # control punto
            self.printLog.value = 1
            print("press step to print next collision data")
    


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
        if(self.printLog.value == 1):
            self.printLog = 0
            force_directions = root.mySphere.mState.constraint
            #force_directions = root.mySphere.collision.collisionObj.constraint
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
            print("----FORCE VALUES RAW ----")
            print(force_values)
            print(type(force_values))
            print("------------------------")
            print("The final force in Newton is: ", (force_values[0] * 0.001)/  dt) # TRASFORMO IN NEWTON E DIVIDO PER DT



def main():


    root = Sofa.Core.Node('root')

    createScene(root)

    Sofa.Simulation.init(root)


    Sofa.Gui.GUIManager.Init('myscene', 'qglviewer')
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 1080)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()



def onEndAnimationStep(self, dt):
        print(self.root.BeamModel.CollisionModel.CollisionDOFs.constraint)
        # Prints matrix H which should represent the direction of the collision
        # source: https://www.sofa-framework.org/community/forum/topic/pneumatic-actuator-soft-robots-plugin/
        #fatForce = np.linalg.norm(self.rootNode.getChild('Fat').getObject('mechObject').force)
        #muscleForce = np.linalg.norm(self.rootNode.getChild('Muscle').getObject('mechObject').force)
        #liverForce = np.linalg.norm(self.rootNode.getChild('liver').getObject('mechObject').force)

        #self.fatForce.append(fatForce)
        #self.muscleForce.append(muscleForce)
        #self.liverForce.append(liverForce)

        #self.iteration.append(self.iteration[-1]+0.005)
        #self.ax.plot(self.iteration, self.fatForce, 'r-')
        #self.ax.plot(self.iteration, self.muscleForce, 'b-')
        #self.ax.plot(self.iteration, self.liverForce, 'g-')
        #plt.draw()
        #plt.pause(0.0001)
        return 0


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()