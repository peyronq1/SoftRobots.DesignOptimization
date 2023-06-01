# -*- coding: utf-8 -*-
"""Config for the SensorFinger"""

__authors__ = "sescaidanavarro, tnavez"
__contact__ = "stefan.escaida@uoh.cl, tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Oct 28 2022"

import math
import numpy as np

from BaseFitnessEvaluationController import BaseFitnessEvaluationController
from stlib3.scene import Scene
from splib3.numerics import vec3
from apex_v1 import Apex

# from Generation import Cavity, Finger

# from MoldGeneration import MoldBox, MoldLid, MoldForCork, FingerClamp

class FitnessEvaluationController(BaseFitnessEvaluationController):
    
    def __init__(self, *args, **kwargs):

        print('>>> Start Init SOFA scene ...')

        super(FitnessEvaluationController,self).__init__(*args, **kwargs)

        # self.ModelNode = self.rootNode.model
        # self.CableConstraint = self.ModelNode.cables.cable1.CableConstraint
        # self.ReferenceMO = self.rootNode.ReferenceMONode.ReferenceMO
        # self.StartPosition = np.array(self.ReferenceMO.position.value[0])
        # self.StartAngle = math.acos( np.abs(self.StartPosition[2]) / np.linalg.norm(self.StartPosition))
        # self.FollowingMO = self.rootNode.model.FollowingMONode.FollowingMO
        #
        # # Cavities
        # self.SurfacePressureConstraint1 = self.ModelNode.Cavity01.SurfacePressureConstraint
        # self.SurfacePressureConstraint2 = self.ModelNode.Cavity02.SurfacePressureConstraint
        #


        self.rootnode = kwargs["rootNode"]

        self.elasticBody = kwargs["elasticBody"]
        self.outerCav = kwargs["outerCav"]
        self.innerCav = kwargs["innerCav"]
        self.probeForce = kwargs["probeForce"]

        self.force = 0.00
        self.innerPressure = 0.00
        self.idForce = 0
        self.costFP = 0
        self.costFD = 0
        # self.innerPressure = -0.0033
        self.volumeOuterCav = 0.0
        self.idPhi = 0
        self.idTh = 0
        self.apexNodesInitPos = self.elasticBody.dofs.rest_position.value

        # Objective evaluation variables
        self.current_iter = 0
        current_objectives = self.config.get_currently_assessed_objectives()
        self.max_iter = max([self.config.get_objective_data()[current_objectives[i]][1] for i in range(len(current_objectives))])
        
        print('>>> ... End')
        

    def onAnimateBeginEvent(self, dt):

        # Perform the simulation
        stepFsimu = 0.010  # in N
        maxFsimu = 0.6
        stepFexp = 0.1

        # Read the file of experimental data, and interpolate on a regular grid for the applied force
        # for phi = 0, theta = 15
        forceExp = [-0.1,-0.2,-0.3,-0.4,-0.5]
        pressureExp = [0.0249, 0.0464, 0.0676, 0.0898, 0.1120]
        dispZExp = [0.31, 0.5490, 0.76, 0.9617, 1.1584]
        nPt = len(forceExp)

        if abs(self.force)<maxFsimu:
            self.force = self.force - stepFsimu

        if self.idForce<nPt:
            if abs(self.force) >= abs(forceExp[self.idForce]):

                # Get Z displacement
                apexNodesPos = self.elasticBody.dofs.position.value
                dispZ = []
                nbNodes = self.elasticBody.dofs.size.value
                for w in range(0, nbNodes):
                    disp = vec3.vsub(apexNodesPos[w], self.apexNodesInitPos[w])
                    dispZ.append(abs(disp[2]))
                maxDispZ = max(dispZ)

                self.costFD += pow(maxDispZ - dispZExp[self.idForce], 2)

                # Get pressure
                volume = self.outerCav.SurfacePressureConstraint.cavityVolume.value
                volumeInit = self.outerCav.SurfacePressureConstraint.initialCavityVolume.value
                # volumeTube = np.pi * pow(0.5,2) * 2000.0 # pneumatic tubing of 0.5mm radius and 2m long
                volumeTube = 0
                deltaPressure = (volumeInit + volumeTube) / (volume + volumeTube) * 100 - 100  # in kPa
                # print("pressure: " + str(deltaPressure))
                self.costFP += pow(deltaPressure - pressureExp[self.idForce], 2)

                if self.idForce == nPt-1:
                    self.costFD = np.sqrt(self.costFD)
                    self.costFP = np.sqrt(self.costFP)

                self.idForce += 1

        th = 15.0*np.pi/180
        phi = 0
        force = [self.force * math.cos(th) * math.sin(phi), self.force * math.sin(th) * math.sin(phi),
                 self.force * math.cos(phi)]
        self.probeForce.ConstantForceField.totalForce = force
        self.innerCav.SurfacePressureConstraint.value[0] = self.innerPressure

        self.current_iter += 1
        
        if self.current_iter == self.max_iter:            
            
            current_objectives_name = self.config.get_currently_assessed_objectives()

            for i in range(len(current_objectives_name)):

                current_objective_name =  current_objectives_name[i]

                # Sensibility metrics. Reflects the efficiency of a pressure sensor.
                if "ErrorForceDisp" == current_objective_name:
                    print("Displacement quadratic error: ", self.costFD)
                    self.objectives.append(self.costFD)
                    
                # Sensibility metrics to volume variation. Reflects the efficiency of a volume sensor.
                if "ErrorForcePressure" == current_objective_name:
                    print("Pressure quadratic error: ", self.costFP)
                    self.objectives.append(self.costFP)
                

def createScene(rootNode, config):
    
    ###############################
    ### Import required plugins ###
    ###############################
    rootNode.addObject("RequiredPlugin", name="SoftRobots")
    rootNode.addObject("RequiredPlugin", name="SofaSparseSolver")
    rootNode.addObject("RequiredPlugin", name="SofaPreconditioner")
    rootNode.addObject("RequiredPlugin", name="SofaPython3")
    rootNode.addObject('RequiredPlugin', name='SofaOpenglVisual')
    rootNode.addObject('RequiredPlugin', name="SofaMiscCollision")
    rootNode.addObject("RequiredPlugin", name="SofaBoundaryCondition")
    rootNode.addObject("RequiredPlugin", name="SofaConstraint")
    rootNode.addObject("RequiredPlugin", name="SofaEngine")
    rootNode.addObject('RequiredPlugin', name='SofaImplicitOdeSolver')
    rootNode.addObject('RequiredPlugin', name='SofaLoader')
    rootNode.addObject('RequiredPlugin', name="SofaSimpleFem")
    rootNode.addObject('RequiredPlugin', name="SofaDeformable")
    rootNode.addObject('RequiredPlugin', name="SofaGeneralLoader")

    ##############################
    ### Visualization settings ###
    ##############################
    rootNode.addObject('LightManager')
    rootNode.addObject('PositionalLight', name="light1", color="0.8 0.8 0.8", position="0 60 50")                
    rootNode.addObject('PositionalLight', name="light2", color="0.8 0.8 0.8", position="0 -60 -50") 
    rootNode.addObject('VisualStyle', displayFlags='hideWireframe showBehaviorModels hideCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields')

    ###########################
    ### Simulation settings ###
    ###########################
    # Define the main architecture of the scene, with a node Modelling, Setting and Simulation
    # Define also the integration method as Euler implicit and the solver as Conjugate Gradient)
    scene = Scene(rootNode, gravity=[0.0, 0.0, 0.0],
                  plugins=[],
                  iterative=False)
    # scene.addMainHeader()
    # scene.addSettings()
    scene.addModelling()
    scene.addSimulation()
    scene.addObject('FreeMotionAnimationLoop')
    # Inverse solver
    # scene.addObject('QPInverseProblemSolver', name='QP', printLog=False)
    scene.addObject('GenericConstraintSolver',maxIterations=1000,tolerance=0.001)
    # ContactHeader(scene, alarmDistance=15e-3, contactDistance=0.5e-3, frictionCoef=0.1)

    # Setting the time step
    rootNode.dt = 0.02

    # Constrain correction
    scene.Simulation.addObject('GenericConstraintCorrection')
    # scene.Settings.mouseButton.stiffness = 1e-2

    ##################
    ### Load model ###
    ##################
    # Create one actuated finger
    apex = Apex(YoungsModulus = config.YoungsModulus, PoissonRation = config.PoissonRation)
    scene.Modelling.addChild(apex)
    rootNode.addObject(FitnessEvaluationController(name="FitnessEvaluationController",
                                                   rootNode=rootNode,
                                                   elasticBody = apex.elasticMaterial,
                                                   innerCav = apex.elasticMaterial.InnerCavity,
                                                   outerCav = apex.OuterCavity,
                                                   probeForce= apex.elasticMaterial.Probe,
                                                   config=config))

    scene.Simulation.TimeIntegrationSchema.firstOrder = True
    scene.Simulation.addChild(apex.elasticMaterial)
    scene.Simulation.addChild(apex.elasticMaterial.InnerCavity)
    scene.Simulation.addChild(apex.OuterCavity)
    scene.Simulation.addChild(apex.elasticMaterial.Constraint)
    
    return rootNode

    
    
