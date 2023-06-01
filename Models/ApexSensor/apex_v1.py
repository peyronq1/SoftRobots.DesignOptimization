import Sofa
import math
import csv
import numpy as np
import serial
from elastic_material_object import ElasticMaterialObject
from fixing_box import FixingBox
from stlib3.components import addOrientedBoxRoi
from stlib3.physics.mixedmaterial import Rigidify
# from actuators.pneumatic import PneumaticCavity
from splib3.constants import Key
from splib3.numerics import vec3

# from stlib3.physics.mixedmaterial import Rigidify

class Apex(Sofa.Prefab):
    prefabParameters = [
        {"name": "name", "type": "string", "help": "Node name", "default": "Apex"},
        {"name": "YoungsModulus", "type": "float", "help": "Young modulus of the Apex material", "default": 2.0},
        {"name": "PoissonRation", "type": "float", "help": "Poisson ratio of the Apex material",
         "default": 0.45}]
        # {"name": "useInverse", "type":"boolean", "help": "Add components for inverse model", "default":False}
    # ]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    # Construct the actuated finger
    def init(self):

        #------------------------- Define the setup Fixed frame
        fixed_frame=self.addChild("FixedFrame")
        fixed_frame.addObject('MechanicalObject', name="dofs", template="Rigid3", position=[0, 0, 0.0, 0, 0, 0, 1],
                             showObject=True, showObjectScale=5)
        c = fixed_frame.addChild("Constraint")
        c.addObject('RestShapeSpringsForceField',
                       points=fixed_frame.dofs,
                       stiffness=1e9)

        #------------------------- Load the apex mesh and create an elastic body from it
        self.elasticMaterial = self.elasticBody()
        self.ElasticBody.init()

        #------------------------- Add a fixing box to constrain the 3mm top portion of the apex
        FixingBox(self, self.elasticMaterial, translation=[0.0, 0.0, 0.0], scale=[40.0, 40.0, 6.0])
        self.FixingBox.BoxROI.drawBoxes = True

        #------------------------- Construct the force sensor cavity
        # Load the cavity mesh and define it as a pressure cavity
        outer_cavity = self.addChild("OuterCavity")
        outer_cavity.addObject('MeshSTLLoader', name='MeshLoader', filename="Meshes/apex_cavite_ext.stl",
                               rotation=[0.0, 0.0, 0.0],
                               translation=[-12.0, -12.0, 33.5])
        outer_cavity.addObject('TriangleSetTopologyContainer', src='@MeshLoader', name='container')
        # outer_cavity.topology.drawEdges = True
        outer_cavity.addObject('MechanicalObject', template='Vec3', name="dofs")
        outer_cavity.addObject('SurfacePressureConstraint',
                               value=0.0,
                               valueType='pressure')
        outer_cavity.dofs.drawMode = 1
        outer_cavity.dofs.showObject = True
        outer_cavity.dofs.showColor = [1, 0, 0, 1]
        outer_cavity.init()

        # Link the mesh bottom part to the fixed frame and the upper part to the deformable apex
        groupIndices = []
        frames = []
        boxL = addOrientedBoxRoi(self, position=[list(i) for i in outer_cavity.dofs.rest_position.value],
                                name="BoxROIL",
                                translation=[0.0,0.0,-20.0],
                                eulerRotation=[0.0,0.0,0.0], scale=[50, 50, 6])
        boxL.drawBoxes = True
        boxL.init()
        groupIndices.append([ind for ind in boxL.indices.value])
        frames.append([0.0,0.0,-20.0,0.0,0.0,0.0,1.0])
        rigidifiedstruct = Rigidify(self,outer_cavity,groupIndices = groupIndices,frames = frames,name = "RigidifiedStruct")
        rigidParts = self.RigidifiedStruct.RigidParts
        defParts = self.RigidifiedStruct.DeformableParts
        defParts.addObject('BarycentricMapping',name="mapping1",input=self.elasticMaterial.dofs.getLinkPath(),output=defParts.dofs.getLinkPath())
        rigidParts.addObject('RigidMapping', name="mapping2", input=fixed_frame.dofs.getLinkPath(),
                       output=rigidParts.RigidifiedParticules.dofs.getLinkPath())

        # Vizualization of the outer cavity mesh
        vizu = outer_cavity.addChild("VisualOuterCav")
        vizu.addObject("MeshSTLLoader", name="outerCavMeshLoader", filename="Meshes/apex_cavite_ext.stl",
                                       rotation=[0.0, 0.0, 0.0],
                                       translation=[-12.0, -12.0, 33.5])
        vizu.addObject("OglModel", name="rendererOuterCav", src="@outerCavMeshLoader",
                                       color=[0.0, 1.0, 0.0, 0.5])
        vizu.addObject("BarycentricMapping", input=outer_cavity.dofs.getLinkPath(),
                                       output=vizu.rendererOuterCav.getLinkPath())

        #------------------------- Define the localized force applied by the probe on the apex
        probe = self.elasticMaterial.addChild("Probe")
        boxF = addOrientedBoxRoi(probe, position=[list(i) for i in self.elasticMaterial.dofs.rest_position.value],
                                 name="BoxROIF",
                                 translation=[3.0, -3.0, -12.0],
                                 eulerRotation=[0.0, 0.0, 0.0], scale=[3, 3, 10])
        boxF.drawBoxes = True
        boxF.init()
        # probe.addObject("ConstantForceField",mstate = self.elasticMaterial.dofs.getLinkPath(),indices=boxF.indices.value,totalForce=[0.0,0.0,-0.000],showArrowSize=300,showColor=[1.0, 0.0, 1.0, 1.0])
        # groupIndices.append([ind for ind in boxL.indices.value])


        #------------------------- Construct the inner apex cavity to simulate blood pressure
        # I guess the pressure is in MPa since we use mm as distance units ?
        inner_cavity = self.elasticMaterial.addChild("InnerCavity")
        inner_cavity.addObject('MeshSTLLoader', name='MeshLoader', filename="Meshes/apex_cavite_int.stl", rotation= [0.0,0.0,0.0],
                            translation=[-12.0, -12.0, 33.5])
        inner_cavity.addObject('MeshTopology', name='topology', src='@MeshLoader')
        inner_cavity.addObject('MechanicalObject', src="@topology")
        inner_cavity.addObject('SurfacePressureConstraint',
                            value=0.0000,
                            valueType='pressure')
        inner_cavity.addObject('BarycentricMapping', name="Mapping", input=self.elasticMaterial.dofs.getLinkPath(), output= inner_cavity.MechanicalObject.getLinkPath(), mapForces=False, mapMasses=False)

        # Vizualization of the inner cavity mesh
        vizu = inner_cavity.addChild("VisualInnerCav")
        vizu.addObject("MeshSTLLoader", name="innerCavMeshLoader", filename="Meshes/apex_cavite_int.stl", rotation=[0.0, 0.0, 0.0],
                         translation=[-12.0, -12.0, 33.5])
        vizu.addObject("OglModel", name="rendererInnerCav", src="@innerCavMeshLoader", color=[1.0, 0.0, 0.0, 0.5])
        vizu.addObject("BarycentricMapping", input=inner_cavity.MechanicalObject.getLinkPath(), output=vizu.rendererInnerCav.getLinkPath())


    def elasticBody(self):
        # Create a body as a child of the parent (the actuated finger)
        body = self.addChild("ElasticBody")

        # Create an ElasticMaterialObject, which import a mesh, assign it dofs  and mechanical properties
        # All the properties are expressed in SI units. The dimensions in the generated mesh are in meter,
        # the young modulus in Pascal ...
        e = body.addChild(ElasticMaterialObject(
            volumeMeshFileName="Meshes/apex_deformable.msh",
            topoMesh="tetrahedron",
            scale=[1, 1, 1],
            totalMass=0.012,
            youngModulus=self.YoungsModulus.value, # in MPa
            poissonRatio=self.PoissonRation.value,
            rotation=[0.0, 0.0, 0.0],
            translation=[-12.0,-12.0,33.5],
            withConstrain=False,
            solverName = "SparseLDL"))

        # Add now a visual model to the flexible part
        visual = body.addChild("VisualApex")
        # Load the STL file for the visualization, the rotation and translation must
        # fit the one for the ElasticMaterialObject
        visual.addObject("MeshSTLLoader", name="visualLoader", filename="Meshes/apex.stl", rotation=[0.0, 0.0, 0.0],
                         translation=[-12.0,-12.0,33.5])
        visual.addObject("OglModel", name="renderer", src="@visualLoader", color=[1.0, 1.0, 1.0, 0.5])

        # Link the dofs of the 3D mesh and the visual model
        visual.addObject("BarycentricMapping", input=e.dofs.getLinkPath(), output=visual.renderer.getLinkPath())

        return e

    def addInverseComponents(self):
        actuators = []
        # Define the probe force as the input of the direct model. It becomes the output in the inverse model
        probe = self.elasticMaterial.Probe
        forceProbeInd = probe.BoxROIF.indices.value
        N = len(forceProbeInd)
        print(str(self.elasticMaterial.dofs.rest_position[forceProbeInd]))

        probe.addObject('VisualStyle', displayFlags="showInteractionForceFields")
        forceAct= probe.addObject('MechanicalObject', name="MO", position=self.elasticMaterial.dofs.rest_position[forceProbeInd])
        probe.addObject('ForcePointActuator', template='Vec3', showForce=True, visuScale=10,
                             direction=[0,0,-1], indices=list(range(N)), maxForce=1.0, minForce=-1.0)
        probe.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)
        probe.activated = True
        # probe.init()
        actuators.append(forceAct)

        # Define the volume of the outer cavity as the effector (output) in the direct model. It becomes the input in the inverse model
        outer_cavity = self.OuterCavity
        vol_init = outer_cavity.SurfacePressureConstraint.initialCavityVolume.value
        volEff = outer_cavity.addObject('VolumeEffector', template='Vec3', triangles='@topo.triangles', desiredVolume=vol_init)
        volEff.flipNormal = True
        outer_cavity.activated = True
        # outer_cavity.init()
        actuators.append(volEff)

        return actuators


class ApexController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.node = kwargs["node"]
        self.duration = 0.2
        self.time = 0.0
        self.timePrec = 0.0
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


    def onAnimateBeginEvent(self, eventType):

        # Read the file of experimental data, and interpolate on a regular grid for the applied force
        forceExp = [0.2,0.4,0.6]
        pressureExp = [0.0,0.0,0.0]
        dispZExp = [0.0,0.0,0.0]
        nPt = len(forceExp)

        # Perform the simulation
        stepFsimu = 0.010 # in N
        stepFexp = 0.2
        maxF = -0.5
        self.force = self.force-stepFsimu
        if abs(self.force) >= abs(self.idForce*(stepFexp+1)):

            if self.idForce<nPt:
                # Get Z displacement
                apexNodesPos = self.elasticBody.dofs.position.value
                dispZ = []
                nbNodes = self.elasticBody.dofs.size.value
                for w in range(0, nbNodes):
                    disp = vec3.vsub(apexNodesPos[w], self.apexNodesInitPos[w])
                    dispZ.append(abs(disp[2]))
                maxDispZ = max(dispZ)

                self.costFD += pow(maxDispZ - dispZExp[self.idForce],2)

                # Get pressure
                volume = self.outerCav.SurfacePressureConstraint.cavityVolume.value
                volumeInit = self.outerCav.SurfacePressureConstraint.initialCavityVolume.value
                volumeTube = 2*np.pi*(1e-3)*2
                pressure = (volume+volumeTube)/(volumeInit+volumeTube)*0.1 #in MPa

                self.costFP += pow(pressure-pressureExp[self.idForce],2)
            else:
                self.costFD = np.sqrt(self.costFD)
                self.costFP = np.sqrt(self.costFP)

        th = 0
        phi = 0
        force = [self.force * math.cos(th) * math.sin(phi), self.force * math.sin(th) * math.sin(phi),
                 self.force * math.cos(phi)]
        self.probeForce.ConstantForceField.totalForce = force
        self.innerCav.SurfacePressureConstraint.value[0] = self.innerPressure


