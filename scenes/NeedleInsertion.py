import Sofa

g_needleLength=0.100 #(m)
g_needleNumberOfElems=20 #(# of edges)
g_needleBaseOffset=[0.04,0.04,0]
g_needleRadius = 0.001 #(m)
g_needleMechanicalParameters = {
    "radius":g_needleRadius,
    "youngModulus":1e11,
    "poissonRatio":0.3
}
g_needleTotalMass=0.01

g_gelRegularGridParameters = {
    "n":[8, 8, 8],
    "min":[-0.125, -0.125, -0.350],
    "max":[0.125, 0.125, -0.100]
} #Again all in mm
g_gelMechanicalParameters = {
    "youngModulus":8000,
    "poissonRatio":0.45,
    "method":"large"
}
g_gelTotalMass = 1
g_cubeColor=[0.8, 0.34, 0.34, 0.3]
g_gelFixedBoxROI=[-0.130, -0.130, -0.360, 0.130, 0.130, -0.300 ]

# Function called when the scene graph is being created
def createScene(root):
    root.gravity=[0,0,-9.81]
    root.dt = 0.01

    root.addObject("RequiredPlugin",pluginName=['Sofa.Component.AnimationLoop',
                                                'Sofa.Component.Constraint.Lagrangian.Solver',
                                                'Sofa.Component.ODESolver.Backward',
                                                'Sofa.Component.Visual',
                                                'Sofa.Component.Constraint.Lagrangian.Correction',
                                                'Sofa.Component.Constraint.Lagrangian.Model',
                                                'Sofa.Component.LinearSolver.Direct',
                                                'Sofa.Component.Mapping.Linear',
                                                'Sofa.Component.Mass',
                                                'Sofa.Component.SolidMechanics.FEM.Elastic',
                                                'Sofa.Component.StateContainer',
                                                'Sofa.Component.Topology.Container.Dynamic',
                                                'Sofa.Component.Topology.Mapping',
                                                'Sofa.Component.Mapping.NonLinear',
                                                'Sofa.Component.Topology.Container.Grid',
                                                'Sofa.Component.Constraint.Projective',
                                                'Sofa.Component.SolidMechanics.Spring',
                                                'Sofa.GL.Component.Rendering3D',
                                                'Sofa.GUI.Component',
                                                'Sofa.Component.Engine.Select',
                                                'MultiThreading',
                                                'CollisionAlgorithm',
                                                'ConstraintGeometry'])


    root.addObject("ConstraintAttachButtonSetting")
    root.addObject("VisualStyle", displayFlags="showVisualModels hideBehaviorModels showCollisionModels hideMappings hideForceFields hideWireframe showInteractionForceFields" )
    root.addObject("FreeMotionAnimationLoop")
    root.addObject("GenericConstraintSolver", tolerance=0.01, maxIt=5000, printLog=False)
    root.addObject("CollisionLoop")

    needleBaseMaster = root.addChild("NeedleBaseMaster")
    needleBaseMaster.addObject("MechanicalObject", name="mstate", position=[0.04, 0.04, 0, 0, 0, 0, 1], template="Rigid3d", showObjectScale=0.002, showObject="true", drawMode=1)
    needleBaseMaster.addObject("LinearMovementProjectiveConstraint",indices=[0], keyTimes=[0,1,7,9],movements=[[0.04, 0.04,0,0,0,0],[0.04, 0.04,0.05,0,3.14/2,0],[0.04, 0.04,-0.07,0,3.14/2,0],[0.05, 0.04,-0.07,0,3.14/2 + 3.14/16,0]],relativeMovements=False)



    needle = root.addChild("Needle")
    needle.addObject("EulerImplicitSolver", firstOrder=True)
    needle.addObject("EigenSparseLU", name="LinearSolver", template="CompressedRowSparseMatrixd")
    needle.addObject("EdgeSetTopologyContainer", name="Container", position=[[i * g_needleLength/(g_needleNumberOfElems) + g_needleBaseOffset[0], g_needleBaseOffset[1],  g_needleBaseOffset[2]] for i in range(g_needleNumberOfElems + 1)]
                                                                 , edges=[[i, i+1] for i in range(g_needleNumberOfElems)])

    needle.addObject("EdgeSetTopologyModifier", name="modifier")
    needle.addObject("PointSetTopologyModifier", name="modifier2")

    needle.addObject("MechanicalObject", name="mstate", template="Rigid3d", showObjectScale=0.0002, showObject="true", drawMode=1)

    needle.addObject("UniformMass", totalMass=g_needleTotalMass)
    needle.addObject("BeamFEMForceField", name="FEM", **g_needleMechanicalParameters)
    # needle.addObject("FixedLagrangianConstraint", indices="0"  )
    needle.addObject("LinearSolverConstraintCorrection", printLog="false", linearSolver="@LinearSolver")

    needleBase = needle.addChild("needleBase")
    needleBase.addObject("PointSetTopologyContainer", name="Container", position=[0, 0, 0])
    needleBase.addObject("MechanicalObject",name="mstate", template="Rigid3d",)
    needleBase.addObject("RestShapeSpringsForceField",points=[0],stiffness=1e8, angularStiffness=1e8,external_points=[0],external_rest_shape="@/NeedleBaseMaster/mstate")

    needleBase.addObject("SubsetMapping", indices="0")

    needleBodyCollision = needle.addChild("bodyCollision")
    needleBodyCollision.addObject("EdgeSetTopologyContainer", name="Container", src="@../Container")
    needleBodyCollision.addObject("MechanicalObject",name="mstate", template="Vec3d",)
    needleBodyCollision.addObject("EdgeGeometry",name="geom",mstate="@mstate", topology="@Container")

    needleBodyCollision.addObject("IdentityMapping")


    needleTipCollision = needle.addChild("tipCollision")
    needleTipCollision.addObject("MechanicalObject",name="mstate",position=[g_needleLength+g_needleBaseOffset[0], g_needleBaseOffset[1], g_needleBaseOffset[2]],template="Vec3d",)
    needleTipCollision.addObject("PointGeometry",name="geom",mstate="@mstate")
    needleTipCollision.addObject("RigidMapping",globalToLocalCoords=True)


    needleVisual = needle.addChild("visual")
    needleVisual.addObject("QuadSetTopologyContainer", name="ContainerCath")
    needleVisual.addObject("QuadSetTopologyModifier", name="Modifier")
    needleVisual.addObject("Edge2QuadTopologicalMapping", nbPointsOnEachCircle=8, radius=g_needleRadius, input="@../Container", output="@ContainerCath")

    needleVisual.addObject("MechanicalObject", name="VisualCatheter", showObjectScale="0.0002", showObject="true", drawMode="1")

    needleVisual.addObject("TubularMapping", nbPointsOnEachCircle=8, radius=g_needleRadius, input="@../mstate", output="@VisualCatheter")

    needleOGL = needleVisual.addChild("OGL")
    needleOGL.addObject("OglModel", position="@../ContainerCath.position",
                           vertices="@../ContainerCath.position",
                           quads="@../ContainerCath.quads",
                           color="0.4 0.34 0.34",
                           material="texture Ambient 1 0.4 0.34 0.34 1.0 Diffuse 0 0.4 0.34 0.34 1.0 Specular 1 0.4 0.34 0.34 0.1 Emissive 1 0.5 0.54 0.54 .01 Shininess 1 20",
                           name="VisualCatheter")
    needleOGL.addObject("IdentityMapping")



    gelTopo = root.addChild("GelGridTopo")
    gelTopo.addObject("RegularGridTopology", name="HexaTop", **g_gelRegularGridParameters)


    volume = root.addChild("Volume")
    volume.addObject("EulerImplicitSolver")
    volume.addObject("EigenSimplicialLDLT", name="LinearSolver", template='CompressedRowSparseMatrixMat3x3d')
    volume.addObject("TetrahedronSetTopologyContainer", name="Container", position="@../GelGridTopo/HexaTop.position")
    volume.addObject("TetrahedronSetTopologyModifier", name="Modifier")
    volume.addObject("Hexa2TetraTopologicalMapping", input="@../GelGridTopo/HexaTop", output="@Container", swapping="false")

    volume.addObject("MechanicalObject", name="mstate", template="Vec3d")
    volume.addObject("TetrahedronGeometry", name="geom",draw=False)
    volume.addObject("AABBBroadPhase",name="AABBTetra",geometry="@geom",nbox=[3,3,3],thread=1)
    volume.addObject("ParallelTetrahedronFEMForceField", name="FF",**g_gelMechanicalParameters)
    volume.addObject("MeshMatrixMass", name="Mass",totalMass=g_gelTotalMass)

    volume.addObject("BoxROI",name="BoxROI",box=g_gelFixedBoxROI)
    volume.addObject("RestShapeSpringsForceField", stiffness='1e6',points="@BoxROI.indices"  )

    volume.addObject("LinearSolverConstraintCorrection", printLog="false", linearSolver="@LinearSolver")

    volumeCollision = volume.addChild("collision")
    volumeCollision.addObject("TriangleSetTopologyContainer", name="TriContainer")
    volumeCollision.addObject("TriangleSetTopologyModifier", name="Modifier")
    volumeCollision.addObject("Tetra2TriangleTopologicalMapping", name="mapping", input="@../Container", output="@TriContainer", flipNormals=False)
    volumeCollision.addObject("MechanicalObject", name="mstate",position="@../Container.position")
    volumeCollision.addObject("TriangleGeometry", name="geom", mstate="@mstate", topology="@TriContainer",draw=False)
    volumeCollision.addObject("PhongTriangleNormalHandler", name="SurfaceTriangles", geometry="@geom")
    volumeCollision.addObject("AABBBroadPhase",name="AABBTriangles",thread=1,nbox=[2,2,3])

    volumeCollision.addObject("IdentityMapping", name="identityMappingToCollision", input="@../mstate", output="@mstate", isMechanical=True)

    volumeVisu = volumeCollision.addChild("visu")
    volumeVisu.addObject("OglModel", position="@../TriContainer.position",
                        vertices="@../TriContainer.position",
                        triangles="@../TriContainer.triangles",
                        color=g_cubeColor,name="VisualCatheter",template="Vec3d")
    volumeVisu.addObject("IdentityMapping")


    root.addObject("FindClosestProximityAlgorithm",name="PunctureAlgo",fromGeom="@Needle/tipCollision/geom", destGeom="@Volume/collision/geom")
    root.addObject("DistanceFilter",algo="@PunctureAlgo",distance=0.01)
    root.addObject("SecondDirection",name="punctureDirection",handler="@Volume/collision/SurfaceTriangles")
    root.addObject("ConstraintUnilateral",input="@PunctureAlgo.output",directions="@punctureDirection",draw_scale="0.001")


    root.addObject("FindClosestProximityAlgorithm",name="InsertionAlgo",fromGeom="@Needle/bodyCollision/geom", destGeom="@Volume/geom")
    root.addObject("DistanceFilter",algo="@InsertionAlgo",distance=0.005)

