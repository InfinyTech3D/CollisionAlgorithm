from re import I
import Sofa

g_needleLength=0.200 #(m)
g_needleNumberOfElems=20 #(# of edges)
g_needleBaseOffset=[0.04,0.15,-0.2]
g_needleRadius = 0.001 #(m)
g_needleMechanicalParameters = {
    "radius":g_needleRadius,
    "youngModulus":4e12,
    "poissonRatio":0.3
}
g_needleTotalMass=0.04

g_gelRegularGridParameters = [
    {
        "n":[6, 4, 4],
        "min":[-0.150, -0.050, -0.250],
        "max":[0.150, 0.0099, -0.100]
    },
    {
        "n":[6, 4, 4],
        "min":[-0.150, 0.0101, -0.250],
        "max":[0.150, 0.060, -0.100]
    },
    {
        "n":[6, 4, 4],
        "min":[-0.150, -0.080, -0.250],
        "max":[0.150, -0.0501, -0.100]
    }
] #Again all in mm
g_gelMechanicalParameters = {
    "youngModulus":8e5,
    "poissonRatio":0.45,
    "method":"large"
}
g_gelTotalMass = 1
g_cubeColor=[[0.8, 0.34, 0.34, 0.3],[0.6, 0.6, 0, 0.3],[1, 1, 1, 0.3]]
g_wireColor=[[0.8, 0.34, 0.34, 1],[0.6, 0.6, 0, 1],[1, 1, 1, 1]]
g_gelFixedBoxROI=[[-0.155, -0.055, -0.255, -0.145, 0.065, -0.095 ], [0.155, -0.055, -0.255, 0.145, 0.065, -0.095 ]]

# Function called when the scene graph is being created
def createScene(root):
    root.gravity=[0,0,0]
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
                                                'ConstraintGeometry'
                                                ])


    root.addObject("ConstraintAttachButtonSetting")
    root.addObject("VisualStyle", displayFlags="showVisualModels hideBehaviorModels showCollisionModels hideMappings hideForceFields showWireframe showInteractionForceFields")
    root.addObject("FreeMotionAnimationLoop")
    root.addObject("GenericConstraintSolver", tolerance=1e-5, maxIt=5000, regularizationTerm=1e-5)
    root.addObject("CollisionLoop")

    needleBaseMaster = root.addChild("NeedleBaseMaster")
    needleBaseMaster.addObject("MechanicalObject", name="mstate_baseMaster", position=[0.04,0.15,-0.2, 0, 0, 0, 1], template="Rigid3d", showObjectScale=0.002, showObject=False, drawMode=1)
    needleBaseMaster.addObject("LinearMovementProjectiveConstraint",indices=[0], keyTimes=[0,1,7,8],movements=
        [ [0.04, 0.15, 0.2, 0, 0, 0]
        , [0.04, 0.30, -0.2, -3.14/2, 0, 0]
        , [0.04, 0.145, -0.2, -3.14/2, 0, 0]
        , [0.03, 0.145, -0.2, -3.14/2 + 3.14/16, 0, 0]
    ],relativeMovements=False)


    needle = root.addChild("Needle")
    needle.addObject("EulerImplicitSolver", firstOrder=True)
    needle.addObject("EigenSparseLU", name="LinearSolver", template="CompressedRowSparseMatrixd")
    needle.addObject("EdgeSetTopologyContainer", name="Container"
        , position=[[g_needleBaseOffset[0], g_needleBaseOffset[1], -(i * g_needleLength/(g_needleNumberOfElems) + g_needleBaseOffset[2])] for i in range(g_needleNumberOfElems + 1)]
        , edges=[[i, i+1] for i in range(g_needleNumberOfElems)]
    )
    needle.addObject("EdgeSetTopologyModifier", name="modifier")
    needle.addObject("PointSetTopologyModifier", name="modifier2")
    needle.addObject("MechanicalObject", name="mstate", template="Rigid3d", showObjectScale=0.0002, showObject=False, drawMode=1)
    needle.addObject("UniformMass", totalMass=g_needleTotalMass)
    needle.addObject("BeamFEMForceField", name="FEM", **g_needleMechanicalParameters)
    needle.addObject("LinearSolverConstraintCorrection", printLog=False, linearSolver="@LinearSolver")

    needleBase = needle.addChild("needleBase")
    needleBase.addObject("PointSetTopologyContainer", name="Container_base", position=[0, 0, 0])
    needleBase.addObject("MechanicalObject",name="mstate_base", template="Rigid3d",)
    needleBase.addObject("RestShapeSpringsForceField",points=[0],stiffness=1e8, angularStiffness=1e8,external_points=[0],external_rest_shape="@/NeedleBaseMaster/mstate_baseMaster")
    needleBase.addObject("SubsetMapping", indices="0")

    needleBodyCollision = needle.addChild("bodyCollision")
    needleBodyCollision.addObject("EdgeSetTopologyContainer", name="Container_body", src="@../Container")
    needleBodyCollision.addObject("MechanicalObject",name="mstate_body", template="Vec3d",)
    needleBodyCollision.addObject("EdgeGeometry",name="geom_body",mstate="@mstate_body", topology="@Container_body")
    needleBodyCollision.addObject("EdgeNormalHandler", name="NeedleBeams", geometry="@geom_body")
    needleBodyCollision.addObject("IdentityMapping")

    needleTipCollision = needle.addChild("tipCollision")
    needleTipCollision.addObject("PointSetTopologyContainer", name="Container_tip"
        , position=[g_needleBaseOffset[0], g_needleBaseOffset[1], -(g_needleLength+g_needleBaseOffset[2])])
    needleTipCollision.addObject("MechanicalObject",name="mstate_tip")#,position=[g_needleLength+g_needleBaseOffset[0], g_needleBaseOffset[1], g_needleBaseOffset[2]],template="Vec3d",)
    needleTipCollision.addObject("PointGeometry",name="geom_tip",mstate="@mstate_tip")
    needleTipCollision.addObject("RigidMapping",globalToLocalCoords=True,index=g_needleNumberOfElems)

    needleVisual = needle.addChild("visual")
    needleVisual.addObject("QuadSetTopologyContainer", name="Container_visu")
    needleVisual.addObject("QuadSetTopologyModifier", name="Modifier")
    needleVisual.addObject("Edge2QuadTopologicalMapping", nbPointsOnEachCircle=8, radius=g_needleRadius, input="@../Container", output="@Container_visu")
    needleVisual.addObject("MechanicalObject", name="mstate_visu", showObjectScale=0.0002, showObject=True, drawMode=1)
    needleVisual.addObject("TubularMapping", nbPointsOnEachCircle=8, radius=g_needleRadius, input="@../mstate", output="@mstate_visu")

    needleOGL = needleVisual.addChild("OGL")
    needleOGL.addObject("OglModel", position="@../Container_visu.position",
                           vertices="@../Container_visu.position",
                           quads="@../Container_visu.quads",
                           color=[0.4, 0.34, 0.34],
                           material="texture Ambient 1 0.4 0.34 0.34 1.0 Diffuse 0 0.4 0.34 0.34 1.0 Specular 1 0.4 0.34 0.34 0.1 Emissive 1 0.5 0.54 0.54 .01 Shininess 1 20",
                           name="visualOgl")
    needleOGL.addObject("IdentityMapping")



    for i in range(0,3):
        gelGridTopoName = "GelGridTopo" + str(i)
        gelTopo = root.addChild(gelGridTopoName)
        gelTopo.addObject("RegularGridTopology", name="HexaTop", **g_gelRegularGridParameters[i])

        volume = root.addChild("Layer"+str(i))
        if(i < 2):
            volume.addObject("EulerImplicitSolver")
            volume.addObject("EigenSimplicialLDLT", name="LinearSolver", template='CompressedRowSparseMatrixMat3x3d')
        volume.addObject("TetrahedronSetTopologyContainer", name="TetraContainer", position="@../"+gelGridTopoName+"/HexaTop.position")
        volume.addObject("TetrahedronSetTopologyModifier", name="TetraModifier")
        volume.addObject("Hexa2TetraTopologicalMapping", input="@../"+gelGridTopoName+"/HexaTop", output="@TetraContainer", swapping=False)
        volume.addObject("MechanicalObject", name="mstate_gel", template="Vec3d")
        volume.addObject("TetrahedronGeometry", name="geom_tetra", mstate="@mstate_gel", topology="@TetraContainer", draw=False)
        volume.addObject("AABBBroadPhase",name="AABBTetra",geometry="@geom_tetra",nbox=[3,3,3],thread=1)
        volume.addObject("TetrahedronFEMForceField", name="FF",**g_gelMechanicalParameters)
        volume.addObject("MeshMatrixMass", name="Mass",totalMass=g_gelTotalMass)
        volume.addObject("BoxROI",name="BoxROI",box=g_gelFixedBoxROI)
        volume.addObject("RestShapeSpringsForceField", stiffness=1e6, points="@BoxROI.indices"  )
        volume.addObject("FixedLagrangianConstraint", indices="@BoxROI.indices"  )
        if(i < 2):
            volume.addObject("LinearSolverConstraintCorrection", printLog=False, linearSolver="@LinearSolver")
    
        volumeCollision = volume.addChild("collision")
        volumeCollision.addObject("TriangleSetTopologyContainer", name="TriContainer")
        volumeCollision.addObject("TriangleSetTopologyModifier", name="TriModifier")
        volumeCollision.addObject("Tetra2TriangleTopologicalMapping", name="mapping", input="@../TetraContainer", output="@TriContainer", flipNormals=False)
        volumeCollision.addObject("MechanicalObject", name="mstate_gelColi",position="@../TetraContainer.position")
        volumeCollision.addObject("TriangleGeometry", name="geom_tri", mstate="@mstate_gelColi", topology="@TriContainer",draw=False)
        volumeCollision.addObject("PhongTriangleNormalHandler", name="SurfaceTriangles", geometry="@geom_tri")
        volumeCollision.addObject("AABBBroadPhase",name="AABBTriangles",thread=1,nbox=[2,2,3])
        volumeCollision.addObject("IdentityMapping", name="identityMappingToCollision", input="@../mstate_gel", output="@mstate_gelColi", isMechanical=True)
    
        volumeVisu = volumeCollision.addChild("visu")
        volumeVisu.addObject("OglModel", position="@../TriContainer.position",
                            vertices="@../TriContainer.position",
                            triangles="@../TriContainer.triangles",
                            color=g_cubeColor[i],name="volume_visu",template="Vec3d")
        volumeVisu.addObject("IdentityMapping")

        volumeVisuWire = volume.addChild("visu_wire")
        volumeVisuWire.addObject("VisualStyle", displayFlags="showWireframe")
        volumeVisuWire.addObject("OglModel", position="@../TetraContainer.position",
                            vertices="@../TetraContainer.position",
                            triangles="@../TetraContainer.triangles",
                            color=g_wireColor[i],name="volume_visu",template="Vec3d")
        volumeVisuWire.addObject("IdentityMapping")

    root.addObject("NearestPointROI", template="Vec3d", name="RedYellow", radius=0.0025,
                   object1="@Layer0/mstate_gel", object2="@Layer1/mstate_gel")
    root.addObject("BilateralLagrangianConstraint", name="RedYellowAttachment", 
                   first_point="@RedYellow.indices1", second_point="@RedYellow.indices2",
                   object1="@Layer0/mstate_gel", object2="@Layer1/mstate_gel")

    root.addObject("NearestPointROI", template="Vec3d", name="RedWhite", radius=0.0025,
                   object1="@Layer0/mstate_gel", object2="@Layer2/mstate_gel")
    root.addObject("BilateralLagrangianConstraint", name="RedWhiteAttachment", 
                   first_point="@RedWhite.indices1", second_point="@RedWhite.indices2",
                   object1="@Layer0/mstate_gel", object2="@Layer2/mstate_gel")


    for i in range(0,3):
        algo = root.addChild("algo"+str(i))
        punctureForce = 1.5 if i < 2 else 2000
        algo.addObject("InsertionAlgorithm", name="InsertionAlgo"+str(i), 
            tipGeom="@/Needle/tipCollision/geom_tip", 
            surfGeom="@/Layer"+str(i)+"/collision/geom_tri", 
            shaftGeom="@/Needle/bodyCollision/geom_body", 
            volGeom="@/Layer"+str(i)+"/geom_tetra", 
            punctureForceThreshold=punctureForce,
            tipDistThreshold=0.009,
            drawcollision=True,
            drawPointsScale=0.0001
        )
        algo.addObject("DistanceFilter", name="DistanceFilter"+str(i), algo="@InsertionAlgo"+str(i), distance=0.01)
        algo.addObject("SecondDirection", name="punctureDirection"+str(i), handler="@../Layer"+str(i)+"/collision/SurfaceTriangles")
        algo.addObject("ConstraintUnilateral", name="cs_Uni"+str(i), input="@InsertionAlgo"+str(i)+".collisionOutput", directions="@punctureDirection"+str(i), draw_scale=0.001, mu=0.1)
        if (i < 2):
            algo.addObject("FirstDirection", name="bindDirection"+str(i), handler="@../Needle/bodyCollision/NeedleBeams")
            algo.addObject("ConstraintInsertion", name="cs_Ins"+str(i), input="@InsertionAlgo"+str(i)+".insertionOutput", directions="@bindDirection"+str(i), draw_scale=0.002, frictionCoeff=0.00)
