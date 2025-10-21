import Sofa

GeomagicActive = False

g_needleLength=0.200 #(m)
g_needleNumberOfElems=40 #(# of edges)
g_needleBaseOffset=[0.15,0.04,0.04]
g_needleRadius = 0.001 #(m)
g_needleMechanicalParameters = {
    "radius":g_needleRadius,
    "youngModulus":2e13,
    "poissonRatio":0.45
}
g_needleTotalMass=0.01

g_gelRegularGridParameters = {
    "n":[8, 8, 8],
    "min":[-0.125, -0.125, -0.350],
    "max":[0.125, 0.125, -0.100]
} #Again all in mm
g_gelMechanicalParameters = {
    "youngModulus":4e4,
    "poissonRatio":0.3,
    "method":"large"
}
g_gelTotalMass = 1
g_cubeColor=[0.8, 0.34, 0.34, 0.3]
g_gelFixedBoxROI=[-0.350, -0.280, -0.360, 0.130, 0.130, -0.200 ]

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
                                                'ConstraintGeometry',
                                                ('Geomagic' if GeomagicActive else ''),
                                                'Sofa.Component.Haptics',
                                                'Sofa.Component.IO.Mesh',
                                                'Sofa.Component.Playback'
                                                ])


    root.addObject("ConstraintAttachButtonSetting")
    root.addObject("VisualStyle", displayFlags="showVisualModels hideBehaviorModels showCollisionModels hideMappings hideForceFields showWireframe showInteractionForceFields" )
    root.addObject("FreeMotionAnimationLoop")
    root.addObject("CollisionPipeline", name='pipeline', depth='6', verbose='0')
    root.addObject("BruteForceBroadPhase")
    root.addObject("BVHNarrowPhase")
    root.addObject("CollisionResponse", name='response', response='FrictionContactConstraint')
    root.addObject("LocalMinDistance", name='proximity', alarmDistance='0.1', contactDistance='0.02')
    root.addObject("ProjectedGaussSeidelConstraintSolver", tolerance=0.00001, maxIt=5000)#, regularizationTerm=0.001)
    root.addObject("CollisionLoop")

    toolController = root.addChild("ToolController")
    controllerPos = ""
    if (GeomagicActive):
        controllerPos = "@GeomagicDevice.positionDevice"
        toolController.addObject("GeomagicDriver"
            , name='GeomagicDevice' 
            , deviceName='Default Device' 
            , scale=0.02 
            , drawDeviceFrame=False 
            , drawDevice=False 
            , manualStart=False
            , positionBase=[0.12, 0, 0] 
            , orientationBase=[0, 0.174, 0, -0.985] 
        )
        #toolController.addObject("WriteState", name="writer", filename="RecordState/NeedleInsertionHaptics.txt"
        #    , period=0.01, writeX=True, writeV=True, time=0)
    else:
        controllerPos = "@reader.position"

    toolController.addObject("MechanicalObject", name="mstate_baseMaster"
        , position=controllerPos
        , template="Rigid3d"
        , showObjectScale=0.01
        , showObject=False
        , drawMode=1
    )
    if (not GeomagicActive):
        toolController.addObject("ReadState", name="reader", filename="RecordState/NeedleInsertionHaptics.txt")

    needle = root.addChild("Needle")
    needle.addObject("EulerImplicitSolver", firstOrder=True)
    needle.addObject("EigenSparseLU", name="LinearSolver", template="CompressedRowSparseMatrixd")
    needle.addObject("EdgeSetTopologyContainer", name="Container"
        , position=[[g_needleBaseOffset[0], g_needleBaseOffset[1], -(i * g_needleLength/(g_needleNumberOfElems) + g_needleBaseOffset[2])] for i in range(g_needleNumberOfElems + 1)]
        , edges=[[i, i+1] for i in range(g_needleNumberOfElems)]
    )
    needle.addObject("EdgeSetTopologyModifier", name="modifier")
    needle.addObject("PointSetTopologyModifier", name="modifier2")
    needle.addObject("MechanicalObject", name="mstate", template="Rigid3d"
        , showObjectScale=0.002, showObject=False, drawMode=1)

    needle.addObject("UniformMass", totalMass=g_needleTotalMass)
    needle.addObject("BeamFEMForceField", name="FEM", **g_needleMechanicalParameters)
    needle.addObject("LinearSolverConstraintCorrection", linearSolver="@LinearSolver")
    needle.addObject("RestShapeSpringsForceField",points=[0],stiffness=1e9, angularStiffness=1e11,external_points=[0],external_rest_shape="@/ToolController/mstate_baseMaster")

    needleBase = needle.addChild("needleBase")
    needleBase.addObject("PointSetTopologyContainer", name="Container_base", position="@../mstate.position")
    needleBase.addObject("MechanicalObject",name="mstate_base", template="Rigid3d")
    needleBase.addObject("SubsetMapping", indices=0)

    needleBodyCollision = needle.addChild("bodyCollision")
    needleBodyCollision.addObject("EdgeSetTopologyContainer", name="Container_body", src="@../Container")
    needleBodyCollision.addObject("MechanicalObject",name="mstate_body", template="Vec3d", drawMode=0, showObject=False, showObjectScale=10)
    needleBodyCollision.addObject("EdgeGeometry",name="geom_body",mstate="@mstate_body", topology="@Container_body")
    needleBodyCollision.addObject("EdgeNormalHandler", name="NeedleBeams", geometry="@geom_body")
    #needleBodyCollision.addObject("LineCollisionModel", name="volLineColi")
    #needleBodyCollision.addObject("PointCollisionModel", name="volPtColi")
    needleBodyCollision.addObject("IdentityMapping")

    needleTipCollision = needle.addChild("tipCollision")
    needleTipCollision.addObject("PointSetTopologyContainer", name="Container_tip"
        , position=[g_needleBaseOffset[0], g_needleBaseOffset[1], -(g_needleLength+g_needleBaseOffset[2])])
    needleTipCollision.addObject("MechanicalObject",name="mstate_tip",template="Vec3d", showObject=False, showObjectScale=20)
    needleTipCollision.addObject("PointGeometry",name="geom_tip",mstate="@mstate_tip")
    needleTipCollision.addObject("RigidMapping",globalToLocalCoords=True,index=g_needleNumberOfElems)


    needleVisual = needle.addChild("visual")
    needleVisual.addObject("QuadSetTopologyContainer", name="Container_visu")
    needleVisual.addObject("QuadSetTopologyModifier", name="Modifier")
    needleVisual.addObject("Edge2QuadTopologicalMapping", nbPointsOnEachCircle=8, radius=g_needleRadius, input="@../Container", output="@Container_visu")
    needleVisual.addObject("MechanicalObject", name="mstate_visu", showObjectScale=0.0002, showObject=False, drawMode=1)
    needleVisual.addObject("TubularMapping", nbPointsOnEachCircle=8, radius=g_needleRadius, input="@../mstate", output="@mstate_visu")

    needleOGL = needleVisual.addChild("OGL")
    needleOGL.addObject("OglModel", position="@../Container_visu.position",
                           vertices="@../Container_visu.position",
                           quads="@../Container_visu.quads",
                           color=[0.4, 0.34, 0.34],
                           material="texture Ambient 1 0.4 0.34 0.34 1.0 Diffuse 0 0.4 0.34 0.34 1.0 Specular 1 0.4 0.34 0.34 0.1 Emissive 1 0.5 0.54 0.54 .01 Shininess 1 20",
                           name="visualOgl")
    needleOGL.addObject("IdentityMapping")

    FF = root.addChild("ForceFeedback")
    FF.addObject("MechanicalObject", name="mstate_lcp", template="Rigid3d"
        , showObject=False, src="@../Needle/needleBase/mstate_base")
    FF.addObject("LCPForceFeedback", name="lcp_ff", activate=1, forceCoef=1)
    FFCollision = FF.addChild("Collision")
    FFCollision.addObject("EdgeSetTopologyContainer", name="Container", src="@../../Needle/bodyCollision/Container_body")
    FFCollision.addObject("MechanicalObject", name="mstate_coli", constraint="@../../Needle/bodyCollision/mstate_body.constraint")
    FFCollision.addObject("RigidMapping", globalToLocalCoords=True)
    #FFTip = FF.addChild("Tip")
    #FFTip.addObject("PointSetTopologyContainer", name="Container", src="@../../Needle/tipCollision/Container_tip")
    #FFTip.addObject("MechanicalObject", name="mstate_coli", constraint="@../../Needle/tipCollision/mstate_tip.constraint")
    #FFTip.addObject("RigidMapping", globalToLocalCoords=True)

    volume = root.addChild("Volume")
    volume.addObject("EulerImplicitSolver")
    volume.addObject("EigenSimplicialLDLT", name="LinearSolver", template='CompressedRowSparseMatrixMat3x3d')
    volume.addObject("MeshGmshLoader", name="meshLoader", filename="mesh/liver.msh", scale3d=[0.08, 0.08, 0.08], translation=[0, -0.3, -0.2])
    volume.addObject("TetrahedronSetTopologyContainer", name="TetraContainer", position="@meshLoader.position", tetrahedra="@meshLoader.tetrahedra")
    volume.addObject("TetrahedronSetTopologyModifier", name="TetraModifier")

    volume.addObject("MechanicalObject", name="mstate_gel", template="Vec3d")
    volume.addObject("TetrahedronGeometry", name="geom_tetra", mstate="@mstate_gel", topology="@TetraContainer", draw=False)
    volume.addObject("PhongTriangleNormalHandler", name="InternalTriangles", geometry="@geom_tetra")
    volume.addObject("FastTetrahedralCorotationalForceField", name="FF",**g_gelMechanicalParameters)
    volume.addObject("MeshMatrixMass", name="Mass",totalMass=g_gelTotalMass)

    volume.addObject("BoxROI",name="BoxROI",box=g_gelFixedBoxROI)
    volume.addObject("RestShapeSpringsForceField", stiffness=1e3, angularStiffness=1e3, points="@BoxROI.indices"  )

    volume.addObject("LinearSolverConstraintCorrection", printLog=False, linearSolver="@LinearSolver")

    volumeCollision = volume.addChild("collision")
    volumeCollision.addObject("TriangleSetTopologyContainer", name="TriContainer")
    volumeCollision.addObject("TriangleSetTopologyModifier", name="TriModifier")
    volumeCollision.addObject("Tetra2TriangleTopologicalMapping", name="mapping", input="@../TetraContainer", output="@TriContainer", flipNormals=False)
    volumeCollision.addObject("MechanicalObject", name="mstate_gelColi",position="@../TetraContainer.position")
    volumeCollision.addObject("TriangleGeometry", name="geom_tri", mstate="@mstate_gelColi", topology="@TriContainer",draw=False)
    volumeCollision.addObject("PhongTriangleNormalHandler", name="SurfaceTriangles", geometry="@geom_tri")
    volumeCollision.addObject("AABBBroadPhase", name="AABBTriangles", thread=1, nbox=[2,2,3], method=2)
    #volumeCollision.addObject("TriangleCollisionModel", name="volTriColi")
    #volumeCollision.addObject("LineCollisionModel", name="volLineColi")
    #volumeCollision.addObject("PointCollisionModel", name="volPtColi")

    volumeCollision.addObject("IdentityMapping", name="identityMappingToCollision", input="@../mstate_gel", output="@mstate_gelColi", isMechanical=True)

    volumeVisu = volumeCollision.addChild("visu")
    volumeVisu.addObject("OglModel", position="@../TriContainer.position",
                        vertices="@../TriContainer.position",
                        triangles="@../TriContainer.triangles",
                        color=g_cubeColor,name="volume_visu",template="Vec3d")
    volumeVisu.addObject("IdentityMapping")

    volumeVisuWire = volume.addChild("visu_wire")
    volumeVisuWire.addObject("OglModel", position="@../TetraContainer.position",
                        vertices="@../TetraContainer.position",
                        triangles="@../TetraContainer.triangles",
                        color=[1, 0, 1, 1],name="volume_visu",template="Vec3d")
    volumeVisuWire.addObject("IdentityMapping")


    root.addObject("InsertionAlgorithm", name="InsertionAlgo", 
        tipGeom="@Needle/tipCollision/geom_tip", 
        surfGeom="@Volume/collision/geom_tri", 
        shaftGeom="@Needle/bodyCollision/geom_body", 
        volGeom="@Volume/geom_tetra", 
        punctureForceThreshold=1., 
        tipDistThreshold=0.02,
        drawcollision=True,
        drawPointsScale=0.0001
    )
    root.addObject("DistanceFilter",algo="@InsertionAlgo",distance=0.02)
    root.addObject("SecondDirection",name="punctureDirection",handler="@Volume/collision/SurfaceTriangles")
    root.addObject("ConstraintUnilateral",input="@InsertionAlgo.collisionOutput",directions="@punctureDirection",draw_scale=0.001, mu=0.001)
    root.addObject("FirstDirection",name="bindDirection", handler="@Needle/bodyCollision/NeedleBeams")
    root.addObject("ConstraintInsertion",
        input="@InsertionAlgo.insertionOutput", 
        directions="@bindDirection",
        draw_scale="0.01", 
        frictionCoeff=0.01,
        slipForce=0.4,
        popForce=0.10
    )
