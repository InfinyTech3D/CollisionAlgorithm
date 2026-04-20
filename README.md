# Collision Algorithm for Needle Insertion - SOFA Plugin

[![Support](https://img.shields.io/badge/support-on_GitHub_Discussions-blue.svg)](https://github.com/sofa-framework/sofa/discussions/)
[![Discord](https://img.shields.io/badge/chat-on_Discord-darkred.svg)](https://discord.gg/G63t3a8Ra6)
[![Contact](https://img.shields.io/badge/contact-on_website-orange.svg)](https://infinytech3d.com/)
[![Support us](https://img.shields.io/badge/support_us-on_Github_Sponsor-purple.svg)](https://github.com/sponsors/InfinyTech3D)

## Description
This SOFA plugin (https://github.com/sofa-framework/sofa) provides a customized collision 
pipeline, designed specifically for needle insertion simulations. 

Coupled with SOFA haptic device plugins, the system offers tactile feedback 
for puncture resistance, release and friction during insertion and retraction.

Unity Engine integration is offered via the [`SOFAUnity`](https://github.com/InfinyTech3D/SofaUnity)
plugin by InfinyTech3D for an enhanced simulation experience. Contact us for more information!

## Features

- Proximity detection between the needle and tissue mesh primitives
- Needle insertion is simulated in phases: puncture, insertion, retraction
- Needle-tissue coupling is done using Lagrangian constraints
- Support for haptic feedback such as resistance during puncture and friction during insertion
- Compatible with SOFA-Unity integration for real-time interactive applications

## Installation and Setup

This plugin can be installed by following the official SOFA documentation for building and registering SOFA plugins
https://sofa-framework.github.io/doc/plugins/build-a-plugin-from-sources/


### Build Steps

- Set up the `external_directories` directory
- Clone into `external_directories`:
```
git clone https://github.com/InfinyTech3D/CollisionAlgorithm.git
```
- Register plugin path in the `external_directories` CMakeLists.txt.
```
sofa_add_subdirectory(plugin CollisionAlgorithm CollisionAlgorithm)
```
- Set the `SOFA_EXTERNAL_DIRECTORIES` variable pointing to `external_directories`
- Configure and generate the SOFA solution using CMake
- Compile SOFA solution (the plugin will be compiled as well)

> [!IMPORTANT]
> In order to use it, this plugin must be build alongside the downstream 
[`ConstraintGeometry`](https://github.com/InfinyTech3D/ConstraintGeometry) plugin.

Supported SOFA version: v25.06 and above

## Architecture

- doc:
    - Code documentation
- scenes:
    - Various simple demo scenes
- src/CollisionAlgorithm:
    - SOFA components implementing the insertion algorithm and the supporting collision pipeline
- regression:
    - Reference files for automated non-regression tests based on SOFA's testing framework

## Get Started

- Include the `CollisionAlgorithm` plugin in a scene file.

``` <RequiredPlugin name=`CollisionAlgorithm`/> ```
- Add the `CollisionLoop` component at the root node of your scene.

``` 
<FreeMotionAnimationLoop/>
<ProjectedGaussSeidelConstraintSolver tolerance='<your tolerance>' maxIt='<maximum solver iterations>' />
*<CollisionLoop/>*

<CollisionPipeline/>
<BruteForceBroadPhase/>
<BVHNarrowPhase/>
<CollisionResponse name='response' response='FrictionContactConstraint'/>
<LocalMinDistance name='proximity' alarmDistance='0.2' contactDistance='0.08'/>
``` 
This component manages the needle insertion algorithm. It can work simultaneously with the existing SOFA `CollisionPipeline` 
and can thus be added incrementally in existing SOFA scenes where contacts and collisions are modelled. 

- Create a node to represent the needle and additional nodes for the needle tip and shaft geometries.
This step is more detailed, so refer to the examples in `scenes/NeedleInsertion.xml` for guidance.

- Add an `InsertionAlgorithm` component inside the needle node as shown below.
```
<Node name='needleInsertion'>
    <InsertionAlgorithm name='algorithm'
        tipGeom='@<path to needle tip geometry component>'
        shaftGeom='@<path to needle shaft geometry component>'
        surfGeom='@<path to tissue surface geometry component>'
        volGeom='@<path to tissue volume geometry component>'
        punctureForceThreshold='<float>'
        tipDistThreshold='<float>'
    />
    <DistanceFilter algo='@algorithm' distance='<float>'/>
    <SecondDirection name='punctureDirection' 
        handler='@<path to the tissue surface triangle handler>'
    />
    <ConstraintUnilateral name='punctureConstraint' 
        input='@algoSkin.collisionOutput' 
        directions='@punctureDirection' 
        mu='<float>'
    />
    <FirstDirection name='bindDirection' handler='@<path to the normal handler of the needle beam'/>
    <ConstraintInsertion name='insertionConstraint' 
        input='@algorithm.insertionOutput' 
        directions='@bindDirection' 
        frictionCoeff='<float>' 
    />
</Node>
```

## Acknowledgments
This project builds upon the original repository from 
[ICube Laboratory, University of Strasbourg](https://icube.unistra.fr/en/) 
and extends it with a needle insertion algorithm and additional functionality.
