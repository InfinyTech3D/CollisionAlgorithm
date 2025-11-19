# Collision Algorithm for Needle Insertion - SOFA Plugin

[![Support](https://img.shields.io/badge/support-on_GitHub_Discussions-blue.svg)](https://github.com/sofa-framework/sofa/discussions/)
[![Discord](https://img.shields.io/badge/chat-on_Discord-darkred.svg)](https://discord.gg/G63t3a8Ra6)
[![Contact](https://img.shields.io/badge/contact-on_website-orange.svg)](https://infinytech3d.com/)
[![Support us](https://img.shields.io/badge/support_us-on_Github_Sponsor-purple.svg)](https://github.com/sponsors/InfinyTech3D)

## Description
This SOFA plugin (https://github.com/sofa-framework/sofa) provides a customized collision 
pipeline, designed specifically for needle insertion simulations. 

When used together with SOFA haptic device plugins, the system offers tactile feedback 
for puncture resistance, release and friction during insertion and retraction.

This plugin has also been integrated in Unity via the [`SOFAUnity`](https://github.com/InfinyTech3D/SofaUnity)
plugin by InfinyTech3D for an enhanced simulation experience. Contact us for more information!

## Features

- Proximity detection between the needle and tissue mesh primitives
- Needle simulation phases: puncture, insertion, retraction
- Constraint-based needle simulation during the 3 phases
- Support for haptic feedback such as resistance during puncture and friction during insertion
- Compatible with SOFA-Unity integration for real-time interactive applications

## Installation and Setup

First review the official SOFA documentation for building and registering SOFA plugins
https://sofa-framework.github.io/doc/plugins/build-a-plugin-from-sources/

### Build Steps

- Set up your `external_directories` directory (described in the SOFA documentation link above)
- Clone this repository into your `external_directories` directory:
    - git clone https://github.com/InfinyTech3D/CollisionAlgorithm.git
- Register the path to your local `CollisionAlgorithm` repository in the CMakeLists.txt file located inside your `external_directories` directory
```sofa_add_subdirectory(plugin CollisionAlgorithm CollisionAlgorithm)```
- Set `SOFA_EXTERNAL_DIRECTORIES` variable (preferably using CMake GUI) to point to your `external_directories` directory
- Configure and generate the SOFA solution using CMake
- Compile SOFA solution (the plugin will be compiled as well)

> [!IMPORTANT]
> In order to use the plugin, make sure that you have also built the downstream 
[`ConstraintGeometry`](https://github.com/InfinyTech3D/ConstraintGeometry) plugin.

Supported SOFA version: v25.06 and above

## Architecture

- doc:
    - Documentation and screenshots of the examples
- scenes:
    - Various simple demo scenes
- src/CollisionAlgorithm:
    - source code of the insertion algorithm SOFA component and supporting collision pipeline classes
- regression:
    - Files for automated regression testing in alignment with SOFA's testing framework

## Acknowledgments
This project builds upon the original repository from 
[ICube Laboratory, University of Strasbourg](https://icube.unistra.fr/en/) 
and extends it with a needle insertion algorithm and additional functionality.
