# Summary: CollisionAlgorithm & ConstraintGeometry Repositories

The documentation here includes two SOFA plugins. Although spread across two distinct repositories, these plugins work together 
to provide a complete **needle insertion simulation model**, forming a two-stage pipeline: detection → constraint resolution.
For completeness, the documentation is centralized in one document. 

---

## CollisionAlgorithm Repository

### Purpose
This plugin drives the needle insertion simulation logic and provides proximity pairs to be consumed by the constraint resolution layer. It does so through nearest-neighbor queries: for each needle element, find the geometrically closest tissue element and return its barycentric parameterization on the live deforming mesh.

### Core Architecture

**Layered Design:**

The plugin is organized in four layers. At the top sits the algorithm layer, which orchestrates detection. 
Beneath it, the geometry layer provides components that attach to a `MechanicalState` in the scene graph, 
decompose its topology into typed elements (points, edges, triangles, tetrahedra), and expose them for spatial queries. 
The element and proximity layers carry the geometric primitives and their barycentric parameterizations that the algorithm layer ultimately queries and stores.

```
Algorithm Layer (InsertionAlgorithm)
     ↓
Geometry Layer (Point/Edge/Triangle/TetrahedronGeometry)
     ↓
Element Layer (PointElement, EdgeElement, TriangleElement, TetrahedronElement)
     ↓
Proximity Layer (EdgeProximity, TriangleProximity, etc.)
```

**Key Class Hierarchies:**

1. **Geometry Hierarchy** - Progressive complexity through inheritance:
   - `PointGeometry` → `EdgeGeometry` → `TriangleGeometry` → `TetrahedronGeometry`
   - Each adds element types from parent
   - `SubsetGeometry` wraps any geometry to expose a filtered index subset

2. **Element Classes** - Geometric primitives with caching:
   - Store precomputed data (normals, areas, barycentric denominators)
   - Use dirty flags for lazy recomputation

3. **Proximity Classes** - Interpolated contact points:
   - `EdgeProximity`, `TriangleProximity`, `TetrahedronProximity`
   - Store barycentric coordinates and provide position/velocity interpolation

4. **Broad-Phase System** - AABB spatial hashing:
   - Partitions space into 8×8×8 grid
   - Supports multithreading and SAT-based intersection tests

5. **Generic Operation Framework** - Type-dispatched operations:
   - `Project`: Find closest point on element
   - `FindClosestProximity`: Spatial search with filtering
   - Factory pattern for runtime type dispatch

### Algorithms

**InsertionAlgorithm** — the main needle insertion algorithm. Each detection step runs one of two mutually exclusive mode sequences depending on whether puncture has occurred (`m_couplingPts` empty):

*Before puncture* (`m_couplingPts` empty):
1. **Puncture Phase**: Finds the closest surface proximity to the needle tip. When the constraint force exceeds `punctureForceThreshold`, records the contact as a coupling point, transitioning to insertion mode.
2. **Shaft Collision Phase**: Finds closest surface proximities along the needle shaft (skipped if puncture just occurred in the same step).

*After puncture* (`m_couplingPts` non-empty):
3. **Insertion Phase**: Adds new shaft↔volume coupling points as the needle tip advances past `tipDistThreshold`.
4. **Reprojection Phase**: Reprojects stored coupling points back onto the current shaft geometry.

**Outputs:**
- `d_collisionOutput` → proximity pairs from the puncture and shaft collision phases
- `d_insertionOutput` → shaft↔tissue coupling pairs produced by reprojection

**Find2DClosestProximityAlgorithm** — finds closest proximities between two geometries using a 2D projection matrix to restrict the search plane.

---

## ConstraintGeometry Repository

### Purpose
Takes collision detection output from the `CollisionAlgorithm` plugin and creates **Lagrangian constraints** prepping them for resolution by the physics solver. 
Handles bilateral, unilateral, and friction-based constraints.

### Core Architecture

**Constraint Pipeline:**
```
Detection Output (from CollisionAlgorithm)
     ↓
TBaseConstraint (processGeometricalData)
     ↓
InternalConstraint (proximity pairs + normals)
     ↓
ConstraintResolution (solver kernels)
     ↓
Force Application (storeLambda)
```

**Key Class Hierarchies:**

1. **Constraint Types:**
   - `ConstraintBilateral` - Equality constraints (position coupling)
   - `ConstraintUnilateral` - Inequality constraints with optional Coulomb friction
   - `ConstraintInsertion` - Specialized insertion physics with friction

2. **Constraint Direction Strategies** - How to compute constraint normals:
   - `BindDirection`: Direct relative position
   - `ContactDirection`: Uses surface normal handler
   - `FirstDirection`/`SecondDirection`: Normal from one proximity
   - `FixedFrameDirection`: Fixed orthogonal frame

3. **Normal Handler Strategies** - Geometry-dependent normal computation:
   - `GouraudTriangleNormalHandler`: Face normals
   - `PhongTriangleNormalHandler`: Smooth interpolated normals
   - `EdgeNormalHandler`: Edge direction
   - `GravityPointNormalHandler`: Radial from center

4. **Constraint Resolution Classes** - Solver kernels:
   - `BilateralConstraintResolution1/2/3` - 1D/2D/3D bilateral
   - `UnilateralConstraintResolution` - 1D contact
   - `UnilateralFrictionResolution` - 3D contact + Coulomb friction
   - `InsertionConstraintResolution` - Specialized insertion solver

---

## How They Work Together

```
┌─────────────────────────────────────────────────────────────────┐
│                    CollisionAlgorithm Plugin                    │
│                                                                 │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐       │
│  │ Geometries   │───▶│ Broad-Phase  │───▶│ InsertionAlg │       │
│  │ (Needle/     │    │ (AABB Grid)  │    │ (Detection)  │       │
│  │  Tissue)     │    └──────────────┘    └──────┬───────┘       │
│  └──────────────┘                               │               │
│                                                 ▼               │
│                              ┌───────────────────────────────┐  │
│                              │ DetectionOutput<Prox1, Prox2> │  │
│                              │ (Pairs of proximity points)   │  │
│                              └───────────────┬───────────────┘  │
└──────────────────────────────────────────────┼──────────────────┘
                                               │
                                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                   ConstraintGeometry Plugin                     │
│                                                                 │
│  ┌──────────────────┐    ┌──────────────────┐                   │
│  │ TBaseConstraint  │───▶│ ConstraintDir    │                   │
│  │ (Bilateral/      │    │ + NormalHandler  │                   │
│  │  Unilateral/     │    └────────┬─────────┘                   │
│  │  Insertion)      │             │                             │
│  └────────┬─────────┘             ▼                             │
│           │              ┌──────────────────┐                   │
│           │              │ ConstraintNormal │                   │
│           │              │ (directions)     │                   │
│           │              └────────┬─────────┘                   │
│           ▼                       ▼                             │
│  ┌───────────────────────────────────────────┐                  │
│  │     InternalConstraint (pairs + normals)  │                  │
│  └────────────────────┬──────────────────────┘                  │
│                       ▼                                         │
│  ┌───────────────────────────────────────────┐                  │
│  │ ConstraintResolution (solver kernels)     │───▶ Forces       │
│  └───────────────────────────────────────────┘                  │
└─────────────────────────────────────────────────────────────────┘
```

### Key Integration Points

1. **Shared Data Type**: `DetectionOutput<FIRST, SECOND>` contains pairs of `BaseProximity` subclasses that both plugins understand

2. **Proximity Abstraction**: Both plugins use the same proximity hierarchy (`EdgeProximity`, `TriangleProximity`, etc.) - defined in CollisionAlgorithm, consumed by ConstraintGeometry

3. **CollisionComponent Interface**: `BaseNormalHandler` in ConstraintGeometry inherits from `CollisionComponent` (from CollisionAlgorithm) to participate in the detection preparation phase

4. **Generic Operation System**: Both plugins use the same `GenericOperation` factory pattern for type-dispatched operations

---

## Summary

- **CollisionAlgorithm** handles the "where" - detecting collision points between needle and tissue through geometric queries and spatial indexing
- **ConstraintGeometry** handles the "how" - converting those detection points into physical constraints with proper force resolution

Together, they enable realistic needle insertion simulations with puncture resistance, friction, and tissue deformation.
