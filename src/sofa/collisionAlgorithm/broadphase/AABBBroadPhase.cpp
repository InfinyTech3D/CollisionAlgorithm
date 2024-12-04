#include <sofa/collisionAlgorithm/broadphase/AABBBroadPhase.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

int AABBBroadPhaseClass = core::RegisterObject(R"(
**AABBBroadPhase** is a real-time collision detection component that uses the Axis-Aligned Bounding Box (AABB) approach for broad-phase collision handling.

It organizes elements into a 3D spatial grid, efficiently indexing and retrieving them to identify potential collisions, optimizing the process of narrowing down collision checks for more precise detection in 3D simulations.
)")
.add< AABBBroadPhase >();

}
