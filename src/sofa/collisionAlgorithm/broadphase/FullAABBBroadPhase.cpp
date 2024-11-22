#include <sofa/collisionAlgorithm/broadphase/FullAABBBroadPhase.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

int FullAABBBroadPhaseClass = core::RegisterObject("FullAABBBroadPhase")
.add< FullAABBBroadPhase >();

}
