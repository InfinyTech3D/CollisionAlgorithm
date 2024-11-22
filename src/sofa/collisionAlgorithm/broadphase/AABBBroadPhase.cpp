#include <sofa/collisionAlgorithm/broadphase/AABBBroadPhase.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

int AABBBroadPhaseClass = core::RegisterObject("AABBBroadPhase")
.add< AABBBroadPhase >();

}
