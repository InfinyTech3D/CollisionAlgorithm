#include <sofa/collisionAlgorithm/CollisionPipeline.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

int CollisionLoopClass = core::RegisterObject("CollisionLoop")
.add< CollisionLoop >();

}
