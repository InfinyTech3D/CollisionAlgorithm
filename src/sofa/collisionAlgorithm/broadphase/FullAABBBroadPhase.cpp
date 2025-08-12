#include <sofa/collisionAlgorithm/broadphase/FullAABBBroadPhase.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm
{
void registerFullAABBBroadPhase(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData("").add<FullAABBBroadPhase>());
}
}  // namespace sofa::collisionAlgorithm
