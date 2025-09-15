#include <CollisionAlgorithm/broadphase/FullAABBBroadPhase.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionalgorithm
{
void registerFullAABBBroadPhase(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData("").add<FullAABBBroadPhase>());
}
}  // namespace sofa::collisionalgorithm
