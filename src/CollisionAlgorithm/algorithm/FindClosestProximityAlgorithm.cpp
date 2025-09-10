#include <CollisionAlgorithm/algorithm/FindClosestProximityAlgorithm.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm
{
void registerFindClosestProximityAlgorithm(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData(
            "An algorithm to find the closest proximity between two BaseGeometry types")
            .add<FindClosestProximityAlgorithm>());
}
}  // namespace sofa::collisionAlgorithm
