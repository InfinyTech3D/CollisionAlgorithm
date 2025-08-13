#include <sofa/collisionAlgorithm/algorithm/Find2DClosestProximityAlgorithm.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm
{
void registerFind2DClosestProximityAlgorithm(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData(
            "An algorithm to find the closest proximity between two BaseGeometry types in 2D")
            .add<Find2DClosestProximityAlgorithm>());
}
}  // namespace sofa::collisionAlgorithm
