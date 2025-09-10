#include <CollisionAlgorithm/filters/DistanceFilter.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm
{
void registerDistanceFilter(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData(
            "This class filters detected proximities based on their distance from source")
            .add<DistanceFilter>());
}
}  // namespace sofa::collisionAlgorithm
