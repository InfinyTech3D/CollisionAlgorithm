#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm
{
void registerPointGeometry(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData(
            "A class bridging point topological information with the proximity detection algorithm")
            .add<PointGeometry<sofa::defaulttype::Vec3dTypes> >());
}
}  // namespace sofa::collisionAlgorithm
