#include <CollisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm
{
void registerTriangleGeometry(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData("A class bridging triangle topological information with "
                                           "the proximity detection algorithm")
            .add<TriangleGeometry<sofa::defaulttype::Vec3dTypes> >());
}
}  // namespace sofa::collisionAlgorithm
