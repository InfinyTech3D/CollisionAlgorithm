#include <CollisionAlgorithm/config.h>
#include <CollisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionalgorithm
{

template class SOFA_COLLISIONALGORITHM_API TriangleGeometry<sofa::defaulttype::Vec3dTypes>;

void registerTriangleGeometry(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData("A class bridging triangle topological information with "
                                           "the proximity detection algorithm")
            .add<TriangleGeometry<sofa::defaulttype::Vec3dTypes> >());
}
}  // namespace sofa::collisionalgorithm
