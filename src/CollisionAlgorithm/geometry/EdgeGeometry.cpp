#include <CollisionAlgorithm/config.h>
#include <CollisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionalgorithm
{

template class SOFA_COLLISIONALGORITHM_API EdgeGeometry<sofa::defaulttype::Vec3dTypes>;

void registerEdgeGeometry(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData(
            "A class bridging edge topological information with the proximity detection algorithm")
            .add<EdgeGeometry<sofa::defaulttype::Vec3dTypes> >());
}
}  // namespace sofa::collisionalgorithm
