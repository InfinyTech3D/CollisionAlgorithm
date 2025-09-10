#include <CollisionAlgorithm/geometry/TetrahedronGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm
{
void registerTetrahedronGeometry(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData(
                                 "A class bridging tetrahedron topological information with "
                                 "the proximity detection algorithm")
                                 .add<TetrahedronGeometry<sofa::defaulttype::Vec3dTypes> >());
}
}  // namespace sofa::collisionAlgorithm
