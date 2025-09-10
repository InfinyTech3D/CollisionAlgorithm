#include <CollisionAlgorithm/geometry/SubsetGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm
{
void registerSubsetGeometry(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData("")
            .add<SubsetGeometry<sofa::defaulttype::Vec3dTypes> >());
}
}  // namespace sofa::collisionAlgorithm
