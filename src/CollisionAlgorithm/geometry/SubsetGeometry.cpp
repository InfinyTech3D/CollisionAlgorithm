#include <CollisionAlgorithm/config.h>
#include <CollisionAlgorithm/geometry/SubsetGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionalgorithm
{

template class SOFA_COLLISIONALGORITHM_API SubsetGeometry<sofa::defaulttype::Vec3dTypes>;

void registerSubsetGeometry(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData("")
            .add<SubsetGeometry<sofa::defaulttype::Vec3dTypes> >());
}
}  // namespace sofa::collisionalgorithm
