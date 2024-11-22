#include <sofa/collisionAlgorithm/geometry/SubsetGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

int SubsetGeometryClass = core::RegisterObject("SubsetGeometry")
.add< SubsetGeometry<sofa::defaulttype::Vec3dTypes> >();

}

}


