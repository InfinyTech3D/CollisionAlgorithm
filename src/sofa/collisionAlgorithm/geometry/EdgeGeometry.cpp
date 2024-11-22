#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

int EdgeGeometryClass = core::RegisterObject("EdgeGeometry")
.add< EdgeGeometry<sofa::defaulttype::Vec3dTypes> >();

}

}
