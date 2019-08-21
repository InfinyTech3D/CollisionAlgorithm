#include <sofa/collisionAlgorithm/geometry/TetrahedronGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(TetrahedronGeometry)

int TetrahedronGeometryClass = core::RegisterObject("TetrahedronGeometry")
.add< TetrahedronGeometry<sofa::defaulttype::Vec3dTypes> >();

}

}
