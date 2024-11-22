#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

int TriangleGeometryClass = core::RegisterObject("TriangleGeometry")
.add< TriangleGeometry<sofa::defaulttype::Vec3dTypes> >();

}
