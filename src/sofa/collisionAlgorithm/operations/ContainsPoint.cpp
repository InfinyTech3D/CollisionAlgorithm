#include <sofa/collisionAlgorithm/operations/ContainsPoint.h>
#include <sofa/collisionAlgorithm/proximity/TetrahedronProximity.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa::collisionAlgorithm::Operations::ContainsPointInElement
{

int register_ContainsPoint_Triangle =
    Operation::register_func<TriangleElement>(&toolbox::TriangleToolBox::containsPoint);

int register_ContainsPoint_Tetrahedron =
    Operation::register_func<TetrahedronElement>(&toolbox::TetrahedronToolBox::containsPoint);

}  // namespace sofa::collisionAlgorithm::Operations::ContainsPointInElement

namespace sofa::collisionAlgorithm::Operations::ContainsPointInProximity
{

int register_ContainsPointInProximity_Triangle =
    Operation::register_func<TriangleElement>(&containsPoint<TriangleProximity>);

int register_ContainsPointInProximity_Tetrahedron =
    Operation::register_func<TetrahedronElement>(&containsPoint<TetrahedronProximity>);

}  // namespace sofa::collisionAlgorithm::Operations::ContainsPointInProximity
