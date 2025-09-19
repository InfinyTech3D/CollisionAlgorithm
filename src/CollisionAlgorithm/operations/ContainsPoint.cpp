#include <CollisionAlgorithm/operations/ContainsPoint.h>
#include <CollisionAlgorithm/proximity/TetrahedronProximity.h>
#include <CollisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa::collisionalgorithm::Operations::ContainsPointInElement
{

int register_ContainsPoint_Triangle =
    Operation::register_func<TriangleElement>(&toolbox::TriangleToolBox::containsPoint);

int register_ContainsPoint_Tetrahedron =
    Operation::register_func<TetrahedronElement>(&toolbox::TetrahedronToolBox::containsPoint);

}  // namespace sofa::collisionalgorithm::Operations::ContainsPointInElement

namespace sofa::collisionalgorithm::Operations::ContainsPointInProximity
{

int register_ContainsPointInProximity_Triangle =
    Operation::register_func<TriangleElement>(&containsPoint<TriangleProximity>);

int register_ContainsPointInProximity_Tetrahedron =
    Operation::register_func<TetrahedronElement>(&containsPoint<TetrahedronProximity>);

}  // namespace sofa::collisionalgorithm::Operations::ContainsPointInProximity
