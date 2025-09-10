#include <CollisionAlgorithm/operations/CreateCenterProximity.h>
#include <CollisionAlgorithm/toolbox/PointToolBox.h>
#include <CollisionAlgorithm/toolbox/EdgeToolBox.h>
#include <CollisionAlgorithm/toolbox/TriangleToolBox.h>
#include <CollisionAlgorithm/toolbox/TetrahedronToolBox.h>

namespace sofa::collisionAlgorithm::Operations::CreateCenterProximity {

int register_CenterProximity_Point = Operation::register_func<PointElement>(&toolbox::PointToolBox::createCenterProximity);

int register_CenterProximity_Edge = Operation::register_func<EdgeElement>(&toolbox::EdgeToolBox::createCenterProximity);

int register_CenterProximity_Triangle = Operation::register_func<TriangleElement>(&toolbox::TriangleToolBox::createCenterProximity);

int register_CenterProximity_Tetrahedron = Operation::register_func<TetrahedronElement>(&toolbox::TetrahedronToolBox::createCenterProximity);

}

