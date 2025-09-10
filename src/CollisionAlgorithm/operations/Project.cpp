#include <CollisionAlgorithm/operations/Project.h>
#include <CollisionAlgorithm/toolbox/PointToolBox.h>
#include <CollisionAlgorithm/toolbox/EdgeToolBox.h>
#include <CollisionAlgorithm/toolbox/TriangleToolBox.h>
#include <CollisionAlgorithm/toolbox/TetrahedronToolBox.h>

namespace sofa::collisionAlgorithm::Operations::Project {

int register_Project_Point = Operation::register_func<PointElement>(&toolbox::PointToolBox::project);

int register_Project_Edge = Operation::register_func<EdgeElement>(&toolbox::EdgeToolBox::project);

int register_Project_Triangle = Operation::register_func<TriangleElement>(&toolbox::TriangleToolBox::project);

int register_Project_Tetrahedron = Operation::register_func<TetrahedronElement>(&toolbox::TetrahedronToolBox::project);

}

