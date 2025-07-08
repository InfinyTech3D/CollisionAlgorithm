#include <sofa/collisionAlgorithm/algorithm/InsertionAlgorithm.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

int InsertionAlgorithmClass = core::RegisterObject("InsertionAlgorithm")
.add< InsertionAlgorithm >();


}



