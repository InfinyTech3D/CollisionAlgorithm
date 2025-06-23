#include <sofa/collisionAlgorithm/algorithm/PunctureAlgorithm.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

int PunctureAlgorithmClass = core::RegisterObject("PunctureAlgorithm")
.add< PunctureAlgorithm >();


}



