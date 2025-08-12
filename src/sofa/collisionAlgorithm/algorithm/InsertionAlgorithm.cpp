#include <sofa/collisionAlgorithm/algorithm/InsertionAlgorithm.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm
{
void registerInsertionAlgorithm(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData(
                                 "A class implementing a customized needle insertion algorithm")
                                 .add<InsertionAlgorithm>());
}
}  // namespace sofa::collisionAlgorithm
