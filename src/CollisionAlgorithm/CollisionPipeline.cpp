#include <CollisionAlgorithm/CollisionPipeline.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionalgorithm
{
void registerCollisionLoop(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData(
            "A collision pipeline customized for proximity detection during needle insertion")
            .add<CollisionLoop>());
}
}  // namespace sofa::collisionalgorithm
