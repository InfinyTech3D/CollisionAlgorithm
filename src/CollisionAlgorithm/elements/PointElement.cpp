#include <CollisionAlgorithm/elements/PointElement.h>
#include <CollisionAlgorithm/elements/EdgeElement.h>
#include <CollisionAlgorithm/elements/TriangleElement.h>
#include <CollisionAlgorithm/elements/TetrahedronElement.h>
#include <CollisionAlgorithm/proximity/PointProximity.h>

namespace sofa::collisionalgorithm {

template class SOFA_COLLISIONALGORITHM_API ElementContainer<PointElement>;

PointElement::SPtr PointElement::create(const BaseProximity::SPtr &prox) {
    return PointElement::SPtr(new PointElement(prox));
}

}
