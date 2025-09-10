#include <CollisionAlgorithm/elements/PointElement.h>
#include <CollisionAlgorithm/elements/EdgeElement.h>
#include <CollisionAlgorithm/elements/TriangleElement.h>
#include <CollisionAlgorithm/elements/TetrahedronElement.h>
#include <CollisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa::collisionAlgorithm {

TriangleElement::SPtr TriangleElement::create(BaseProximity::SPtr prox0, BaseProximity::SPtr prox1,BaseProximity::SPtr prox2) {
    PointElement::SPtr p0 = PointElement::create(prox0);
    PointElement::SPtr p1 = PointElement::create(prox1);
    PointElement::SPtr p2 = PointElement::create(prox2);

    EdgeElement::SPtr e0 = EdgeElement::create(p0,p1);
    EdgeElement::SPtr e1 = EdgeElement::create(p1,p2);
    EdgeElement::SPtr e2 = EdgeElement::create(p2,p0);

    return TriangleElement::create(p0,p1,p2,e0,e1,e2);
}

TriangleElement::SPtr TriangleElement::create(PointElement::SPtr p0, PointElement::SPtr p1, PointElement::SPtr p2,
                                              EdgeElement::SPtr edge0, EdgeElement::SPtr edge1, EdgeElement::SPtr edge2) {
    auto res = TriangleElement::SPtr(new TriangleElement());

    res->m_pointElements.insert(p0);
    res->m_pointElements.insert(p1);
    res->m_pointElements.insert(p2);

    res->m_edgeElements.insert(edge0);
    res->m_edgeElements.insert(edge1);
    res->m_edgeElements.insert(edge2);

    p0->triangleAround().insert(res);
    p1->triangleAround().insert(res);
    p2->triangleAround().insert(res);

//    res->m_triangleElements.insert(res.get());

    return res;
}



}
