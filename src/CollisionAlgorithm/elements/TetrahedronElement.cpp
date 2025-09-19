#include <CollisionAlgorithm/elements/PointElement.h>
#include <CollisionAlgorithm/elements/EdgeElement.h>
#include <CollisionAlgorithm/elements/TriangleElement.h>
#include <CollisionAlgorithm/elements/TetrahedronElement.h>
#include <CollisionAlgorithm/proximity/TetrahedronProximity.h>

namespace sofa::collisionalgorithm {

TetrahedronElement::SPtr TetrahedronElement::create(PointElement::SPtr p0,PointElement::SPtr p1,PointElement::SPtr p2,PointElement::SPtr p3,
                                                    EdgeElement::SPtr e0, EdgeElement::SPtr e1, EdgeElement::SPtr e2, EdgeElement::SPtr e3,EdgeElement::SPtr e4,EdgeElement::SPtr e5,
                                                    TriangleElement::SPtr tri0, TriangleElement::SPtr tri1, TriangleElement::SPtr tri2, TriangleElement::SPtr tri3) {

    TetrahedronElement::SPtr res = TetrahedronElement::SPtr(new TetrahedronElement());

    res->m_pointElements.insert(p0);
    res->m_pointElements.insert(p1);
    res->m_pointElements.insert(p2);
    res->m_pointElements.insert(p3);

    res->m_edgeElements.insert(e0);
    res->m_edgeElements.insert(e1);
    res->m_edgeElements.insert(e2);
    res->m_edgeElements.insert(e3);
    res->m_edgeElements.insert(e4);
    res->m_edgeElements.insert(e5);

    res->m_triangleElements.insert(tri0);
    res->m_triangleElements.insert(tri1);
    res->m_triangleElements.insert(tri2);
    res->m_triangleElements.insert(tri3);

//    res->m_tetrahedronElements.insert(res.get());

    return res;
}

TetrahedronElement::SPtr TetrahedronElement::create(BaseProximity::SPtr prox0, BaseProximity::SPtr prox1,BaseProximity::SPtr prox2,BaseProximity::SPtr prox3) {
    PointElement::SPtr p0 = PointElement::create(prox0);
    PointElement::SPtr p1 = PointElement::create(prox1);
    PointElement::SPtr p2 = PointElement::create(prox2);
    PointElement::SPtr p3 = PointElement::create(prox3);

    EdgeElement::SPtr e0 = EdgeElement::create(p0,p1);
    EdgeElement::SPtr e1 = EdgeElement::create(p1,p2);
    EdgeElement::SPtr e2 = EdgeElement::create(p2,p0);

    EdgeElement::SPtr e3 = EdgeElement::create(p3,p0);
    EdgeElement::SPtr e4 = EdgeElement::create(p3,p1);
    EdgeElement::SPtr e5 = EdgeElement::create(p3,p2);

    TriangleElement::SPtr t0 = TriangleElement::create(p0,p1,p2,e0,e1,e2);
    TriangleElement::SPtr t1 = TriangleElement::create(p0,p1,p2,e0,e4,e3);
    TriangleElement::SPtr t2 = TriangleElement::create(p0,p1,p2,e4,e1,e5);
    TriangleElement::SPtr t3 = TriangleElement::create(p0,p1,p2,e3,e2,e5);

    return TetrahedronElement::create(p0,p1,p2,p3,
                                      e0,e1,e2,e3,e4,e5,
                                      t0,t1,t2,t3);
}

}
