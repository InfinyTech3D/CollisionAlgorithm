#pragma once

#include <CollisionAlgorithm/BaseElement.h>
#include <sofa/gl/gl.h>

namespace sofa::collisionAlgorithm {

class PointElement : public BaseElement {
public:

    typedef std::shared_ptr<PointElement> SPtr;

    const std::type_info& getTypeInfo() const override { return typeid(PointElement); }

    std::string name() const override { return "PointElement"; }

    inline BaseProximity::SPtr getP0() const { return m_prox; }

    void update() override {}

    void draw(const core::visual::VisualParams * /*vparams*/) override {
        type::Vec3 p0 = m_prox->getPosition();

        glBegin(GL_POINT);
            glVertex3dv(p0.data());
        glEnd();
    }

    ElementContainer<TriangleElement> & triangleAround() { return m_triangleAround; }

    static SPtr create(const BaseProximity::SPtr & prox);

    const ElementContainer<PointElement> & pointElements() const override { return ElementContainer<PointElement>::empty(); }

    const ElementContainer<EdgeElement> & edgeElements() const override { return ElementContainer<EdgeElement>::empty(); }

    const ElementContainer<TriangleElement> & triangleElements() const override { return ElementContainer<TriangleElement>::empty(); }

    const ElementContainer<TetrahedronElement> & tetrahedronElements() const override { return ElementContainer<TetrahedronElement>::empty(); }

private:
    PointElement(BaseProximity::SPtr prox) : m_prox(prox) {}

    BaseProximity::SPtr m_prox;
    ElementContainer<TriangleElement> m_triangleAround;

//    ElementContainer<PointElement> m_pointElements;
//    ElementContainer<EdgeElement> m_edgeElements;
//    ElementContainer<TriangleElement> m_triangleElements;
//    ElementContainer<TetrahedronElement> m_tetrahedronElements;
};


}
