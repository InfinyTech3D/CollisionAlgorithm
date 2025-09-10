#pragma once

#include <CollisionAlgorithm/BaseElement.h>
#include <CollisionAlgorithm/elements/PointElement.h>

namespace sofa::collisionAlgorithm {

class EdgeElement : public BaseElement {
public:
    typedef std::shared_ptr<EdgeElement> SPtr;

    inline BaseProximity::SPtr getP0() const { return pointElements()[0]->getP0(); }

    inline BaseProximity::SPtr getP1() const { return pointElements()[1]->getP0(); }

    const std::type_info& getTypeInfo() const override { return typeid(EdgeElement); }

    std::string name() const override { return "EdgeElement"; }

    void update() override {}

    void draw(const core::visual::VisualParams * /*vparams*/) override {

        type::Vec3 p0 = getP0()->getPosition();
        type::Vec3 p1 = getP1()->getPosition();

        glBegin(GL_LINES);
            glVertex3dv(p0.data());
            glVertex3dv(p1.data());
        glEnd();
    }

    static SPtr create(const PointElement::SPtr & point0, const PointElement::SPtr & point1);

    static SPtr create(const BaseProximity::SPtr & p0,const BaseProximity::SPtr & p1);

    const ElementContainer<PointElement> & pointElements() const override { return m_pointElements; }

    const ElementContainer<EdgeElement> & edgeElements() const override { return ElementContainer<EdgeElement>::empty(); }

    const ElementContainer<TriangleElement> & triangleElements() const override { return ElementContainer<TriangleElement>::empty(); }

    const ElementContainer<TetrahedronElement> & tetrahedronElements() const override { return ElementContainer<TetrahedronElement>::empty(); }

private:
    EdgeElement() {}

    ElementContainer<PointElement> m_pointElements;
//    ElementContainer<EdgeElement> m_edgeElements;
};

}
