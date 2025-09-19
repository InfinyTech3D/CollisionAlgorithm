#pragma once

#include <CollisionAlgorithm/BaseElement.h>
#include <CollisionAlgorithm/elements/EdgeElement.h>

namespace sofa::collisionalgorithm {

class SOFA_COLLISIONALGORITHM_API TriangleElement : public BaseElement {
public:
    typedef std::shared_ptr<TriangleElement> SPtr;

    struct TriangleInfo
    {
        type::Vec3d v0,v1;
        double d00;
        double d01;
        double d11;
        double invDenom;
        double area;

        type::Vec3d ax1,ax2;
        type::Vec3d P0,P1,P2;
        type::Vec3d N;

        void update(const type::Vec3d & p0, const type::Vec3d & p1,const type::Vec3d & p2) {
            P0 = p0;
            P1 = p1;
            P2 = p2;

            v0 = P1 - P0;
            v1 = P2 - P0;
            N=cross(v1,v0);
            area = N.norm()/2;
            N.normalize();

            d00 = dot(v0,v0);
            d01 = dot(v0,v1);
            d11 = dot(v1,v1);

            invDenom = 1.0 / (d00 * d11 - d01 * d01);

            ax1 = v0;
            ax2 = v0.cross(N);

            ax1.normalize();
            ax2.normalize();
        }
    };

    const std::type_info& getTypeInfo() const override { return typeid(TriangleElement); }

    std::string name() const override { return "TriangleElement"; }

    void update() override {
        m_tinfo.update(getP0()->getPosition(),
                       getP1()->getPosition(),
                       getP2()->getPosition());
    }

    inline const TriangleInfo & getTriangleInfo() {
        if (m_isDirty) {
            update();
            setDirty(false);
        }
        return m_tinfo;
    }

    inline BaseProximity::SPtr getP0() const { return this->pointElements()[0]->getP0(); }

    inline BaseProximity::SPtr getP1() const { return this->pointElements()[1]->getP0(); }

    inline BaseProximity::SPtr getP2() const { return this->pointElements()[2]->getP0(); }

    void draw(const core::visual::VisualParams * vparams) override {
        type::Vec3 p0 = getP0()->getPosition();
        type::Vec3 p1 = getP1()->getPosition();
        type::Vec3 p2 = getP2()->getPosition();

        if (vparams->displayFlags().getShowWireFrame()) {
            glBegin(GL_LINES);
                glVertex3dv(p0.data());glVertex3dv(p1.data());
                glVertex3dv(p1.data());glVertex3dv(p2.data());
                glVertex3dv(p2.data());glVertex3dv(p0.data());
            glEnd();
        } else {
            glBegin(GL_TRIANGLES);
                glVertex3dv(p0.data());
                glVertex3dv(p1.data());
                glVertex3dv(p2.data());
            glEnd();
        }
    }

    static TriangleElement::SPtr create(BaseProximity::SPtr p0, BaseProximity::SPtr p1,BaseProximity::SPtr p2);

    static TriangleElement::SPtr create(PointElement::SPtr p0, PointElement::SPtr p1, PointElement::SPtr p2,
                                        EdgeElement::SPtr edge0, EdgeElement::SPtr edge1, EdgeElement::SPtr edge2);

    const ElementContainer<PointElement> & pointElements() const override { return m_pointElements; }

    const ElementContainer<EdgeElement> & edgeElements() const override { return m_edgeElements; }

    const ElementContainer<TriangleElement> & triangleElements() const override { return ElementContainer<TriangleElement>::empty(); }

    const ElementContainer<TetrahedronElement> & tetrahedronElements() const override { return ElementContainer<TetrahedronElement>::empty(); }

private:
    TriangleElement() {}

    TriangleInfo m_tinfo;

    ElementContainer<PointElement> m_pointElements;
    ElementContainer<EdgeElement> m_edgeElements;
//    ElementContainer<TriangleElement> m_triangleElements;
};

}
