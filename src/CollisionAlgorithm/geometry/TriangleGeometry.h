#pragma once

#include <CollisionAlgorithm/geometry/EdgeGeometry.h>
#include <CollisionAlgorithm/proximity/TriangleProximity.h>
#include <CollisionAlgorithm/elements/TriangleElement.h>
#include <CollisionAlgorithm/toolbox/TriangleToolBox.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class TriangleGeometry : public EdgeGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TriangleElement ELEMENT;
    typedef TriangleGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef std::function<BaseProximity::SPtr(const TriangleElement * elmt, double f0,double f1,double f2)> ProximityCreatorFunc;

    SOFA_CLASS(GEOMETRY,Inherit);

    void buildTriangleElements() override {
        for (unsigned i=0;i<this->l_topology->getNbTriangles();i++) {
            auto triangle = this->l_topology->getTriangle(i);
            auto edgeId = this->l_topology->getEdgesInTriangle(i);

            PointElement::SPtr point0 = this->pointElements()[triangle[0]];
            PointElement::SPtr point1 = this->pointElements()[triangle[1]];
            PointElement::SPtr point2 = this->pointElements()[triangle[2]];

            EdgeElement::SPtr edge0 = this->edgeElements()[edgeId[0]];
            EdgeElement::SPtr edge1 = this->edgeElements()[edgeId[1]];
            EdgeElement::SPtr edge2 = this->edgeElements()[edgeId[2]];

            this->triangleElements().insert(TriangleElement::create(point0, point1, point2, edge0, edge1, edge2));
        }
    }

    inline ElementIterator::SPtr begin(unsigned id = 0) const override { return this->triangleBegin(id); }

};

}

