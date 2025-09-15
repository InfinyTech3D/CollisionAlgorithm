#pragma once

#include <CollisionAlgorithm/geometry/TriangleGeometry.h>
#include <CollisionAlgorithm/elements/TetrahedronElement.h>
#include <CollisionAlgorithm/proximity/TetrahedronProximity.h>
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa::collisionalgorithm {

template<class DataTypes>
class TetrahedronGeometry : public TriangleGeometry<DataTypes> {
public:

    typedef DataTypes TDataTypes;
    typedef TetrahedronElement ELEMENT;
    typedef TetrahedronGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef std::function<BaseProximity::SPtr(const TetrahedronElement * elmt,double f0,double f1,double f2,double f3)> ProximityCreatorFunc;

    SOFA_CLASS(GEOMETRY,Inherit);

    void buildTetrahedronElements() override {
        for (unsigned i=0;i<this->l_topology->getNbTetrahedra();i++) {
            auto tetra = this->l_topology->getTetrahedron(i);

            auto eit = this->l_topology->getEdgesInTetrahedron(i);
            auto tit = this->l_topology->getTrianglesInTetrahedron(i);

            this->tetrahedronElements().insert(TetrahedronElement::create(
                                                   this->pointElements()[tetra[0]],this->pointElements()[tetra[1]],this->pointElements()[tetra[2]],this->pointElements()[tetra[3]],
                                                   this->edgeElements()[eit[0]],this->edgeElements()[eit[1]],this->edgeElements()[eit[2]],this->edgeElements()[eit[3]],this->edgeElements()[eit[4]],this->edgeElements()[eit[5]],
                                                   this->triangleElements()[tit[0]],this->triangleElements()[tit[1]],this->triangleElements()[tit[2]],this->triangleElements()[tit[3]]));
        }
    }

    ElementIterator::SPtr begin(unsigned id = 0) const override { return this->tetrahedronBegin(id); }

};

}
