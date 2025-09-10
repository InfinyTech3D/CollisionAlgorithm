#pragma once

#include <CollisionAlgorithm/geometry/PointGeometry.h>
#include <CollisionAlgorithm/toolbox/EdgeToolBox.h>
#include <CollisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class EdgeGeometry : public PointGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef EdgeElement ELEMENT;
    typedef EdgeGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef std::function<BaseProximity::SPtr(const EdgeElement * elmt,double f0,double f1)> ProximityCreatorFunc;

    SOFA_CLASS(GEOMETRY,Inherit);

    core::objectmodel::SingleLink<GEOMETRY,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

    EdgeGeometry()
    : l_topology(initLink("topology", "link to topology")) {
        l_topology.setPath("@.");
    }

    virtual void buildEdgeElements() override {
        for (unsigned i=0;i<this->l_topology->getNbEdges();i++) {
            auto edge = this->l_topology->getEdge(i);

            PointElement::SPtr point0 = this->pointElements()[edge[0]];
            PointElement::SPtr point1 = this->pointElements()[edge[1]];

            this->edgeElements().insert(EdgeElement::create(point0,point1));
        }
    }

    inline ElementIterator::SPtr begin(unsigned id = 0) const override { return this->edgeBegin(id); }


};

}
