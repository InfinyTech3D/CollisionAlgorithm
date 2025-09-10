#pragma once

#include <CollisionAlgorithm/BaseGeometry.h>
#include <CollisionAlgorithm/proximity/PointProximity.h>
#include <CollisionAlgorithm/toolbox/PointToolBox.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class SubsetGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef PointElement ELEMENT;
    typedef SubsetGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef std::function<BaseProximity::SPtr(const PointElement * elmt)> ProximityCreatorFunc;

    SOFA_CLASS(GEOMETRY,Inherit);

    core::objectmodel::SingleLink<GEOMETRY,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;
    Data<type::vector<int> > d_indices;

    SubsetGeometry()
      : l_geometry(initLink("wholeGeometry", "link to topology"))
      , d_indices(initData(&d_indices,"indices", "Indices to keep"))
    {
      l_geometry.setPath("@.");
    }
//    type::Vec3 getPosition(unsigned pid) override {
//        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
//        return pos[pid];
//    }

	void init()
	{
		type::vector<int>  indices = d_indices.getValue();

		std::sort(indices.begin(), indices.end());

		m_elements.clear();
		auto iter = l_geometry->begin();

		int id=0;
		for(int keepId : indices)
		{
			while(id != keepId)
			{
				++id;
				iter++;
			}
			m_elements.push_back(iter->element());
		}
		std::cout<<"SubsetGeometry : "<<m_elements.size()<<std::endl;

	}


    void prepareDetection() override {}

    ElementIterator::SPtr begin(unsigned id = 0) const override {
        return ElementIterator::SPtr(new TDefaultElementIterator_ref(m_elements,id));
    }

private:
    std::vector<BaseElement::SPtr> m_elements;
};

}
