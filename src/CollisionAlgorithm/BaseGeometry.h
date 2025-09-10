#pragma once

#include <CollisionAlgorithm/CollisionPipeline.h>
#include <CollisionAlgorithm/BaseElement.h>
#include <CollisionAlgorithm/BaseProximity.h>
#include <CollisionAlgorithm/ElementIterator.h>
#include <sofa/gl/gl.h>
#include <sofa/helper/AdvancedTimer.h>
#include <CollisionAlgorithm/elements/PointElement.h>
#include <CollisionAlgorithm/elements/EdgeElement.h>
#include <CollisionAlgorithm/elements/TriangleElement.h>
#include <CollisionAlgorithm/elements/TetrahedronElement.h>
#include <CollisionAlgorithm/InternalData.h>


namespace sofa ::collisionAlgorithm {


/*!
 * \brief The BaseGeometry class is an abstract class defining a basic geometry
 * iterates through Proximity elements and draws them
 */
class BaseGeometry : public CollisionComponent {
public:

    class BroadPhase : public sofa::core::objectmodel::BaseObject {
    public:

        SOFA_ABSTRACT_CLASS(BroadPhase,sofa::core::objectmodel::BaseObject);

        core::objectmodel::SingleLink<BroadPhase,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

        BroadPhase()
        : l_geometry(initLink("geometry", "link to geometry")) {
            l_geometry.setPath("@.");
        }

        void init() {
            if (l_geometry != NULL) l_geometry->setBroadPhase(this);
        }

        virtual type::Vec3i getNbox() = 0;

        virtual type::Vec3i getBoxCoord(const type::Vec3 & P) const = 0;

        virtual const std::set<BaseElement::SPtr> & getElementSet(unsigned i, unsigned j, unsigned k) const = 0;

        const std::type_info& getTypeInfo() const { return l_geometry->getTypeInfo(); }

        virtual void initBroadPhase() = 0;

        virtual void updateBroadPhase() = 0;

    };



    SOFA_ABSTRACT_CLASS(BaseGeometry,CollisionComponent);

    Data<sofa::type::RGBAColor> d_color;
    Data<double> d_drawScaleNormal;
    Data<bool> d_draw;

    BaseGeometry()
    : d_color(initData(&d_color, sofa::type::RGBAColor(1,0,1,1), "color", "Color of the collision model"))
    , d_drawScaleNormal(initData(&d_drawScaleNormal, 1.0, "drawScaleNormal", "Color of the collision model"))
    , d_draw(initData(&d_draw, true, "draw", "draw"))
    , m_broadPhase(NULL) {
        this->f_listening.setValue(true);
    }

    void init() {
        m_pointElements.clear();
        m_edgeElements.clear();
        m_triangleElements.clear();
        m_tetrahedronElements.clear();

        buildPointElements();
        buildEdgeElements();
        buildTriangleElements();
        buildTetrahedronElements();
        prepareDetection();
    }

    virtual void buildPointElements() {}

    virtual void buildEdgeElements() {}

    virtual void buildTriangleElements() {}

    virtual void buildTetrahedronElements() {}

    virtual void prepareDetection() {
        std::string timerName = std::string("Timer for ") + typeid(*this).name();
//        sofa::helper::AdvancedTimer::stepBegin(timerName.c_str());
        for (unsigned i=0;i<m_pointElements.size();i++) m_pointElements[i]->setDirty(true);
        for (unsigned i=0;i<m_edgeElements.size();i++) m_edgeElements[i]->setDirty(true);
        for (unsigned i=0;i<m_triangleElements.size();i++) m_triangleElements[i]->setDirty(true);
        for (unsigned i=0;i<m_tetrahedronElements.size();i++) m_tetrahedronElements[i]->setDirty(true);

        if (m_broadPhase) m_broadPhase->updateBroadPhase();

		auto &  ID = internalData();
		for( auto it = ID.cbegin(); it != ID.cend(); it++)
			it->second->setDirty(true);

//		sofa::helper::AdvancedTimer::stepEnd(timerName.c_str());
	}

    virtual ElementIterator::SPtr begin(unsigned id = 0) const = 0;

    inline const std::type_info& getTypeInfo() const { return begin()->getTypeInfo(); }

    BroadPhase * getBroadPhase() { return m_broadPhase; }

    inline const BaseGeometry * end() const { return this; }

    virtual unsigned getSize() const = 0;

    virtual type::Vec3 getPosition(unsigned pid, core::VecCoordId v = core::vec_id::write_access::position) const = 0;

    virtual type::Vec3 getVelocity(unsigned pid, core::VecDerivId v = core::vec_id::write_access::velocity) const = 0;

    void draw(const core::visual::VisualParams * vparams) override {
        if (! vparams->displayFlags().getShowCollisionModels()) return;

        if (! d_draw.getValue()) return;
        type::RGBAColor color = d_color.getValue();

        glDisable(GL_LIGHTING);
        if (color[3] == 0.0) return;

        glColor4f(color[0],color[1],color[2],color[3]);
        for (auto it = begin();it != end(); it++) {
            it->element()->draw(vparams);
        }        
    }

    inline ElementIterator::SPtr pointBegin(unsigned id = 0) const {
        return ElementIterator::SPtr(new TDefaultElementIterator_ref(m_pointElements,id));
    }

    inline ElementIterator::SPtr edgeBegin(unsigned id = 0) const {
        return ElementIterator::SPtr(new TDefaultElementIterator_ref(m_edgeElements,id));
    }

    inline ElementIterator::SPtr triangleBegin(unsigned id = 0) const {
        return ElementIterator::SPtr(new TDefaultElementIterator_ref(m_triangleElements,id));
    }

    inline ElementIterator::SPtr tetrahedronBegin(unsigned id = 0) const {
        return ElementIterator::SPtr(new TDefaultElementIterator_ref(m_tetrahedronElements,id));
    }

    void setBroadPhase(BroadPhase * b) {
        m_broadPhase = b;
        this->addSlave(m_broadPhase);
        m_broadPhase->initBroadPhase();
    }

    virtual ElementContainer<PointElement> & pointElements() { return m_pointElements; }

    virtual ElementContainer<EdgeElement> & edgeElements() { return m_edgeElements; }

    virtual ElementContainer<TriangleElement> & triangleElements() { return m_triangleElements; }

    virtual ElementContainer<TetrahedronElement> & tetrahedronElements() { return m_tetrahedronElements; }


    virtual InternalDataContainer & internalData() { return m_internalData; }


private:

    ElementContainer<PointElement> m_pointElements;
    ElementContainer<EdgeElement> m_edgeElements;
    ElementContainer<TriangleElement> m_triangleElements;
    ElementContainer<TetrahedronElement> m_tetrahedronElements;

    BroadPhase * m_broadPhase;
    InternalDataContainer m_internalData;

};

template<class DataTypes>
class TBaseGeometry : public BaseGeometry {
public:

    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::Deriv Deriv1;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    SOFA_ABSTRACT_CLASS(SOFA_TEMPLATE(TBaseGeometry,DataTypes),BaseGeometry);

    core::objectmodel::SingleLink<TBaseGeometry<DataTypes>,State,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_state;

    TBaseGeometry()
    : l_state(initLink("mstate", "link to state")) {
        l_state.setPath("@.");
//        for (unsigned j=0; j<this->getState()->getSize(); j++) {
//            m_topoProx.push_back(TBaseProximity<DataTypes>::template create<TopologyProximity<DataTypes>>(l_state, j));
// //            m_topoProx.push_back(BaseProximity::create<TopologyProximity<DataTypes>>(l_state, j));
//        }
    }



//    virtual void buildTopologyProximity() {
//        this->m_topoProx.clear();
//        for (unsigned j=0; j<this->getState()->getSize(); j++) {
////            m_topoProx.push_back(TBaseProximity<DataTypes>::template create<TopologyProximity<DataTypes>>(l_state, j));
//             m_topoProx.push_back(BaseProximity::create<TopologyProximity<DataTypes>>(l_state, j));
//        }
//    }

    unsigned getSize() const override {
        return l_state->getSize();
    }

    sofa::type::Vec3 getPosition(unsigned pid, core::VecCoordId v = core::vec_id::write_access::position) const override {
        const helper::ReadAccessor<DataVecCoord> & pos = l_state->read(v);
        return pos[pid];

    }

    sofa::type::Vec3 getVelocity(unsigned pid, core::VecDerivId v = core::vec_id::write_access::velocity) const override {
        const helper::ReadAccessor<DataVecDeriv> & vel = l_state->read(v);
        return vel[pid];

    }



    inline sofa::core::behavior::MechanicalState<DataTypes> * getState() const {
        return l_state.get();
    }

//    inline std::vector<typename TopologyProximity<DataTypes>::SPtr> & getTopoProx(){
//        return m_topoProx;
//    }

//    inline typename TopologyProximity<DataTypes>::SPtr getTopoProxIdx(unsigned i){
//        return m_topoProx[i];
//    }

//	inline std::vector<BaseElement::SPtr > getBaseElements() = 0;

    inline void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const {
        auto res = sofa::helper::getWriteAccessor(*resId[this->getState()].write());
        const typename DataTypes::MatrixDeriv& j = cParams->readJ(this->getState())->getValue();
        j.multTransposeBaseVector(res, lambda );
    }


    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MechanicalState.
    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg) {
        sofa::core::behavior::MechanicalState<DataTypes>* state = NULL;
        context->get(state);
        if (state == NULL) return false;
        return sofa::core::objectmodel::BaseObject::canCreate(obj, context, arg);
    }

};



}

