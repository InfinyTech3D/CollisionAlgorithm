#pragma once

//#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/simulation/CollisionBeginEvent.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/core/BehaviorModel.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/simulation/Visitor.h>
#include <sofa/gl/gl.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/simulation/Node.h>

namespace sofa ::collisionAlgorithm {

class CollisionComponent : public core::objectmodel::BaseObject {
public:

    virtual void update() = 0;

};

class CollisionAlgorithm : public core::objectmodel::BaseObject {
public:

    virtual void doDetection() = 0;

};

class CollisionLoop : public core::objectmodel::BaseObject {
public:

    SOFA_ABSTRACT_CLASS(CollisionLoop,core::objectmodel::BaseObject);

    class UpdateComponentVisitor : public simulation::Visitor {
    public:
        UpdateComponentVisitor() : Visitor(sofa::core::ExecParams::defaultInstance()) {}

        Visitor::Result processNodeTopDown(simulation::Node* node) {
            for_each(this, node, node->object, &UpdateComponentVisitor::processObject);
            return Visitor::RESULT_CONTINUE;
        }

        void processObject(simulation::Node*, core::objectmodel::BaseObject* obj) {
            if (CollisionComponent * component = dynamic_cast<CollisionComponent *>(obj)) {
                component->update();
            }
        }

        virtual const char* getClassName() const {return "UpdateComponentVisitor";}
    };


    class UpdateAlgorithmVisitor : public simulation::Visitor {
    public:
        UpdateAlgorithmVisitor() : Visitor(sofa::core::ExecParams::defaultInstance()) {}

        Visitor::Result processNodeTopDown(simulation::Node* node) {
            for_each(this, node, node->object, &UpdateAlgorithmVisitor::processObject);
            return Visitor::RESULT_CONTINUE;
        }

        void processObject(simulation::Node*, core::objectmodel::BaseObject* obj) {
            if (CollisionAlgorithm * component = dynamic_cast<CollisionAlgorithm *>(obj)) {
                component->doDetection();
            }
        }

        virtual const char* getClassName() const {return "UpdateAlgorithmVisitor";}
    };

    CollisionLoop() {
        this->f_listening.setValue(true);
    }

    void handleEvent(sofa::core::objectmodel::Event *event) {
        if (! dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event)) return;

        UpdateComponentVisitor v_comp;
        v_comp.execute(this->getContext());

        UpdateAlgorithmVisitor v_algo;
        v_algo.execute(this->getContext());
    }

};

}

