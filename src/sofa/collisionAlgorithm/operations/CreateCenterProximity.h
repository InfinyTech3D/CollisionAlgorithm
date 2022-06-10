#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/proximity/MultiProximity.h>

namespace sofa::collisionAlgorithm::Operations::CreateCenterProximity {

typedef BaseProximity::SPtr Result;

class Operation : public GenericOperation<Operation, //type of operation
                                          Result, //Default return type
                                          const BaseElement::SPtr &//Parameters
                                          > {
public:

    BaseProximity::SPtr defaultFunc(const BaseElement::SPtr &) const override {
        return NULL;
    }

    void notFound(const std::type_info & id) const override {
        std::cerr << "ERROR the operation CreateCenterProximityOperation is not registered with for type = " << sofa::helper::NameDecoder::decodeFullName(id) << std::endl;
    }

};

typedef Operation::FUNC FUNC;

}

