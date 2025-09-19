#pragma once

#include <CollisionAlgorithm/BaseElement.h>
#include <CollisionAlgorithm/BaseOperation.h>

namespace sofa::collisionalgorithm::Operations::ContainsPointInElement
{

typedef bool Result;

class Operation : public GenericOperation<Operation,  // Type of the operation
                                          Result,     // Default return type
                                          const type::Vec3&, const BaseElement::SPtr&  // Parameters
                                          >
{
   public:
    Result defaultFunc(const type::Vec3&, const BaseElement::SPtr&) const override { return false; }

    void notFound(const std::type_info& id) const override
    {
        msg_error("ContainsPointInElement")
            << "The operation ContainsPointInElementOperation is not registered with for type = "
            << sofa::helper::NameDecoder::decodeFullName(id);
    }
};

typedef Operation::FUNC FUNC;

}  // namespace sofa::collisionalgorithm::Operations::ContainsPointInElement

namespace sofa::collisionalgorithm::Operations::ContainsPointInProximity
{

typedef bool Result;

class Operation
    : public GenericOperation<Operation,  // Type of the operation
                              Result,     // Default return type
                              const type::Vec3&, const BaseProximity::SPtr&  // Parameters
                              >
{
   public:
    Result defaultFunc(const type::Vec3&, const BaseProximity::SPtr&) const override
    {
        return false;
    }

    void notFound(const std::type_info& id) const override
    {
        msg_error("ContainsPointInProximity")
            << "The operation ContainsPointInProximityOperation is not registered with for type = "
            << sofa::helper::NameDecoder::decodeFullName(id);
    }
};

template <typename PROX>
Result containsPoint(const type::Vec3& P, const typename std::shared_ptr<PROX>& prox)
{
    if (!prox)
    {
        msg_warning("ContainsPointInProximity") << "Null proximity pointer in containsPoint"
            << "Operation is disabled; returning false"; 
        return false;
    }

    auto elem = prox->element();
    auto containsPointInElem =
        sofa::collisionalgorithm::Operations::ContainsPointInElement::Operation::get(elem);
    return containsPointInElem(P, elem);
}

typedef Operation::FUNC FUNC;

}  // namespace sofa::collisionalgorithm::Operations::ContainsPointInProximity
