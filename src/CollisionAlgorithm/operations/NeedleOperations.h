#pragma once

#include <CollisionAlgorithm/config.h>
#include <CollisionAlgorithm/BaseOperation.h>
#include <CollisionAlgorithm/BaseProximity.h>
#include <CollisionAlgorithm/elements/EdgeElement.h>

namespace sofa::collisionalgorithm::Operations::Needle
{

class SOFA_COLLISIONALGORITHM_API PrunePointsAheadOfTip
    : public GenericOperation<PrunePointsAheadOfTip,  // Type of the operation
                              bool,                   // Default return type
                              std::vector<BaseProximity::SPtr>&,
                              const BaseElement::SPtr&  // Parameters
                              >
{
   public:
    bool defaultFunc(std::vector<BaseProximity::SPtr>&, const BaseElement::SPtr&) const override
    {
        return false;
    }

    void notFound(const std::type_info& id) const override
    {
        msg_error("Needle::PrunePointsAheadOfTip")
            << "The operation PrunePointsAheadOfTipOperation is not registered with for type = "
            << sofa::helper::NameDecoder::decodeFullName(id);
    }
};

/**
* Returns true when at least one coupling point was popped from the back, 
* false when the set was left unchanged (including the null-edge error path
* and the non-retracting early-exit).
*/
bool prunePointsUsingEdges(std::vector<BaseProximity::SPtr>& couplingPts,
                           const EdgeElement::SPtr& edgeProx);

}  // namespace sofa::collisionalgorithm::Operations::Needle
