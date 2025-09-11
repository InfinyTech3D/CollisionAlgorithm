#pragma once

#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa::collisionAlgorithm::Operations::Needle
{

class PrunePointsAheadOfTip
    : public GenericOperation<PrunePointsAheadOfTip,  // Type of the operation
                              bool,                   // Default return type
                              std::vector<BaseProximity::SPtr>&,
                              const BaseProximity::SPtr&  // Parameters
                              >
{
   public:
    bool defaultFunc(std::vector<BaseProximity::SPtr>&, const BaseProximity::SPtr&) const override
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

}  // namespace sofa::collisionAlgorithm::Operations::Needle
