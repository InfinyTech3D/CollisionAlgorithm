#include <CollisionAlgorithm/operations/NeedleOperations.h>

namespace sofa::collisionalgorithm::Operations::Needle
{

bool prunePointsUsingEdges(std::vector<BaseProximity::SPtr>& couplingPts,
                           const EdgeElement::SPtr& edge)
{
    if (!edge) 
    {
        msg_warning("Needle::PrunePointsAheadOfTip")
            << "Null element pointer in prunePointsUsingEdges; returning false";
        return false;
    }
    const type::Vec3 edgeBase(edge->getP0()->getPosition());
    const type::Vec3 tip(edge->getP1()->getPosition());
    const type::Vec3 edgeDirection = tip - edgeBase;

    const int initSize = couplingPts.size();

    while(!couplingPts.empty())
    {
        const type::Vec3 tip2Pt = couplingPts.back()->getPosition() - tip;

        // Negative dot product means the point is behind the tip
        if(dot(tip2Pt, edgeDirection) < 0_sreal) break;
        couplingPts.pop_back();
    }
    return (initSize == couplingPts.size());
}

int register_PrunePointsAheadOfTip_Edge =
    PrunePointsAheadOfTip::register_func<EdgeElement>(&prunePointsUsingEdges);
}  // namespace sofa::collisionalgorithm::Operations::Needle
