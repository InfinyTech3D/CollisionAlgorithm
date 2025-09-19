#pragma once

#include <CollisionAlgorithm/config.h>
#include <CollisionAlgorithm/BaseAlgorithm.h>

namespace sofa::collisionalgorithm {

/*!
 * \brief The DistanceFilter class
 * accepts proximities which positions are within a limited distance from each other
 */
class SOFA_COLLISIONALGORITHM_API DistanceFilter : public BaseAlgorithm::BaseFilter {
public:
    SOFA_CLASS(DistanceFilter, BaseFilter);

    Data<double> d_distance;

    DistanceFilter()
     : d_distance(initData(&d_distance, std::numeric_limits<double>::max(), "distance", "Min distance")) {}

    bool accept(const BaseProximity::SPtr & p1,const BaseProximity::SPtr & p2) const {
        return (p1->getPosition()-p2->getPosition()).norm()<d_distance.getValue();
    }
};

}
