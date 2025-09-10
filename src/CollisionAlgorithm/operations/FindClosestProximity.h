#pragma once

#include <CollisionAlgorithm/BaseProximity.h>
#include <CollisionAlgorithm/BaseOperation.h>
#include <limits.h>
#include <CollisionAlgorithm/operations/Project.h>

namespace sofa::collisionAlgorithm::Operations::FindClosestProximity {

typedef BaseProximity::SPtr Result;
typedef std::function<bool(const BaseProximity::SPtr&,const BaseProximity::SPtr&)> FilterFUNC;

//Specific operation to find the closest point on a geometry (the code is in the c++ class)
class Operation : public Operations::GenericOperation<Operation,//operation type
                                                      Result, // default return
                                                      const BaseProximity::SPtr &,BaseGeometry *, Operations::Project::FUNC, FilterFUNC // parameters
                                                      > {
public:

    static ElementIterator::SPtr broadPhaseIterator(BaseProximity::SPtr prox, BaseGeometry::BroadPhase * broadphase) {
        //old params : type::Vec3i cbox, std::set<BaseProximity::Index> & selectElements, int d
        type::Vec3i nbox = broadphase->getNbox();

        //take the first broad phase...
        type::Vec3 P = prox->getPosition();

        type::Vec3i cbox = broadphase->getBoxCoord(P);

        //project the box in the bounding box of the object
        //search with the closest box in bbox
        cbox[0] = std::max(0,std::min(nbox[0],cbox[0]));
        cbox[1] = std::max(0,std::min(nbox[1],cbox[1]));
        cbox[2] = std::max(0,std::min(nbox[2],cbox[2]));

    //        int max = 0;
    //        for (int i = 0 ; i < 3 ; i++) {
    //            max = std::max (max, cbox[i]) ;
    //            max = std::max (max, nbox[i]-cbox[i]) ;
    //        }

        std::set<BaseElement::SPtr> selectedElements;

        for (int d=0; selectedElements.empty();d++) {
            {
                int i=-d;
                if (cbox[0]+i >= 0 && cbox[0]+i <= nbox[0])
                {
                    for (int j=-d; j <= d; j++)
                    {
                        if (cbox[1]+j < 0 || cbox[1]+j > nbox[1])
                            continue;

                        for (int k=-d;k<=d;k++)
                        {
                            if (cbox[2]+k < 0 || cbox[2]+k > nbox[2])
                                continue;


                            const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                            selectedElements.insert(elmts.cbegin(),elmts.cend());
                        }
                    }
                }
            }

            //No need to process all the box for d==0 since they all point on the same box
            if (d==0) continue;

            {
                int i=d;
                if (cbox[0]+i >= 0 && cbox[0]+i <= nbox[0])
                {
                    for (int j=-d;j<=d;j++)
                    {
                        if (cbox[1]+j < 0 || cbox[1]+j > nbox[1])
                            continue;

                        for (int k=-d;k<=d;k++)
                        {
                            if (cbox[2]+k < 0 || cbox[2]+k > nbox[2])
                                continue;

                            const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                            selectedElements.insert(elmts.cbegin(),elmts.cend());
                        }
                    }
                }
            }


            {
                int j=-d;
                if (cbox[1]+j >= 0 && cbox[1]+j <= nbox[1])
                {
                    for (int i=-d+1;i<d;i++)
                    {
                        if (cbox[0]+i < 0 || cbox[0]+i > nbox[0])
                            continue;

                        for (int k=-d;k<=d;k++)
                        {
                            if (cbox[2]+k < 0 || cbox[2]+k > nbox[2])
                                continue;

                            const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                            selectedElements.insert(elmts.cbegin(),elmts.cend());
                        }
                    }
                }
            }

            {
                int j=d;
                if (cbox[1]+j >= 0 && cbox[1]+j <= nbox[1])
                {
                    for (int i=-d+1;i<d;i++)
                    {
                        if (cbox[0]+i < 0 || cbox[0]+i > nbox[0])
                            continue;

                        for (int k=-d;k<=d;k++)
                        {
                            if (cbox[2]+k < 0 || cbox[2]+k > nbox[2])
                                continue;

                            const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                            selectedElements.insert(elmts.cbegin(),elmts.cend());
                        }
                    }
                }
            }

            {
                int k=-d;
                if (cbox[2]+k >= 0 && cbox[2]+k <= nbox[2])
                {
                    for (int i=-d+1;i<d;i++)
                    {
                        if (cbox[0]+i < 0 || cbox[0]+i > nbox[0])
                            continue;

                        for (int j=-d+1;j<d;j++)
                        {
                            if (cbox[1]+j < 0 || cbox[1]+j > nbox[1])
                                continue;

                            const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                            selectedElements.insert(elmts.cbegin(),elmts.cend());
                        }
                    }
                }
            }

            {
                int k=d;
                if (cbox[2]+k >= 0 && cbox[2]+k <= nbox[2])
                {
                    for (int i=-d+1;i<d;i++)
                    {
                        if (cbox[0]+i < 0 || cbox[0]+i > nbox[0])
                            continue;

                        for (int j=-d+1;j<d;j++)
                        {
                            if (cbox[1]+j < 0 || cbox[1]+j > nbox[1])
                                continue;

                            const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                            selectedElements.insert(elmts.cbegin(),elmts.cend());
                        }
                    }
                }
            }
        }

        return ElementIterator::SPtr(new TDefaultElementIterator_copy(selectedElements));
    }

    BaseProximity::SPtr defaultFunc(const BaseProximity::SPtr & prox, BaseGeometry * geometry, Operations::Project::FUNC projectOp,BaseAlgorithm::FilterFUNC filter) const override {
        const ElementIterator::SPtr & itdest = (geometry->getBroadPhase()) ?
                                               broadPhaseIterator(prox, geometry->getBroadPhase()) :
                                               geometry->begin();

        return doFindClosesPoint(prox, itdest, projectOp, filter);
    }

private:
    BaseProximity::SPtr doFindClosesPoint(const BaseProximity::SPtr & prox, ElementIterator::SPtr itdest, Operations::Project::FUNC projectOp,BaseAlgorithm::FilterFUNC filter) const {
        double min_dist = std::numeric_limits<double>::max();
        BaseProximity::SPtr res = NULL;

//        type::Vec3 P = prox->getPosition();

        for (; ! itdest->end();itdest++) {
            auto edest = itdest->element();
            if (edest == nullptr) continue;

            Operations::Project::Result result = projectOp(prox->getPosition(),edest);

            if (result.prox == NULL) continue;

            if (result.distance< min_dist) {
				auto normalizedProx = result.prox->copy();
				normalizedProx->normalize();

				if (! filter(prox,normalizedProx)) continue;


				res = result.prox;
                min_dist = result.distance;
            }
        }

        return res;
    }
};

typedef Operation::FUNC FUNC;

}

