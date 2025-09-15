#pragma once

#include <sofa/helper/AdvancedTimer.h>
#include <CollisionAlgorithm/BaseAABBBroadPhase.h>
#include <CollisionAlgorithm/BaseAlgorithm.h>
#include <CollisionAlgorithm/BaseElement.h>

namespace sofa::collisionalgorithm {

class FullAABBBroadPhase : public BaseAABBBroadPhase {
public:

    SOFA_CLASS(FullAABBBroadPhase,BaseAABBBroadPhase);


    FullAABBBroadPhase() {}

    void updateData() override {}

    const std::set<BaseElement::SPtr> & getElementSet(unsigned i, unsigned j, unsigned k) const override {
        if (m_indexedElement[i][j][k].size() == 0) {
            static std::set<BaseElement::SPtr> empty;
            return empty;
        } else {
            return m_indexedElement[i][j][k];
        }
    }

    void newContainer() override {
        m_indexedElement.clear();

        m_indexedElement.resize(m_nbox[0]);
        for (int i=0; i<m_nbox[0]; i++) {
            m_indexedElement[i].resize(m_nbox[1]);
            for (int j=0; j<m_nbox[1]; j++) {
                m_indexedElement[i][j].resize(m_nbox[2]);
            }
        }
//        std::cout << "test size vector : " << m_indexedElement.size() << " " <<  m_indexedElement[0].size() << " " << m_indexedElement[0][0].size() << " " << std::endl;
    }

    void addElement(int i, int j, int k, BaseElement::SPtr elmt) override {
        m_indexedElement[i][j][k].insert(elmt);
    }


protected:
    std::vector<std::vector<std::vector< std::set<BaseElement::SPtr> >>> m_indexedElement;
    type::Vec<2, size_t> m_offset;
};


}
