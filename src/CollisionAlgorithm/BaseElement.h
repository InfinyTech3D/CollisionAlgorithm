#pragma once

#include <CollisionAlgorithm/BaseProximity.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa::collisionAlgorithm {

class PointElement;
class EdgeElement;
class TriangleElement;
class TetrahedronElement;

template<class ELMT>
class ElementContainer {
public:
    typedef typename std::shared_ptr<ELMT> SPtr;

    typedef typename std::vector<SPtr>::const_iterator const_iterator;

    void insert(SPtr e) {
        for (unsigned i=0;i<m_data.size();i++)
            if (e == m_data[i]) return;

        m_data.push_back(e);
    }

    inline SPtr & operator[](unsigned i) { return m_data[i]; }

    inline const SPtr & operator[](unsigned i) const { return m_data[i]; }

    inline void clear() { m_data.clear(); }

    inline unsigned size() const { return m_data.size(); }

    inline const_iterator cbegin() const { return m_data.cbegin(); }

    inline const_iterator cend() const { return m_data.cend(); }

    static inline const ElementContainer & empty() {
        static ElementContainer s_empty;
        return s_empty;
    }

private:
    std::vector<SPtr> m_data;
};

class BaseElement {
public:

    typedef std::shared_ptr<BaseElement> SPtr;

    virtual const ElementContainer<PointElement> & pointElements() const = 0;//{ return m_pointElements; }

    virtual const ElementContainer<EdgeElement> & edgeElements() const = 0;//{ return m_edgeElements; }

    virtual const ElementContainer<TriangleElement> & triangleElements() const = 0;//{ return m_triangleElements; }

    virtual const ElementContainer<TetrahedronElement> & tetrahedronElements() const = 0;//{ return m_tetrahedronElements; }

    virtual void draw(const core::visual::VisualParams * vparams) = 0;

    virtual const std::type_info& getTypeInfo() const = 0;

    virtual std::string name() const = 0;

    virtual void update() = 0;

    void setDirty(bool b) {m_isDirty = b;}

    bool isDirty() {return m_isDirty;}

protected:
    bool m_isDirty;

};


}
