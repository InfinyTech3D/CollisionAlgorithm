#pragma once

#include <CollisionAlgorithm/CollisionPipeline.h>
#include <CollisionAlgorithm/BaseElement.h>
#include <CollisionAlgorithm/BaseProximity.h>
#include <CollisionAlgorithm/ElementIterator.h>
#include <sofa/gl/gl.h>
#include <sofa/helper/AdvancedTimer.h>
#include <CollisionAlgorithm/elements/PointElement.h>
#include <CollisionAlgorithm/elements/EdgeElement.h>
#include <CollisionAlgorithm/elements/TriangleElement.h>
#include <CollisionAlgorithm/elements/TetrahedronElement.h>

namespace sofa ::collisionAlgorithm {


class InternalDataContainer {
public:


    class InternalData {
    public:
        InternalData() : m_dirty(true) {};
        virtual ~InternalData() = default;

		virtual void setDirty( bool _dirty) { m_dirty = _dirty; };
		virtual bool isDirty() { return m_dirty; };

	private:
		bool m_dirty;
    };

    InternalDataContainer() = default;


    template<class KEY,class CLASS = KEY>
    inline CLASS * get() {
        size_t hash = typeid(KEY).hash_code();
        auto it = m_internalData.find(hash);
        if (it==m_internalData.cend()) return NULL;
        return reinterpret_cast<CLASS*>(it->second.get());
    }

    template<class T>
    inline void set(InternalData * ptr) {
        size_t hash = typeid (T).hash_code();
        m_internalData[hash] = std::shared_ptr<InternalData>(ptr);
    }

    template<class T>
    inline void clear() {
        size_t hash = typeid (T).hash_code();
        m_internalData.erase(hash);
    }

    template<class KEY,class CLASS = KEY, class... ARGS>
    inline CLASS * get_or_create(ARGS... args) {
        size_t hash = typeid (KEY).hash_code();

        auto it = m_internalData.find(hash);
        if (it==m_internalData.cend()) {
            CLASS * ptr = new CLASS(args...);

            m_internalData[hash] = std::shared_ptr<InternalData>(ptr);

            return ptr;


        }

        return reinterpret_cast<CLASS*>(it->second.get());
    }



    unsigned size() const { return m_internalData.size(); }

    std::map<size_t,std::shared_ptr<InternalData> >::const_iterator cbegin() const { return m_internalData.cbegin(); }

    std::map<size_t,std::shared_ptr<InternalData> >::const_iterator cend() const { return m_internalData.cend(); }

private:
    std::map<size_t,std::shared_ptr<InternalData> > m_internalData;
};



}

