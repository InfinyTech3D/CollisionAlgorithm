#pragma once

#include <CollisionAlgorithm/BaseProximity.h>
#include <CollisionAlgorithm/BaseElement.h>

namespace sofa::collisionalgorithm {

class BaseGeometry;

class ElementIterator {
public:

    typedef std::shared_ptr<ElementIterator> SPtr;

    //This is necessary to have the correct behavior with the delete !
    virtual ~ElementIterator() = default;

    virtual bool end() const = 0;

    virtual void next() = 0;

    virtual BaseElement::SPtr element() = 0;

    virtual const BaseElement::SPtr element() const = 0;

    virtual const std::type_info& getTypeInfo() const = 0;

    ///returns a new EmptyIterator
    static inline ElementIterator::SPtr empty();

};

static inline bool operator != (ElementIterator::SPtr & it, const BaseGeometry * /*geo*/) {
    return ! it->end();
}

static inline void operator ++ (ElementIterator::SPtr & it) {
    return it->next();
}

static inline void operator ++ (ElementIterator::SPtr & it, int /*NB*/) {
//    for (int i=0;i<NB;i++)
        it->next();
}

class EmptyIterator : public ElementIterator {
public:
    friend class ElementIterator;

    bool end() const override { return true; }

    void next() override {}

    BaseElement::SPtr element() override { return m_empty_element; }

    const BaseElement::SPtr element() const override { return m_empty_element; }

    const std::type_info& getTypeInfo() const override {
        return typeid(EmptyIterator);
    }

    ///returns a new EmptyIterator
    static inline ElementIterator::SPtr get() {
        static EmptyIterator::SPtr s_empty = NULL;
        if (s_empty==NULL) s_empty=ElementIterator::SPtr(new EmptyIterator());
        return s_empty;
    }

private:
    EmptyIterator() : m_empty_element(NULL) {}

    BaseElement::SPtr m_empty_element;
};


template<class CONTAINER>
class TDefaultElementIterator : public ElementIterator {
public:
    void next() override { m_it++; }

    bool end() const override { return m_it==m_end; }

    const std::type_info& getTypeInfo() const override {
        if (m_it == m_end) return typeid(EmptyIterator);
        else return (*m_it)->getTypeInfo();
    }

    inline BaseElement::SPtr element() override { return *m_it; }

    inline const BaseElement::SPtr element() const override { return *m_it; }

protected:
    typename CONTAINER::const_iterator m_it;
    typename CONTAINER::const_iterator m_end;
};


template<class CONTAINER>
class TDefaultElementIterator_copy : public TDefaultElementIterator<CONTAINER> {
public:
    TDefaultElementIterator_copy(const CONTAINER & c,int id=0)
    : m_container(c) {
        this->m_it = m_container.cbegin();
        this->m_end = m_container.cend();

        while (id>0 && ! this->end()) {
            this->next();
            id--;
        }
    }

private:
    const CONTAINER m_container;
};

template<class CONTAINER>
class TDefaultElementIterator_ref : public TDefaultElementIterator<CONTAINER> {
public:
    TDefaultElementIterator_ref(const CONTAINER & c,int id=0) {
        this->m_it = c.cbegin();
        this->m_end = c.cend();

        while (id>0 && ! this->end()) {
            this->next();
            id--;
        }
    }
};

ElementIterator::SPtr ElementIterator::empty(){
    return EmptyIterator::get();
}

}
