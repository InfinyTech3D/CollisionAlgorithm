#pragma once

#include <CollisionAlgorithm/BaseGeometry.h>
#include <sofa/helper/NameDecoder.h>
#include <memory>

namespace sofa::collisionalgorithm::Operations {

template<class A,class B>
class REAL_TYPE_CHECK {
public:
    static inline void doCheck() {
        using A_type = std::remove_pointer_t<A>;
        using B_type = std::remove_pointer_t<B>;

        static_assert(( std::is_base_of<A_type, B_type>::value), "Invalid type in registered function");
    }
};

//Specialization for shared pointer
template<class A,class B>
class REAL_TYPE_CHECK<typename std::shared_ptr<A>, typename std::shared_ptr<B>> {
public:
    static inline void doCheck() {
        using A_type = std::remove_pointer_t<A>;
        using B_type = std::remove_pointer_t<B>;

        static_assert((std::is_base_of<A_type, B_type>::value), "Invalid type in registered function");
    }
};

template<class A,class B>
class REAL_TYPE_CHECK<typename std::shared_ptr<A> &, typename std::shared_ptr<B> &> {
public:
    static inline void doCheck() {
        using A_type = std::remove_pointer_t<A>;
        using B_type = std::remove_pointer_t<B>;

        static_assert((std::is_base_of<A_type, B_type>::value), "Invalid type in registered function");
    }
};

template<class A,class B>
class REAL_TYPE_CHECK<const typename std::shared_ptr<A> &,const  typename std::shared_ptr<B> &> {
public:
    static inline void doCheck() {
        using A_type = std::remove_pointer_t<A>;
        using B_type = std::remove_pointer_t<B>;

        static_assert((std::is_base_of<A_type, B_type>::value), "Invalid type in registered function");
    }
};


template <typename First_A, typename... A>
struct CAST_PARAMS {
    template <typename First_B, typename... B>
    struct AS {
        static void doCheck() { CAST_PARAMS<A...>::template AS<B...>::doCheck(); }
    };
};

template <typename A>
struct CAST_PARAMS<A> {
    template <typename B>
    struct AS {
        static void doCheck() { REAL_TYPE_CHECK<A,B>::doCheck(); }
    };
};

template<typename CHILD, class RETURN_TYPE, class... PARAMS>
class GenericOperation {
public:
    friend CHILD;

    typedef RETURN_TYPE (*FUNC)(PARAMS...);
    typedef size_t KEY_MAP; // <RETURN,TYPE>

    template<class T>
    static inline RETURN_TYPE (*get(T obj)) (PARAMS...) {
        return get(obj->getTypeInfo());
    }

    static inline RETURN_TYPE (*get(const std::type_info & id)) (PARAMS...) {
        typedef RETURN_TYPE (*function_type)(PARAMS...);

        KEY_MAP key = id.hash_code();

        //size_t id = itelmt->getTypeInfo();
        auto it_type = getSingleton()->m_map.find(key);
        if (it_type == getSingleton()->m_map.cend()) {
            getSingleton()->notFound(id);
            return &GenericOperation::t_defaultFunc;
        }

        return *((function_type*) &it_type->second);
    }

    template<class ELMT, class... PARAMS_TYPED>
    static int register_func(RETURN_TYPE (*f)(PARAMS_TYPED...)) {
        KEY_MAP key = typeid(ELMT).hash_code();

        CAST_PARAMS<PARAMS...>::template AS<PARAMS_TYPED...>::doCheck();

        auto it = getSingleton()->m_map.find(key);
        if (it != getSingleton()->m_map.cend()) {
            std::cerr << sofa::helper::NameDecoder::decodeFullName(typeid(CHILD)) << " with operation " << sofa::helper::NameDecoder::decodeFullName(typeid(ELMT)) << " already in the factory" << std::endl;
            return 1;
        }

        getSingleton()->m_map[key] = (void*) f;

        return 0;
    }

    virtual RETURN_TYPE defaultFunc(PARAMS...) const = 0;

    virtual void notFound(const std::type_info & ) const {}

private:
    std::map<KEY_MAP,void*> m_map;

    static GenericOperation * getSingleton() {
        static GenericOperation * m_singleton = NULL;
        if (m_singleton == NULL) m_singleton = new CHILD();
        return m_singleton;
    }

    static inline RETURN_TYPE t_defaultFunc(PARAMS... params) {
        return getSingleton()->defaultFunc(params...);
    }

    //No construction of the is is allowed
    GenericOperation() {}
};



template<typename CHILD, class RETURN_TYPE, class... PARAMS>
class GenericOperation2 {
public:
    friend CHILD;

    typedef RETURN_TYPE (*FUNC)(PARAMS...);
    typedef std::pair<size_t,size_t> KEY_MAP; // <RETURN,TYPE1,TYPE2>

    template<class T1,class T2>
    static inline RETURN_TYPE (*get(T1 obj1,T2 obj2)) (PARAMS...) {
        return get(obj1->getTypeInfo(),obj2->getTypeInfo());
    }

    static inline RETURN_TYPE (*get(const std::type_info & id1, const std::type_info & id2)) (PARAMS...) {
        typedef RETURN_TYPE (*function_type)(PARAMS...);

        KEY_MAP key(id1.hash_code(),id2.hash_code());

        //size_t id = itelmt->getTypeInfo();
        auto it = getSingleton()->m_map.find(key);
        if (it == getSingleton()->m_map.cend()) {
            getSingleton()->notFound(id1,id2);
            return &GenericOperation2::t_defaultFunc;
        }

        return *((function_type*) &it->second);
    }

    template<class ELMT1,class ELMT2, class... PARAMS_TYPED>
    static int register_func(RETURN_TYPE (*f)(PARAMS_TYPED...)) {
        KEY_MAP key(typeid(ELMT1).hash_code(),typeid(ELMT2).hash_code());

        CAST_PARAMS<PARAMS...>::template AS<PARAMS_TYPED...>::doCheck();

        auto it = getSingleton()->m_map.find(key);
        if (it != getSingleton()->m_map.cend()) {
            std::cerr << sofa::helper::NameDecoder::decodeFullName(typeid(CHILD)) << " with operation2 " << sofa::helper::NameDecoder::decodeFullName(typeid(ELMT1)) << "," << sofa::helper::NameDecoder::decodeFullName(typeid(ELMT2)) << " already in the factory" << std::endl;
            return 1;
        }

        getSingleton()->m_map[key] = (void*) f;

        return 0;
    }

    virtual RETURN_TYPE defaultFunc(PARAMS...) const = 0;

    virtual void notFound(const std::type_info & id1,const std::type_info & id2) const = 0;

private:
    std::map<KEY_MAP,void*> m_map;

    static GenericOperation2 * getSingleton() {
        static GenericOperation2 * m_singleton = NULL;
        if (m_singleton == NULL) m_singleton = new CHILD();
        return m_singleton;
    }

    static inline RETURN_TYPE t_defaultFunc(PARAMS... params) {
        return getSingleton()->defaultFunc(params...);
    }

    //No construction of the is is allowed
    GenericOperation2() {}
};

}
