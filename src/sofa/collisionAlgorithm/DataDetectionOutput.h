#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class FIRST = BaseProximity,class SECOND = BaseProximity>
class DetectionOutput {
public:
    typedef std::pair<typename FIRST::SPtr,typename SECOND::SPtr> PairDetection;

    friend std::ostream& operator<<(std::ostream& os, const DetectionOutput& t)  {
        os << t.m_output.size() << ":" ;
        for (unsigned i=0;i<t.m_output.size();i++) {
            os << "[" << t.m_output[i].first->getPosition() << " | " << t.m_output[i].second->getPosition() << "] ";
        }
        return os;
    }

    friend std::istream& operator>>(std::istream& i, DetectionOutput& /*t*/) {
        return i;
    }

    void clear() {
        m_output.clear();
    }

    typename sofa::type::vector<PairDetection>::const_iterator begin() const {
        return m_output.begin();
    }

    typename sofa::type::vector<PairDetection>::const_iterator end() const {
        return m_output.end();
    }

    unsigned size() const {
        return m_output.size();
    }

    inline void add(const typename FIRST::SPtr & p1,const typename SECOND::SPtr & p2) {
        if (p1 == NULL) return;
        if (p2 == NULL) return;
        m_output.push_back(PairDetection(p1,p2));
    }

    inline const PairDetection & operator[](int i) const {
        return m_output[i];
    }

    inline const PairDetection & back() const {
        return m_output.back();
    }

protected:
    sofa::type::vector< PairDetection > m_output;
};

}

/// Specific class to activate copy and write on a DetectionOutput
namespace defaulttype {

template<class TDataType>
struct DetectionOutputTypeInfo
{
    typedef TDataType DataType;
    typedef DataType BaseType;
    typedef DataType ValueType;
    typedef long long ConvType;
    typedef DetectionOutputTypeInfo<TDataType> BaseTypeInfo;
    typedef DetectionOutputTypeInfo<TDataType> ValueTypeInfo;

    enum { ValidInfo       = 1                             }; ///< 1 if this type has valid infos
    enum { FixedSize       = 1                             }; ///< 1 if this type has a fixed size  -> always 1 Image
    enum { ZeroConstructor = 0                             }; ///< 1 if the constructor is equivalent to setting memory to 0  -> I guess so, a default Image is initialzed with nothing
    enum { SimpleCopy      = 0                             }; ///< 1 if copying the data can be done with a memcpy
    enum { SimpleLayout    = 0                             }; ///< 1 if the layout in memory is simply N values of the same base type
    enum { Integer         = 0                             }; ///< 1 if this type uses integer values
    enum { Scalar          = 0                             }; ///< 1 if this type uses scalar values
    enum { Text            = 0                             }; ///< 1 if this type uses text values
    enum { CopyOnWrite     = 1                             }; ///< 1 if this type uses copy-on-write -> it seems to be THE important option not to perform too many copies
    enum { Container       = 0                             }; ///< 1 if this type is a container

    enum { Size = 1 }; ///< largest known fixed size for this type, as returned by size()

    static size_t size() { return 1; }
    static size_t byteSize() { return 1; }

    static size_t size(const DataType& /*data*/) { return 1; }

    static bool setSize(DataType& /*data*/, size_t /*size*/) { return false; }

    template <typename T>
    static void getValue(const DataType &/*data*/, size_t /*index*/, T& /*value*/)
    {
        return;
    }

    template<typename T>
    static void setValue(DataType &/*data*/, size_t /*index*/, const T& /*value*/ )
    {
        return;
    }

    static void getValueString(const DataType &data, size_t index, std::string& value)
    {
        if (index != 0) return;
        std::ostringstream o; o << data; value = o.str();
    }

    static void setValueString(DataType &data, size_t index, const std::string& value )
    {
        if (index != 0) return;
        std::istringstream i(value); i >> data;
    }

    static const void* getValuePtr(const DataType&)
    {
        return NULL;
    }

    static void* getValuePtr(DataType&)
    {
        return NULL;
    }
};

template<>
struct DataTypeInfo< collisionAlgorithm::DetectionOutput<collisionAlgorithm::BaseProximity::SPtr, collisionAlgorithm::BaseProximity::SPtr> > : public DetectionOutputTypeInfo< collisionAlgorithm::DetectionOutput<collisionAlgorithm::BaseProximity::SPtr, collisionAlgorithm::BaseProximity::SPtr> >
{
    static std::string name() { std::ostringstream o; o << "DetectionOutput"; return o.str(); }
};


} // namespace defaulttype


} // namespace sofa

