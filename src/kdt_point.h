#ifndef KDT_POINT_H
#define KDT_POINT_H

#include <vector>
#include <array>
#include <iostream>

namespace kdt {

template <typename T, typename U> struct VectorPointWithUserData 
    : public std::vector<T> {
    U userData;

    void WritePointToStream(std::ostream& stream,
                            unsigned int numDimensions) const {
        stream.write(reinterpret_cast<const char*>(std::vector<T>::data()),
                     numDimensions * sizeof(T));
        stream.write(reinterpret_cast<const char*>(&userData),
                     sizeof(userData));
        
    }

    void ReadPointFromStream(std::istream& stream, 
                             unsigned int numDimensions) {
        std::vector<T>::resize(numDimensions);
        stream.read(reinterpret_cast<char*>(std::vector<T>::data()),
                    numDimensions * sizeof(T));
        stream.read(reinterpret_cast<char*>(&userData),
                    sizeof(userData));
    }

    double Distance(const VectorPointWithUserData& other) const {
        double distanceSq = 0.0;
        for (unsigned i = 0; i < std::vector<T>::size(); i++) {
            double diff = (*this)[i] - other[i];
            distanceSq += diff * diff;
        }
        return distanceSq;
    }
};

template <typename T, std::size_t NumDimensions, typename U>
struct ArrayPointWithUserData
    : public std::array<T, NumDimensions> {
    U userData;

    void WritePointToStream(std::ostream& stream,
        unsigned int numDimensions) const {
        stream.write(reinterpret_cast<const char*>(std::vector<T>::data()),
            NumDimensions * sizeof(T));
        stream.write(reinterpret_cast<const char*>(&userData),
            sizeof(userData));

    }

    void ReadPointFromStream(std::istream& stream,
        unsigned int numDimensions) {
        stream.read(reinterpret_cast<char*>(std::vector<T>::data()),
            NumDimensions * sizeof(T));
        stream.read(reinterpret_cast<char*>(&userData),
            sizeof(userData));
    }

    double Distance(const ArrayPointWithUserData& other) const {
        double distanceSq = 0.0;
        for (unsigned i = 0; i < NumDimensions; i++) {
            double diff = (*this)[i] - other[i];
            distanceSq += diff * diff;
        }
        return distanceSq;
    }

    void resize(std::size_t n) {}
};

} //namespace kdt

#endif