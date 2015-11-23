#ifndef KDT_POINT_H
#define KDT_POINT_H

#include <vector>
#include <iostream>

namespace kdt {

template <typename T, typename U> struct VectorPointWithUserData 
    : public std::vector<T> {
    U userData;

    void WritePointToStream(std::ostream& stream,
                            unsigned int numDimensions) const {
        stream.write(reinterpret_cast<const char*>(data()), 
                     numDimensions * (sizeof T));
        stream.write(reinterpret_cast<const char*>(&userData),
                     sizeof userData);
        
    }

    void ReadPointFromStream(std::istream& stream, 
                             unsigned int numDimensions) {
        resize(numDimensions);
        stream.read(reinterpret_cast<char*>(data()), 
                    numDimensions * (sizeof T));
        stream.read(reinterpret_cast<char*>(&userData),
                    sizeof userData);
    }

    double Distance(const VectorPointWithUserData& other) const {
        double distanceSq = 0.0;
        for (unsigned i = 0; i < size(); i++) {
            double diff = (*this)[i] - other[i];
            distanceSq += diff * diff;
        }
        return distanceSq;
    }
};

} //namespace kdt

#endif