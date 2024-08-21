#ifndef BOOST_SERIALIZATION_H
#define BOOST_SERIALIZATION_H

#include <tf2/LinearMath/Transform.h>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <fstream>
#include <iostream>

namespace boost {

namespace serialization {

// Serialization function for tf2::Vector3
template<class Archive>
void serialize(Archive &ar, tf2::Vector3 &v, const unsigned int version) {
    ar & v.m_floats[0];
    ar & v.m_floats[1];
    ar & v.m_floats[2];
}

// Save function for tf2::Matrix3x3
template<class Archive>
void save(Archive &ar, const tf2::Matrix3x3 &m, const unsigned int version) {
    tf2Scalar data[3][3];
    for (int i = 0; i < 3; ++i) {
        tf2::Vector3 row = m.getRow(i);
        data[i][0] = row.x();
        data[i][1] = row.y();
        data[i][2] = row.z();
    }
    ar & data[0][0] & data[0][1] & data[0][2];
    ar & data[1][0] & data[1][1] & data[1][2];
    ar & data[2][0] & data[2][1] & data[2][2];
}

// Load function for tf2::Matrix3x3
template<class Archive>
void load(Archive &ar, tf2::Matrix3x3 &m, const unsigned int version) {
    tf2Scalar data[3][3];
    ar & data[0][0] & data[0][1] & data[0][2];
    ar & data[1][0] & data[1][1] & data[1][2];
    ar & data[2][0] & data[2][1] & data[2][2];
    m.setValue(data[0][0], data[0][1], data[0][2],
               data[1][0], data[1][1], data[1][2],
               data[2][0], data[2][1], data[2][2]);
}

template<class Archive>
void serialize(Archive &ar, tf2::Matrix3x3 &m, const unsigned int version) {
    boost::serialization::split_free(ar, m, version);
}

// Save function for tf2::Transform
template<class Archive>
void save(Archive &ar, const tf2::Transform &t, const unsigned int version) {
    tf2::Vector3 origin = t.getOrigin();
    tf2::Matrix3x3 basis = t.getBasis();

    ar & origin;
    ar & basis;
}

// Load function for tf2::Transform
template<class Archive>
void load(Archive &ar, tf2::Transform &t, const unsigned int version) {
    tf2::Vector3 origin;
    tf2::Matrix3x3 basis;

    ar & origin;
    ar & basis;

    t.setOrigin(origin);
    t.setBasis(basis);
}

template<class Archive>
void serialize(Archive &ar, tf2::Transform &t, const unsigned int version) {
    boost::serialization::split_free(ar, t, version);
}

} // namespace serialization

} // namespace boost


#endif // BOOST_SERIALIZATION_H