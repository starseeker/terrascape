#ifndef VEC3_INCLUDED
#define VEC3_INCLUDED

#include <cmath>

class Vec3 {
private:
    double data[3];

public:
    // Constructors
    Vec3() { data[0] = data[1] = data[2] = 0.0; }
    Vec3(double x, double y, double z) { data[0] = x; data[1] = y; data[2] = z; }
    Vec3(const Vec2& v, double z) { data[0] = v[0]; data[1] = v[1]; data[2] = z; }
    Vec3(const Vec3& v) { data[0] = v.data[0]; data[1] = v.data[1]; data[2] = v.data[2]; }

    // Array access
    double& operator[](int i) { return data[i]; }
    const double& operator[](int i) const { return data[i]; }

    // Assignment
    Vec3& operator=(const Vec3& v) {
        data[0] = v.data[0];
        data[1] = v.data[1];
        data[2] = v.data[2];
        return *this;
    }

    // Arithmetic operators
    Vec3 operator+(const Vec3& v) const {
        return Vec3(data[0] + v.data[0], data[1] + v.data[1], data[2] + v.data[2]);
    }

    Vec3 operator-(const Vec3& v) const {
        return Vec3(data[0] - v.data[0], data[1] - v.data[1], data[2] - v.data[2]);
    }

    Vec3 operator*(double s) const {
        return Vec3(data[0] * s, data[1] * s, data[2] * s);
    }

    // Methods
    double length() const {
        return sqrt(data[0] * data[0] + data[1] * data[1] + data[2] * data[2]);
    }

    Vec3 clone() const {
        return Vec3(*this);
    }
};

#endif