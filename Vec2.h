#ifndef VEC2_INCLUDED
#define VEC2_INCLUDED

#include <cmath>
#include <iostream>

class Vec2 {
private:
    double data[2];

public:
    // Constructors
    Vec2() { data[0] = data[1] = 0.0; }
    Vec2(double x, double y) { data[0] = x; data[1] = y; }
    Vec2(const Vec2& v) { data[0] = v.data[0]; data[1] = v.data[1]; }

    // Array access
    double& operator[](int i) { return data[i]; }
    const double& operator[](int i) const { return data[i]; }

    // Assignment
    Vec2& operator=(const Vec2& v) {
        data[0] = v.data[0];
        data[1] = v.data[1];
        return *this;
    }

    // Arithmetic operators
    Vec2 operator+(const Vec2& v) const {
        return Vec2(data[0] + v.data[0], data[1] + v.data[1]);
    }

    Vec2 operator-(const Vec2& v) const {
        return Vec2(data[0] - v.data[0], data[1] - v.data[1]);
    }

    Vec2 operator*(double s) const {
        return Vec2(data[0] * s, data[1] * s);
    }

    // Methods
    double length() const {
        return sqrt(data[0] * data[0] + data[1] * data[1]);
    }

    Vec2 clone() const {
        return Vec2(*this);
    }

    // Comparison operators
    bool operator==(const Vec2& v) const {
        return (fabs(data[0] - v.data[0]) < 1e-10 && fabs(data[1] - v.data[1]) < 1e-10);
    }

    bool operator!=(const Vec2& v) const {
        return !(*this == v);
    }
};

// Stream output operator
inline std::ostream& operator<<(std::ostream& os, const Vec2& v) {
    os << "(" << v[0] << ", " << v[1] << ")";
    return os;
}

#endif