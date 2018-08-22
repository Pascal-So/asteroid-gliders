#ifndef POINT_HPP
#define POINT_HPP

#include <iostream>
#include <ostream>
#include <cmath>

struct point {
    float x;
    float y;

    point() : x(0), y(0) {}

    point(float x, float y) : x(x), y(y) {}

    point(const point& o) : x(o.x), y(o.y) {}

    point operator+(const point& o) const { return point(x + o.x, y + o.y); }

    point operator-(const point& o) const { return point(x - o.x, y - o.y); }

    inline friend point operator*(const point& p, float f) {
        return point(p.x * f, p.y * f);
    }

    inline friend point operator*(float f, const point& p) {
        return point(p.x * f, p.y * f);
    }

    point operator/(const float f) const { return point(x/f, y/f); }

    float sqmag() const {
        return x*x + y*y;
    }

    float mag() const {
        return sqrt(sqmag());
    }

    float arg() const {
        return atan2(y, x);
    }

    point norm() const {
        return (*this) / mag();
    }
    

    point& operator+=(const point& o) {
        x += o.x;
        y += o.y;

        return *this;
    }

    point& operator-=(const point& o) {
        x -= o.x;
        y -= o.y;

        return *this;
    }

    point& operator*=(float f) {
        x *= f;
        y *= f;
        return *this;
    }

    bool operator<(const point& o) const {
        return std::tie(x, y) < std::tie(o.x, o.y);
    }

    bool operator==(const point& o) const { return x == o.x && y == o.y; }

    bool operator!=(const point& o) const { return !((*this) == o); }
};

inline std::ostream& operator<<(std::ostream& out, const point& p) {
    return out << "(" << p.x << ", " << p.y << ")";
}

point randomPoint(std::array<point,2> const& bounds, std::mt19937 &rng) {
    std::uniform_real_distribution<float> dist_x(bounds[0].x, bounds[1].x);
    std::uniform_real_distribution<float> dist_y(bounds[0].y, bounds[1].y);

    return point(dist_x(rng), dist_y(rng));
}

#endif // POINT_HPP
