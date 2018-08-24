#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include <array>
#include "point.hpp"

// System as in solar system, but not really, because the masses are
// all stationary.

struct Planet {
    point pos;
    float mass;
    bool ccw;
};

class System {
    std::array<point, 2> bounds;

    const float gravitational_constant = 2000.0;

    template<typename RNG>
    void populatePlanets(const int n, const float max_mass, RNG &rng) {
        planets.resize(n);
        std::uniform_real_distribution<float> dist_mass(0, max_mass);
        std::bernoulli_distribution dist_ccw(0.5);
        for(auto &p:planets){
            const bool ccw = dist_ccw(rng);
            p = {point::randomPoint(bounds, rng), dist_mass(rng), ccw};
        }
    }

public:
    std::vector<Planet> planets;

    template<typename RNG>
    System(const int n, std::array<point, 2> const& bounds, RNG &rng) : bounds(bounds) {
        populatePlanets(n, 1.0, rng);
    }

    System& operator=(System const& other) {
        bounds = other.bounds;
        planets = other.planets;
        return *this;
    }

    System& operator=(System&& other) {
        bounds = std::move(other.bounds);
        planets = std::move(other.planets);
        return *this;
    }

    point probeGravity(point const& pos) const {
        point out {0.f, 0.f};
        for (auto &p:planets) {
            const point r = pos - p.pos;
            out -= r.norm() / r.sqmag() * p.mass;
        }

        return out * gravitational_constant;
    }

    float probePotential(point const& pos) const {
        float out = 0.f;
        for (auto &p:planets) {
            out -= p.mass / (pos - p.pos).mag();
        }

        return out * gravitational_constant;
    }

    point probeAngularPotentialGradient(point const& pos) const {
        point out (0.f, 0.f);

        for (auto const& p : planets) {
            const point r = pos - p.pos;
            const float planet_contribution =
                p.mass * (p.ccw ? 1 : -1) / r.sqmag();

            out += point(r.y, -r.x) * planet_contribution;
        }

        return out;
    }

    float probeWeightedAngleDiff(point const& a, point const& b) const {
        float weighted_angle_diff = 0.f;
        for (auto const& p : planets) {
            const point r_a = a - p.pos;
            const point r_b = b - p.pos;

            const float angle_a = atan2(r_a.y, r_a.x);
            const float angle_b = atan2(r_b.y, r_b.x);
            float diff = angle_b - angle_a;

            if (diff > M_PI) {
                diff -= 2*M_PI;
            } else if (diff < -M_PI) {
                diff += 2*M_PI;
            }
            
            weighted_angle_diff += diff * p.mass * (p.ccw ? 1 : -1);
        }

        return weighted_angle_diff;
    }
};

#endif // SYSTEM_HPP
