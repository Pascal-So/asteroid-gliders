#ifndef SYSTEM_HPP
#define SYSTEM_HPP

// System as in solar system, but not really, because the masses are
// all stationary.

class System {
    std::array<point, 2> bounds;

    const float gravitational_constant = 2000.0;

    template<typename RNG>
    void populatePlanets(const int n, const float max_mass, RNG &rng) {
        planets.resize(n);
        std::uniform_real_distribution<float> dist_mass(0, max_mass);
        for(auto &p:planets)
            p = {randomPoint(bounds, rng), dist_mass(rng)};
    }

public:
    std::vector<std::pair<point, float>> planets;

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
        point out {0.0, 0.0};
        for (auto &p:planets) {
            const auto r = pos - p.first;
            out -= r.norm() / r.sqmag();
        }

        return out * gravitational_constant;
    }

    float probePotential(point const& pos) const {
        float out = 0.0;
        for (auto &p:planets) {
            const float mass = p.second;
            out -= mass / (pos - p.first).mag();
        }

        return out * gravitational_constant;
    }
};

#endif // SYSTEM_HPP
