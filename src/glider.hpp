#ifndef GLIDER_HPP
#define GLIDER_HPP

#include <limits>
#include <vector>
#include <Eigen/Dense>
#include "system.hpp"
#include "point.hpp"
#include "integrator.hpp"

point gliderStep(point const& start_pos, const float angular_potential_factor,
                       System const& system, const bool ccw) {

    auto gradient_func = [&](point const& pos){
        //
        //             Glider               Resulting Motion
        //   angle  <-- •              (for when ccw is true, flipped otherwise)   
        //   gradient   |                      __
        //              v gravity             |\                       .
        //                                      \                      .
        //              • Planet
        //
        //  We have two potentials in the space, the gravitational potential
        //  and the angular potential. These potentials are summed up, where
        //  the angular potential is first multiplied by a factor to control
        //  its infulence.
        //
        //  To get a path where the total potential remains zero, we add the
        //  gradients for both fields, and move in a direction perpendicular
        //  to the gradient for this total potential.

        const point gravity_potential_gradient = -system.probeGravity(pos);
        const point angular_gradient =
            angular_potential_factor * system.probeAngularPotentialGradient(pos);

        //  We need a minus sign here because I'm bad at maths..
        const point total_gradient = gravity_potential_gradient - angular_gradient;

        const point equipot_motion = point(-total_gradient.y, total_gradient.x).norm();
        return equipot_motion * (ccw ? 1 : -1);
    };

    const float stepsize = 10.f;
    return integrator::rungeKutta4(start_pos, gradient_func, stepsize);
}

std::vector<point> generateGliderTrajectory(point pos,
                                            System const& system,
                                            const float spiral_factor,
                                            const std::size_t max_steps) {
    const float sq_lower_dist_limit = 0.005f;
    const float sq_upper_dist_limit = 400.f;

    std::vector<point> points {pos};
    const bool ccw = rand()&1;

    for (std::size_t step = 0; step < max_steps; ++step) {
        const point last_pos = pos;
        pos = gliderStep(pos, spiral_factor, system, ccw);

        const float sq_last_dist = (pos - last_pos).sqmag();
        if (sq_last_dist > sq_upper_dist_limit ||
            sq_last_dist < sq_lower_dist_limit) {
            break;
        }

        points.push_back(pos);
    }

    return points;
}

float scorePath(System const& system, std::array<point, 2> const& bounds,
                std::vector<point> const& path) {
    float path_length = 0.f;
    unsigned planet_switches = 0;
    float penalty = 0.f;
    std::size_t current_closest_planet = 0;
    /*std::vector<point> centres;
      const float curve_check_interval = 50.f;
      const float last_curve_check = 0.f;
      std::size_t last_curve_check_index = 0;*/
    for (std::size_t i = 1; i < path.size(); ++i) {
        // Only count points inside bounds
        if (path[i].x < bounds[0].x || path[i].y < bounds[0].y ||
            path[i].x > bounds[1].x || path[i].y > bounds[1].y) {
            penalty += 3;
            continue;
        }

        path_length += (path[i] - path[i-1]).mag();

        float sq_lowest_dist = std::numeric_limits<float>::max();
        std::size_t new_closest_planet = 0;
        for (std::size_t planet_id = 0; planet_id < system.planets.size(); ++planet_id) {
            const point r = system.planets[planet_id].pos - path[i];
            if (r.sqmag() < sq_lowest_dist) {
                sq_lowest_dist = r.sqmag();
                new_closest_planet = planet_id;
            }
        }

        if (new_closest_planet != current_closest_planet) {
            const point r_current = system.planets[current_closest_planet].pos - path[i];
            const point r_new = system.planets[new_closest_planet].pos - path[i];
                
            // prevent frequent switches near a border
            if (r_new.sqmag() * 1.2f < r_current.sqmag()) {
                current_closest_planet = new_closest_planet;
                ++planet_switches;    
            }
        }

        const point r = system.planets[current_closest_planet].pos - path[i];
        if (r.sqmag() < 100.f) penalty += 500;

        /*
          if (path_length - last_curve_check > curve_check_interval &&
          last_curve_check_index < i - 2) {
                
          const point
          a = path[last_curve_check_index],
          b = path[(last_curve_check_index + i) / 2],
          c = path[i];

          // Find the centre of the circle described by the three
          // points a, b, c. Algorithm from
          // https://math.stackexchange.com/a/1460096
            
          Eigen::Matrix<float, 3, 4> data;
          data <<
              a.sqmag(), a.x, a.y, 1,
              b.sqmag(), b.x, b.y, 1,
              c.sqmag(), c.x, c.y, 1;

          Eigen::Matrix3f m = data.block(0, 1, 3, 3);
          const float m11 = m.determinant();
          if (fabs(m11) < 0.001) {
              // points are roughly colinear
              continue;
          }
            
          m.col(0) = data.col(0);
          const float m12 = m.determinant();
          m.col(1) = data.col(1);
          const float m13 = m.determinant();

          point center (m12/m11, m13/m11);
          center /= 2;

          //last_curve_check = path_length;
          last_curve_check_index = i;
          }
        */
    }

    const float score = 0 * path_length + planet_switches * 100.f - penalty;

    return score;
}

point findNicePath(System const& system,
                   const float spiral_factor, const std::size_t max_steps,
                   std::array<point,2> const& bounds, const int seed) {
    // Offset seed to avoid collision with generated planets
    std::mt19937 rng(seed + 2000);

    float candidate_score = std::numeric_limits<float>::min();
    point candidate;

    const int max_attempts = 1000;

    for (int i = 0; i < max_attempts; ++i) {
        const point p = point::randomPoint(bounds, rng);

        const auto trajectory = generateGliderTrajectory(p, system, spiral_factor, max_steps);

        const float score = scorePath(system, bounds, trajectory);
        
        if (score >= candidate_score) {
            candidate_score = score;
            candidate = p;
        }
    }

    std::cout << "found path with score " << candidate_score << '\n';
    
    return candidate;
}

#endif // GLIDER_HPP
