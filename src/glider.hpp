#ifndef GLIDER_HPP
#define GLIDER_HPP

#include <limits>
#include <vector>
#include <Eigen/Dense>
#include "system.hpp"
#include "point.hpp"

point gliderStep(point const& pos, const float desired_potential, System const& system, const bool ccw) {
    const float stepsize = 6.f;

    // First calculate the new position by integrating with the midpoint method.

    point gravity = system.probeGravity(pos).norm();
    point motion_half (gravity.y, -gravity.x);
    motion_half *= stepsize / 2;

    gravity = system.probeGravity(pos + motion_half).norm();
    point motion (gravity.y, -gravity.x);
    if (ccw) {
        motion *= -1;
    }
    motion *= stepsize;

    const point new_pos_initial = pos + motion;

    // Uncomment this to only use explicit euler / midpoint method.
    // return pos + motion_half * 2.f; // Euler
    // return new_pos_initial;         // Midpoint


    // Then we correct the difference to the desired gravitational potential.
    
    const float new_potential = system.probePotential(new_pos_initial);
    const float diff = new_potential - desired_potential;
    
    // Here we assume the slope to be linear, because we're only correcting in a small
    // neighbourhood. We can thus use the fact that the gravitational force vector is
    // minus the gradient of the potential, so we have: (where '·' denotes the scalar
    // product)
    //
    //      delta potential = offset · -gravity
    //                      = ± |offset| * |gravity|   // We're only moving along the
    //                                                 // steepest slope, so the offset
    //                                                 // and gravity are colinear.
    //
    //      |offset| = ± delta potential / |gravity|
    //
    // Now we multiply both sides by the normalized gravity to get a vector again.
    //
    //      offset = ± delta potential * gravity / |gravity|²
    //
    // The sign can be seen to be a minus by thinking about it..

    gravity = system.probeGravity(new_pos_initial);
    const point corrected_pos_midpoint = new_pos_initial + diff * gravity / gravity.sqmag() / 2.f;
    gravity = system.probeGravity(corrected_pos_midpoint);
    const point corrected_pos = new_pos_initial + diff * gravity / gravity.sqmag();

    const bool debug_output = false;
    if (debug_output) {
        const float new_diff = system.probePotential(corrected_pos) - desired_potential;
        std::cerr << "Absolute potential:           " << new_potential << '\n';
        std::cerr << "Difference before correcting: " << diff << '\n';
        std::cerr << "Difference after correcting:  " << new_diff << '\n';
        std::cerr << "Improved error by factor:     " << diff / new_diff << '\n';

        std::cerr << '\n';
    }

    return corrected_pos;
}

std::vector<point> generateGliderTrajectory(point const& start_pos,
                                            System const& system,
                                            const float spiral_factor,
                                            const std::size_t max_steps) {
    point last_pos = start_pos;
    float desired_potential = system.probePotential(last_pos);

    float sq_last_dist = 1.f;
    const float sq_lower_dist_limit = 0.005f;
    const float sq_upper_dist_limit = 400.f;

    std::vector<point> points;

    int steps = max_steps;
    while (sq_last_dist > sq_lower_dist_limit &&
           sq_last_dist < sq_upper_dist_limit &&
           --steps) {
        const point new_pos = gliderStep(last_pos, desired_potential, system, false);

        if (spiral_factor != 0.f) {
            const float weighted_angle_diff = system.probeWeightedAngleDiff(last_pos, new_pos);
            desired_potential += weighted_angle_diff * spiral_factor;
        }

        sq_last_dist = (new_pos - last_pos).sqmag();

        points.push_back(new_pos);
        last_pos = new_pos;
    }

    return points;
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

        float path_length = 0.f;
        std::vector<point> centres;
        const float curve_check_interval = 50.f;
        const float last_curve_check = 0.f;
        std::size_t last_curve_check_index = 0;
        for (std::size_t i = 1; i < trajectory.size(); ++i) {
            // Only count points inside bounds
            if (trajectory[i].x < bounds[0].x || trajectory[i].y < bounds[0].y ||
                trajectory[i].x > bounds[1].x || trajectory[i].y > bounds[1].y) {
                continue;
            }

            const point diff = trajectory[i] - trajectory[i-1];
            path_length += diff.mag();


            if (path_length - last_curve_check > curve_check_interval &&
                last_curve_check_index < i - 2) {
                
                const point
                    a = trajectory[last_curve_check_index],
                    b = trajectory[(last_curve_check_index + i) / 2],
                    c = trajectory[i];

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
        }

        const float score = path_length;

        if (score > candidate_score) {
            candidate_score = score;
            candidate = p;
            std::cerr << "score: " << score << '\n'
                      << "    path length: " << path_length << '\n';
        }
    }

    return candidate;
}

#endif // GLIDER_HPP
