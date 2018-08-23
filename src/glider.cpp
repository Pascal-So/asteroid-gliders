#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <random>
#include <sstream>
#include "point.hpp"
#include "system.hpp"


// ##########   Main parameters ##################

const unsigned int height = 720;
const unsigned int width = 1080;

int seed = 26;
const size_t nr_planets = 4;
const size_t nr_gliders = 4000;

const bool debug_output = false;

// ##############################################


point gliderStep(point const& pos, const float desired_potential, System const& system) {
    const float stepsize = 3.f;

    // First calculate the new position by integrating with the midpoint method.

    point gravity = system.probeGravity(pos).norm();
    point motion_half (gravity.y, -gravity.x);
    motion_half *= stepsize / 2;

    gravity = system.probeGravity(pos + motion_half).norm();
    point motion (gravity.y, -gravity.x);
    motion *= stepsize;

    const point new_pos_initial = pos + motion;

    // Uncomment this to only use explicit euler / midpoint method.
    // return pos + motion_half * 2.f; // Euler
    return new_pos_initial;         // Midpoint


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



void drawPlanets(sf::RenderWindow* win, System const& system) {
    const sf::Color planet_color(100, 100, 100);
    for (auto const& p:system.planets) {
        float radius = sqrt(p.second) * 10.f;
        sf::CircleShape planet(radius);
        planet.setFillColor(planet_color);
        planet.setPosition(p.first.x - radius, p.first.y - radius);
        win->draw(planet);
    }
}
void drawSingleTrajectory(sf::RenderWindow* win, System const& system, point const& start_pos) {
    point last_pos = start_pos;
    float last_potential = system.probePotential(last_pos);

    float sq_last_dist = 1.f;
    const float sq_lower_dist_limit = 0.001f;

    const sf::Color glider_color(255, 255, 255, 30);

    std::vector<sf::Vertex> vertices;
    vertices.emplace_back(sf::Vector2f(last_pos.x, last_pos.y),
                          glider_color);

    int max_steps = 300;
    while (sq_last_dist > sq_lower_dist_limit && --max_steps) {
        const point new_pos = gliderStep(last_pos, last_potential, system);

        vertices.emplace_back(sf::Vector2f(new_pos.x, new_pos.y), glider_color);
        sq_last_dist = (new_pos - last_pos).sqmag();
        last_pos = new_pos;
    }

    win->draw(&vertices[0], vertices.size(), sf::LinesStrip);
}
void generateAndDrawEverything(sf::RenderWindow* win,
                               const int seed, const size_t nr_planets, const size_t nr_gliders,
                               std::array<point, 2> const& bounds) {
    std::mt19937 rng (seed);
    System system (nr_planets, bounds, rng);

    win->clear(sf::Color(30, 30, 30));
    drawPlanets(win, system);

    for (size_t i = 0; i < nr_gliders; ++i) {
        const point p = randomPoint(bounds, rng);
        drawSingleTrajectory(win, system, p);
    }
    win->display();
}

int main() {
    sf::RenderWindow win(sf::VideoMode(width, height), "Loren's Asteroid Gliders");
    const std::array<point, 2> bounds {point(0.f, 0.f), point((float)width, (float)height)};

    generateAndDrawEverything(&win, seed, nr_planets, nr_gliders, bounds);

    while (win.isOpen()) {
        sf::Event event;
        while (win.pollEvent(event)) {
            switch (event.type) {
            case sf::Event::Closed:
                win.close();
                break;
            case sf::Event::KeyPressed:
                switch (event.key.code) {
                case sf::Keyboard::Escape:
                case sf::Keyboard::Q:
                    win.close();
                    break;
                case sf::Keyboard::R:
                    ++seed;
                    generateAndDrawEverything(&win, seed, nr_planets, nr_gliders, bounds);
                    std::cout << "regenerated with seed " << seed << "\n";
                    break;
                default:
                    break;
                }
                break;
            default:
                break;
            }
        }
    }
}
