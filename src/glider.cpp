#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <random>
#include <sstream>
#include "point.hpp"
#include "system.hpp"


// ##########   Main parameters ##################

const unsigned int height = 720;
const unsigned int width = 1080;

int seed = 23;
const size_t nr_planets = 4;
const size_t nr_gliders = 100;

// ##############################################


point gliderStep(point const& pos, const float last_potential, System const& system) {
    const float stepsize = 1.f;

    // First calculate the new position by integrating with the midpoint method.

    point gravity = system.probeGravity(pos).norm();
    point motion_half (gravity.y, -gravity.x); // counter-clockwise around planet
    motion_half *= stepsize / 2;

    gravity = system.probeGravity(pos + motion_half).norm();
    point motion (gravity.y, -gravity.x);
    motion *= stepsize;

    const point new_pos_initial = pos + motion;

    // return new_pos_initial;
    // return pos + motion_half * 2.f;
    
    // Then correct for the difference in gravitational potential. Here we assume the
    // slope to be linear, because we're only correcting in a small neighbourhood. We
    // can thus use the fact that the gravitational force vector is minus the gradient
    // of the potential, so we have: (where '·' denotes the scalar product)
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
    
    const float new_potential = system.probePotential(new_pos_initial);
    gravity = system.probeGravity(new_pos_initial);

    const point final_pos =
        new_pos_initial + gravity * (new_potential - last_potential) / gravity.sqmag();

    // const float old_diff = new_potential - last_potential;
    // const float new_diff = system.probePotential(final_pos) - last_potential;

    // std::cerr << "Improved error by factor: " << old_diff / new_diff << '\n';
    
    // std::cerr << "Difference before correcting: " << old_diff << '\n';
    // std::cerr << "Difference after correcting:  " << new_diff << "\n\n";

    return final_pos;
}



void drawPlanets(sf::RenderWindow* win, System const& system) {
    
    for (auto const& p:system.planets) {
        float radius = sqrt(p.second) * 10.f;
        sf::CircleShape planet(radius);
        planet.setFillColor(sf::Color(180, 200, 210));
        planet.setPosition(p.first.x - radius, p.first.y - radius);
        win->draw(planet);
    }
}
void drawSingleTrajectory(sf::RenderWindow* win, System const& system, point const& start_pos) {
    point last_pos = start_pos;
    float last_potential = system.probePotential(last_pos);

    float sq_last_dist = 1.f;
    const float sq_lower_dist_limit = 0.001f;

    std::vector<sf::Vertex> vertices;
    vertices.emplace_back(sf::Vector2f(last_pos.x, last_pos.y),
                          sf::Color(200, 200, 200, 100));

    int max_steps = 3000;
    while (sq_last_dist > sq_lower_dist_limit && --max_steps) {
        const point new_pos = gliderStep(last_pos, last_potential, system);

        vertices.emplace_back(sf::Vector2f(new_pos.x, new_pos.y), sf::Color(200, 200, 200, 100));
        // std::cout << new_pos << '\n';
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
