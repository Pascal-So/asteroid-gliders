#include <boost/filesystem.hpp>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <random>
#include <sstream>
#include <iomanip>

#include "point.hpp"
#include "glider.hpp"
#include "system.hpp"



// ##########   Main parameters ##################

unsigned int height = 1000;
unsigned int width = 1800;

int seed = 49;
const std::size_t nr_planets = 6;
const std::size_t nr_gliders = 20000;
const std::size_t max_steps = 200;
const float spiral_factor = 4.f;

// ##############################################


// In here you will only find the drawing code, which is not too
// interesting. Have a look at glider.hpp to see the actual
// calculations.


void drawPlanets(sf::RenderWindow& win, System const& system) {
    const sf::Color planet_color(100, 100, 100);
    for (auto const& p:system.planets) {
        float radius = sqrt(p.mass) * 10.f;
        sf::CircleShape planet(radius);
        planet.setFillColor(planet_color);
        planet.setPosition(p.pos.x - radius, p.pos.y - radius);
        win.draw(planet);
    }
}

void drawSingleTrajectory(sf::RenderWindow& win,
                          System const& system,
                          point const& start_pos,
                          const std::size_t max_steps,
                          sf::Color const& glider_color = sf::Color(255, 255, 255, 20)) {
    const auto points = generateGliderTrajectory(start_pos, system, spiral_factor, max_steps);

    std::vector<sf::Vertex> vertices;
    
    for (auto const& p:points) {
        vertices.emplace_back(sf::Vector2f(p.x, p.y), glider_color);
    }

    win.draw(&vertices[0], vertices.size(), sf::LinesStrip);
}

void drawTrajectories(sf::RenderWindow& win,
                      System const& system, const std::size_t nr_gliders, const int seed,
                      std::array<point, 2> const& bounds, const std::size_t max_steps) {

    // offset the seed because otherwise, we might get the same points as we did for
    // the planets, which would not be very helpful.
    std::mt19937 rng (seed + 1000);

    // drawPlanets(win, system);

    for (std::size_t i = 0; i < nr_gliders; ++i) {
        const point p = point::randomPoint(bounds, rng);
        drawSingleTrajectory(win, system, p, max_steps);
    }
}

void drawPotentialPlot(sf::RenderWindow& win,
                       System const& system,
                       std::array<point,2> const& bounds, const float resolution = 2.f) {
    const sf::Vector2f tile_size (resolution, resolution);
    
    for (float x = bounds[0].x; x <= bounds[1].x - resolution; x += resolution) {
        for (float y = bounds[0].y; y <= bounds[1].y - resolution; y += resolution) {
            const point p (x + resolution / 2, y + resolution / 2);
            const float normalized_potential = -(system.probePotential(p)) / 100.f;
            const uint8_t value = 255.f * fmod(1000.0 * normalized_potential, 1.0);
            const sf::Color color(value, value, value);
            sf::RectangleShape tile(tile_size);
            tile.setFillColor(color);
            tile.setPosition(x, y);
            win.draw(tile);
        }
    }
}

void drawGravity(sf::RenderWindow& win, System const& system,
                 std::array<point,2> const& bounds, const float resolution = 12.f) {
    // const sf::Color line_color (255, 255, 255);
    const sf::Color dot_color (200, 125, 120);
    const float radius = 1.6f;

    std::vector<sf::Vertex> lines;
    
    for (float x = bounds[0].x; x <= bounds[1].x - resolution; x += resolution) {
        for (float y = bounds[0].y; y <= bounds[1].y - resolution; y += resolution) {
            const point p (x + resolution / 2, y + resolution / 2);
            const point grav = system.probeGravity(p).norm() * 8.f;

            sf::CircleShape dot(radius);
            dot.setFillColor(dot_color);
            dot.setPosition(p.x - radius, p.y - radius);
            win.draw(dot);
            
            lines.emplace_back(sf::Vector2f(p.x, p.y));
            lines.emplace_back(sf::Vector2f(p.x + grav.x, p.y + grav.y));
        }
    }

    win.draw(&lines[0], lines.size(), sf::Lines);
}

void saveScreenshot(sf::RenderWindow& win, const int seed) {
    namespace fs = boost::filesystem;

    auto const screenshot_dir = fs::current_path() / "screenshots";

    if (!fs::exists(screenshot_dir)) {
        fs::create_directory(screenshot_dir);
    }

    auto mkpath = [&screenshot_dir, seed](const std::size_t i) {
        std::stringstream name;
        name << "glider_" << seed << '_';
        name << std::setfill('0') << std::setw(4) << i;
        name << ".png";

        return screenshot_dir / name.str();
    };

    std::size_t i = 0;
    while (fs::exists(mkpath(i))) ++i;

    sf::Texture texture;
    texture.create(width, height);
    texture.update(win, 0, 0);
    sf::Image screenshot = texture.copyToImage();
    screenshot.saveToFile(mkpath(i).string());
}

int main() {
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow win(sf::VideoMode(width, height), "Loren's Asteroid Gliders",
                         sf::Style::Default, settings);
    std::array<point, 2> bounds {point(0.f, 0.f), point((float)width, (float)height)};

    std::mt19937 rng(seed);
    System system(nr_planets, bounds, rng);

    enum class Display {trajectories, potential, gravity} display = Display::trajectories;
    bool redraw = true;
    int nice_path_seed = 1;

    while (win.isOpen()) {
        sf::Event event;
        while (win.pollEvent(event)) {
            switch (event.type) {
            case sf::Event::MouseButtonPressed:
                drawSingleTrajectory(win, system, point(event.mouseButton.x, event.mouseButton.y),
                    max_steps, sf::Color(255, 0, 0));
                win.display();
                break;
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
                    if (event.key.shift) {
                        --seed;
                    } else {
                        ++seed;
                    }
                    std::cout << "New seed: " << seed << '\n';
                    redraw = true;
                    break;
                case sf::Keyboard::T:
                    redraw = true;
                    display = Display::trajectories;
                    break;
                case sf::Keyboard::P:
                    redraw = true;
                    display = Display::potential;
                    break;
                case sf::Keyboard::G:
                    redraw = true;
                    display = Display::gravity;
                    break;
                case sf::Keyboard::S:
                    saveScreenshot(win, seed);
                    break;
                case sf::Keyboard::N:
                    if (event.key.shift) {
                        --nice_path_seed;
                    } else {
                        ++nice_path_seed;
                    }
                    redraw = true;
                    break;
                default:
                    break;
                }
                break;
            default:
                break;
            }
        }

        if (redraw) {
            win.clear(sf::Color(30, 30, 30));

            rng.seed(seed);
            system = System(nr_planets, bounds, rng);
            
            switch (display) {
            case Display::trajectories:
                {
                    drawTrajectories(win, system, nr_gliders, seed, bounds, max_steps);

                    /*const size_t nice_path_length = 1000;
                    const point nice_path_start =
                        findNicePath(system, spiral_factor, nice_path_length,
                                     bounds, nice_path_seed);

                    drawSingleTrajectory(win, system, nice_path_start,
                    nice_path_length, sf::Color(255, 0, 0));*/
                }
                break;
            case Display::potential:
                drawPotentialPlot(win, system, bounds);
                drawTrajectories(win, system, 10, seed, bounds, max_steps);
                break;
            case Display::gravity:
                drawGravity(win, system, bounds);
                drawTrajectories(win, system, 10, seed, bounds, max_steps);
                break;
            }
            win.display();
            redraw = false;
        }
    }
}
