#include <iostream>
#include <tuple>
#include <vector>
#include <algorithm>
#include <numeric>

#include "Costmap2D.hpp"
#include "RTTStar.hpp"

/**
 * Prompts the users for a value.
 *
 * @tparam T The type of the value to prompt the user for.
 * @param prompt The prompt.
 * @return The user entered value.
 */
template <typename T>
T prompt(const std::string &prompt) {
    T val;
    std::cout << prompt << std::flush;
    std::cin >> val;
    return val;
}

/**
 * Returns a point which the user entered.
 *
 * @return A point.
 */
Point2D prompt_point() {
    int64_t x, y;
    std::cout << "Enter the x coordinate: " << std::flush;
    std::cin >> x;
    std::cout << "Enter the y coordinate: " << std::flush;
    std::cin >> y;
    return std::make_tuple(x, y);
}

/**
 * Returns a cost map with the borders populated.
 *
 * @param border_size The size of the borders.
 * @return The populated cost map.
 */
Costmap2D<bool> make_cost_map(const size_t &border_size) {
    Costmap2D<bool> cost_map;

    std::vector<size_t> indices(border_size);
    std::iota(indices.begin(), indices.end(), 0);

    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_int_distribution<bool> distribution(false,true);

    std::for_each(indices.begin(), indices.end(), [&cost_map, &indices, &distribution, &generator](const auto &i) {
        std::for_each(indices.begin(), indices.end(), [&cost_map, &indices, &distribution, &generator](const auto &i) {
            if (distribution(generator)) {
                cost_map.update(std::make_tuple(i, 0), true);
            }

        });
    });

    return cost_map;
}

int main() {
    const auto border_size = prompt<size_t>("Enter the size of the boarder square: ");
    const auto cost_map = make_cost_map(border_size);
    const auto step_size = prompt<double_t>("Enter the step size (i.e. double): ");
    const auto radius = prompt<double_t>("Enter the size of the neighbor search radius (i.e. double): ");
    const auto extra_time = prompt<size_t>("Enter the amount of time the search should continue for after getting an initial path (in milliseconds): ");

    RTTStar rtt_star(cost_map, step_size, radius);

    std::cout << "Where should the algorithm search start point be?" << std::endl;
    const auto start_point = prompt_point();
    std::cout << "Where should the algorithm search goal point be?" << std::endl;
    const auto end_point = prompt_point();

    const auto path = rtt_star.initial_path(start_point, end_point);

    std::cout << "Initial path (size = " << path.size() << "): ";
    for_each(path.begin(), path.end(), [](const auto &a) {
        std::cout << "(" <<std::get<0>(a) << "," << std::get<1>(a) << ") ";
    });
    std::cout << std::endl;

    const auto refined_path = rtt_star.refine_path(extra_time);

    std::cout << "Refined path (size = " << refined_path.size() << "): ";
    for_each(refined_path.begin(), refined_path.end(), [](const auto &a) {
        std::cout << "(" << std::get<0>(a) << "," << std::get<1>(a) << ") ";
    });
    std::cout << std::endl;

    return 0;
}
