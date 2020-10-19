#pragma once

#include <tuple>
#include <vector>
#include <map>

/**
 * A simple representation of a point in 2D space.
 */
typedef std::tuple<int64_t, int64_t> Point2D;

/**
 * A class which represents a 2D cost map of booleans which represent whether or a coordinate is occupied.
 */
class Costmap2D {
public:
    Costmap2D() noexcept = default;
    std::optional<bool> update(const Point2D &point, const bool &value);
    std::optional<bool> get(const Point2D &point) const;
    [[nodiscard]] bool contains(const Point2D &point) const;
    [[nodiscard]] std::tuple<std::uniform_int_distribution<>, std::uniform_int_distribution<>> sample_space(const std::vector<Point2D> &waypoints) const;

private:
    std::map<Point2D, bool> sparse_costmap;
    std::optional<int64_t> min_x, max_x, min_y, max_y;
};
