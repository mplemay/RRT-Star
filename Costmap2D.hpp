#pragma once

#include <tuple>
#include <vector>
#include <map>
#include <algorithm>
#include <numeric>

/**
 * A simple representation of a point in 2D space.
 */
typedef std::tuple<int64_t, int64_t> Point2D;

template<class T>
class Costmap2D {
public:
    Costmap2D() noexcept = default;
    std::optional<T> update(const Point2D &point, const T &value);
    std::optional<T> get(const Point2D &point) const;
    [[nodiscard]] bool contains(const Point2D &point) const;
    [[nodiscard]] std::tuple<std::uniform_int_distribution<>, std::uniform_int_distribution<>> sample_space(const std::vector<Point2D> &waypoints) const;

private:
    std::map<Point2D, T> sparse_costmap;
    std::optional<int64_t> min_x, max_x, min_y, max_y;
};

/**
 * Updates a value at a location.
 *
 * @tparam T The data type to use for the cost map.
 * @param point The point to set the value for.
 * @param value The value to associate with a point.
 * @return Returns the old value if there is one.
 */
template<class T>
std::optional<T> Costmap2D<T>::update(const Point2D &point, const T &value) {
    // If the minimum values was previously set
    if (this->max_y.has_value() and this->max_x.has_value() and this->min_y.has_value() and this->min_x.has_value()) {
        this->min_x = std::min(this->min_x.value(), std::get<0>(point));
        this->min_y = std::min(this->min_y.value(), std::get<1>(point));
        this->max_x = std::max(this->max_x.value(), std::get<0>(point));
        this->max_y = std::max(this->max_y.value(), std::get<1>(point));
    } else {
        this->min_x = std::get<0>(point);
        this->max_x = std::get<0>(point);
        this->min_y = std::get<1>(point);
        this->max_y = std::get<1>(point);
    }

    // Get the location and value of the point if it exists
    const auto loc = this->sparse_costmap.find(point);
    const auto val = loc->second;

    // Copy point and value into map
    this->sparse_costmap.insert(std::make_pair(point, value));

    // Return the old value if it is valid
    return loc != this->sparse_costmap.end() ? std::optional<T>{val} : std::nullopt;
}

/**
 * Returns the value at the give point.
 *
 * @tparam T The data type to use for the cost map.
 * @param point The point to get the value of.
 * @return The value associated with a point.
 */
template<class T>
std::optional<T> Costmap2D<T>::get(const Point2D &point) const {
    const auto loc = this->sparse_costmap.find(point);
    return loc != this->sparse_costmap.end() ? std::optional<T>{loc->second} : std::nullopt;
}

/**
 * Returns a normal distribution of the values based on the maxima and minima.
 *
 * @tparam T The data type to use for the cost map.
 * @param waypoints Additional values to adjust the maxima and minima with.
 * @return The random normal distribution of x and y values.
 */
template<class T>
std::tuple<std::uniform_int_distribution<>, std::uniform_int_distribution<>> Costmap2D<T>::sample_space(const std::vector<Point2D> &waypoints) const {
    int64_t s_min_x, s_min_y, s_max_x, s_max_y;

    // If there are no values, the sample space should not be infinite
    if (!(this->max_y.has_value() and this->max_x.has_value() and this->min_y.has_value() and this->min_x.has_value()) and waypoints.empty()) {
        throw std::runtime_error("The sample space is undefined");
    }

    // If the there is a already a max
    if (this->max_y.has_value() and this->max_x.has_value() and this->min_y.has_value() and this->min_x.has_value()) {
        s_min_x = this->min_x.value();
        s_max_x = this->max_x.value();
        s_max_y = this->max_y.value();
        s_min_y = this->min_y.value();
    } else {
        s_max_x = s_min_x = std::get<0>(waypoints.front());
        s_min_y = s_max_y = std::get<1>(waypoints.front());
    }

    // Traverse all of the waypoints updating the extrema
    std::for_each(waypoints.begin(), waypoints.end(), [&s_min_x, &s_max_x, &s_min_y, &s_max_y](const auto &point) {
        s_min_x = std::min(s_min_x, std::get<0>(point));
        s_max_x = std::max(s_max_x, std::get<0>(point));
        s_max_y = std::max(s_max_y, std::get<1>(point));
        s_min_y = std::min(s_min_y, std::get<1>(point));
    });

    return std::make_tuple(std::uniform_int_distribution<>(s_min_x, s_max_x), std::uniform_int_distribution<>(s_min_y, s_max_y));
}

/**
 * Checks if a value has been set in the cost map.
 *
 * @tparam T The data type to use for the cost map.
 * @param point The point to check for.
 * @return true if the value is in the cost map; false otherwise.
 */
template<class T>
bool Costmap2D<T>::contains(const Point2D &point) const {
    return this->sparse_costmap.contains(point);
}