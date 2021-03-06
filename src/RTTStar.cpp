#include "RTTStar.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <numeric>
#include <utility>

/**
 * Constructs the RRT Star class.
 *
 * @param cost_map The cost_map to perform path finding on.
 * @param epsilon The maximum distance the new point can be away from the
 * nearest point.
 * @param radius The radius to select nearby points from.
 */
RTTStar::RTTStar(std::unique_ptr<CostMap2D> cost_map, double_t epsilon,
                 double_t radius) noexcept
    : cost_map(std::move(cost_map)), eps(epsilon), radius(radius) {
  this->gen = std::mt19937(std::random_device()());
}

/**
 * Constructs an initial path between two points.
 *
 * @param start The starting point of the path.
 * @param goal The goal point of the path.
 * @return The first path constructed between the start && goal.
 */
std::vector<Point2D> RTTStar::initial_path(const Point2D &start,
                                           const Point2D &goal) {
  this->reset();

  this->start_point = start;
  this->end_point = goal;

  this->costs.insert(std::make_pair(start, 0.0));

  std::tie(this->x_sample_space, this->y_sample_space) =
      this->cost_map->sample_space(std::vector{start, goal});

  while (!this->costs.contains(goal) && !this->relations.contains(goal)) {
    this->iterate();
  }

  return this->path();
}

/**
 * Iterates on the current path in order to refine it.
 *
 * @param time The amount of time in milliseconds to refine the path for.
 * @return Returns the refined path.
 */
std::vector<Point2D> RTTStar::refine_path(size_t time) {
  const auto end_time =
      std::chrono::system_clock::now() + std::chrono::milliseconds(time);

  // Run the function for approximately the specified amount of time
  while (std::chrono::system_clock::now() < end_time) {
    this->iterate();
  }

  return this->path();
}

/**
 * Iterates on the current path in order to refine it.
 */
void RTTStar::iterate() {
  // Return if the start || end points were not set
  if (!this->start_point.has_value() || !this->end_point.has_value()) {
    return;
  }

  // Get a new random point
  const Point2D x =
      std::make_tuple(x_sample_space(this->gen), y_sample_space(this->gen));

  // Check if the random point is valid || has already been selected
  if (this->cost_map->get(x) == std::nullopt && !costs.contains(x)) {
    // Get the point nearest to the new point
    const Point2D x_nearest = this->nearest(x);
    // Adjust the distance of the new point
    const Point2D x_new = this->steer(x_nearest, x);

    // If the new point does not intersect with an obstacle
    if (this->obstacle_free(x_nearest, x_new)) {
      // Set the points cost && parent
      this->costs.insert(std::make_pair(
          x_new, costs.at(x_nearest) + this->distance(x_new, x_nearest)));
      this->relations.insert(std::make_pair(x_new, x_nearest));

      // Update the neighbors of the new point with the new point if the cost is
      // lower
      const auto x_near = this->near(x_new);
      std::for_each(
          x_near.begin(), x_near.end(), [&x_new, this](const auto &p) {
            const auto c = this->costs.at(x_new) + this->distance(p, x_new);
            if (c < this->costs.at(p) && this->obstacle_free(p, x_new)) {
              this->costs.at(p) = c;
              this->relations.at(p) = x_new;
            }
          });
    }
  }
}

/**
 * Resets the class so that it can be re-run on an updated map.
 */
void RTTStar::reset() {
  this->costs.clear();
  this->relations.clear();
  this->start_point = std::nullopt;
  this->end_point = std::nullopt;
}

/**
 * Returns the current path
 *
 * @return Return the current optimal path || an empty path if a path does not
 * exist.
 */
std::vector<Point2D> RTTStar::path() {
  // Check if there if the start && goal have not been set
  if (!this->start_point.has_value() || !this->end_point.has_value()) {
    return std::vector<Point2D>{};
  }

  // Construct the path
  std::vector<Point2D> p{this->end_point.value()};
  while (true) {
    p.emplace_back(this->relations.at(p.back()));

    // Check if the point was found
    if (p.back() == start_point.value()) {
      return p;
    }

    // If a point is double counted a valid path does not exist.
    if (p.back() == p.at(p.size() - 2)) {
      return std::vector<Point2D>{};
    }
  }
}

/**
 * Returns the nearest point to a point.
 *
 * @param x The point to find a point near.
 * @return The nearest point to a point.
 */
Point2D RTTStar::nearest(const Point2D &x) {
  const auto it = std::min_element(this->costs.begin(), this->costs.end(),
                                   [&x, this](const auto &a, const auto &b) {
                                     return this->distance(a.first, x) <
                                                this->distance(b.first, x) &&
                                            a.first != x;
                                   });

  return it->first;
}

/**
 * Limits the maximum step size of a move.
 *
 * @param x_nearest The anchor point which you can only move a certain distance
 * away from.
 * @param x The ideal move position.
 * @return The new point to move to.
 */
Point2D RTTStar::steer(const Point2D &x_nearest, const Point2D &x) {
  const auto dist = this->distance(x_nearest, x);

  if (dist >= this->eps) {
    return std::make_tuple(
        std::get<0>(x_nearest) +
            std::floor((static_cast<double_t>(std::get<0>(x) -
                                              std::get<0>(x_nearest)) *
                        this->eps) /
                       dist),
        std::get<1>(x_nearest) +
            std::floor((static_cast<double_t>(std::get<1>(x) -
                                              std::get<1>(x_nearest)) *
                        this->eps) /
                       dist));
  } else {
    return x;
  }
}

/**
 * Checks if the line
 *
 * @param x_nearest The nearest point to x_new
 * @param x_new The point that will be moved to.
 * @return The
 */
bool RTTStar::obstacle_free(const Point2D &x_nearest, const Point2D &x_new) {
  const auto traversed = traversed_points(x_nearest, x_new);

  // Check that none of the points are in the cost map
  return std::all_of(
      traversed.begin(), traversed.end(),
      [this](const auto &pt) { return !this->cost_map->contains(pt); });
}

/**
 * Returns the points that fall on a line.
 *
 * @param x_nearest A point on the line.
 * @param x_new The associated cost_map data type.
 * @return The points that fall on the line.
 */
std::vector<Point2D> RTTStar::traversed_points(const Point2D &x_nearest,
                                               const Point2D &x_new) {
  std::vector<Point2D> intersections;

  // The change
  const auto delta_x = std::get<0>(x_new) - std::get<0>(x_nearest);
  const auto delta_y = std::get<1>(x_new) - std::get<1>(x_nearest);

  // If the line is not vertical
  if (delta_x != 0) {
    // Get the sign of the slope && slope
    const double_t sign =
        (delta_y * delta_x > 0) ? 1 : ((delta_y * delta_x < 0) ? -1 : 0);
    const double_t delta_error = std::abs(static_cast<double_t>(delta_y) /
                                          static_cast<double_t>(delta_x));

    // Get the x indices to traverse
    std::vector<int64_t> indices(std::abs(delta_x) + 1);
    std::iota(indices.begin(), indices.end(),
              std::min(std::get<0>(x_new), std::get<0>(x_nearest)));

    intersections.reserve(indices.size());

    // Get the y value associated with the smallest x value
    int64_t y = std::min(std::get<0>(x_new), std::get<0>(x_nearest)) ==
                        std::get<0>(x_new)
                    ? std::get<1>(x_new)
                    : std::get<1>(x_nearest);
    double_t error = 0.0;

    // Collect all of the points intersecting on the line
    std::for_each(
        indices.begin(), indices.end(),
        [&error, &y, &sign, &delta_error, &intersections](const auto &x) {
          intersections.emplace_back(std::make_tuple(x, y));
          error += delta_error;
          while (error >= 0.5) {
            y += sign;
            error -= 1.0;
            intersections.emplace_back(std::make_tuple(x, y));
          }
        });
  } else {
    // Get the y indices to traverse
    std::vector<int64_t> indices(std::abs(delta_y) + 1);
    std::iota(indices.begin(), indices.end(),
              std::min(std::get<1>(x_new), std::get<1>(x_nearest)));

    intersections.reserve(indices.size());

    // Get the x value
    const int64_t x = std::get<0>(x_new);

    // Collect all of the points on the line
    std::transform(indices.begin(), indices.end(),
                   std::back_inserter(intersections),
                   [&x](const auto &y) { return std::make_tuple(x, y); });
  }

  return intersections;
}

/**
 * Returns the euclidean distance between two points.
 *
 * @param first_point A point to get the distance between.
 * @param second_point A point to get the distance between.
 * @return The distance between the two points.
 */
inline double_t RTTStar::distance(const Point2D &first_point,
                                  const Point2D &second_point) {
  return std::sqrt(static_cast<double_t>(
      std::pow(std::get<0>(first_point) - std::get<0>(second_point), 2) +
      std::pow(std::get<1>(first_point) - std::get<1>(second_point), 2)));
}

/**
 * Returns the points nearby a point.
 *
 * @param point The points to find nearby points for.
 * @return The nearby points.
 */
std::vector<Point2D> RTTStar::near(const Point2D &point) {
  std::vector<Point2D> near;

  std::for_each(this->costs.begin(), this->costs.end(), [&](const auto &p) {
    if (this->distance(p.first, point) < this->radius && p.first != point) {
      near.emplace_back(p.first);
    }
  });

  return near;
}
