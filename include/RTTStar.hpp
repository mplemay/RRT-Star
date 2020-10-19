#pragma once

#include <chrono>  // NOLINT [build/c++11]
#include <map>
#include <memory>
#include <random>
#include <tuple>
#include <vector>

#include "CostMap2D.hpp"

/**
 * A single-threaded implementation of the RRT Star path finding algorithm.
 */
class RTTStar {
 public:
  explicit RTTStar(const std::unique_ptr<CostMap2D> cost_map, double_t epsilon,
                   double_t radius) noexcept;

  std::vector<Point2D> initial_path(const Point2D &start, const Point2D &goal);
  std::vector<Point2D> refine_path(size_t time);

 protected:
  void iterate();
  void reset();
  std::vector<Point2D> path();

  Point2D nearest(const Point2D &x);
  std::vector<Point2D> near(const Point2D &point);
  Point2D steer(const Point2D &x_nearest, const Point2D &x);
  bool obstacle_free(const Point2D &x_nearest, const Point2D &x_new);

  const std::unique_ptr<CostMap2D> cost_map;
  std::optional<Point2D> start_point, end_point;
  std::map<Point2D, double_t> costs;
  std::map<Point2D, Point2D> relations;

 private:
  std::vector<Point2D> traversed_points(const Point2D &x_nearest,
                                        const Point2D &x_new);
  inline double_t distance(const Point2D &first_point,
                           const Point2D &second_point);

  const double_t eps, radius;

  std::mt19937 gen;
  std::uniform_int_distribution<> x_sample_space, y_sample_space;
};
