#pragma once

#include "farmtrax/field.hpp"
#include "farmtrax/utils/utils.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/touches.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <memory>
#include <vector>

namespace farmtrax {

    class ObstacleAvoider {
        std::vector<BPolygon> obstacles_;
        std::vector<BPolygon> inflated_obstacles_;
        float inflation_distance_;
        concord::Datum datum_;

      public:
        // Constructor takes array of polygons as obstacles
        explicit ObstacleAvoider(const std::vector<BPolygon> &obstacles, const concord::Datum &datum = concord::Datum{})
            : obstacles_(obstacles), inflation_distance_(0.0f), datum_(datum) {}

        // Constructor with concord::Polygon obstacles
        explicit ObstacleAvoider(const std::vector<concord::Polygon> &obstacles,
                                 const concord::Datum &datum = concord::Datum{})
            : datum_(datum) {
            obstacles_.reserve(obstacles.size());
            for (const auto &poly : obstacles) {
                obstacles_.push_back(utils::to_boost(poly));
            }
            inflation_distance_ = 0.0f;
        }

        // Main avoidance method
        std::vector<std::shared_ptr<Swath>> avoid(const std::vector<std::shared_ptr<const Swath>> &input_swaths,
                                                  float inflation_distance) {
            inflation_distance_ = inflation_distance;
            inflate_obstacles();

            std::vector<std::shared_ptr<Swath>> result_swaths;

            for (const auto &swath : input_swaths) {
                auto processed_swaths = process_swath(swath);
                result_swaths.insert(result_swaths.end(), processed_swaths.begin(), processed_swaths.end());
            }

            return result_swaths;
        }

        // Overload for non-const swaths
        std::vector<std::shared_ptr<Swath>> avoid(const std::vector<std::shared_ptr<Swath>> &input_swaths,
                                                  float inflation_distance) {
            std::vector<std::shared_ptr<const Swath>> const_swaths;
            const_swaths.reserve(input_swaths.size());
            for (const auto &swath : input_swaths) {
                const_swaths.push_back(swath);
            }
            return avoid(const_swaths, inflation_distance);
        }

        // Get original obstacles
        const std::vector<BPolygon> &get_obstacles() const { return obstacles_; }

        // Get inflated obstacles
        const std::vector<BPolygon> &get_inflated_obstacles() const { return inflated_obstacles_; }

      private:
        // Inflate all obstacles by the given distance
        void inflate_obstacles() {
            inflated_obstacles_.clear();
            inflated_obstacles_.reserve(obstacles_.size());

            for (const auto &obstacle : obstacles_) {
                BPolygon inflated = inflate_polygon(obstacle, inflation_distance_);
                if (!boost::geometry::is_empty(inflated)) {
                    inflated_obstacles_.push_back(inflated);
                }
            }
        }

        // Inflate a single polygon
        BPolygon inflate_polygon(const BPolygon &polygon, float distance) const {
            boost::geometry::model::multi_polygon<BPolygon> buffered;

            // Buffer strategy components
            boost::geometry::strategy::buffer::distance_symmetric<double> dist_strategy(distance);
            boost::geometry::strategy::buffer::side_straight side_strategy;
            boost::geometry::strategy::buffer::join_miter join_strategy;
            boost::geometry::strategy::buffer::end_flat end_strategy;
            boost::geometry::strategy::buffer::point_square point_strategy;

            try {
                boost::geometry::buffer(polygon, buffered, dist_strategy, side_strategy, join_strategy, end_strategy,
                                        point_strategy);
            } catch (const std::exception &e) {
                std::cerr << "Buffer operation failed: " << e.what() << std::endl;
                return polygon; // Return original if buffering fails
            }

            // Return the largest polygon from the multi_polygon result
            if (!buffered.empty()) {
                const BPolygon *largest = &buffered.front();
                double max_area = boost::geometry::area(*largest);

                for (const auto &poly : buffered) {
                    double area = boost::geometry::area(poly);
                    if (area > max_area) {
                        max_area = area;
                        largest = &poly;
                    }
                }
                return *largest;
            }

            return polygon; // Return original if no result
        }

        // Process a single swath against all inflated obstacles
        std::vector<std::shared_ptr<Swath>> process_swath(const std::shared_ptr<const Swath> &swath) {
            std::vector<std::shared_ptr<Swath>> result;

            // Check if swath intersects with any inflated obstacle
            bool intersects = false;
            std::vector<BLineString> cut_segments;

            for (const auto &inflated_obstacle : inflated_obstacles_) {
                if (boost::geometry::touches(swath->b_line, inflated_obstacle) ||
                    boost::geometry::intersects(swath->b_line, inflated_obstacle)) {
                    intersects = true;

                    // Cut the swath by subtracting the inflated obstacle
                    std::vector<BLineString> difference_result;
                    boost::geometry::difference(swath->b_line, inflated_obstacle, difference_result);

                    cut_segments.insert(cut_segments.end(), difference_result.begin(), difference_result.end());
                }
            }

            if (!intersects) {
                // No intersection, return original swath as a copy
                result.push_back(std::make_shared<Swath>(*swath));
                return result;
            }

            // If intersects, create cut segments and connections
            if (cut_segments.empty()) {
                // Swath is completely inside obstacle, skip it
                return result;
            }

            // Sort segments by position along original swath direction
            sort_segments_by_position(cut_segments, swath);

            // Create swaths from cut segments and connections between them
            for (size_t i = 0; i < cut_segments.size(); ++i) {
                const auto &segment = cut_segments[i];

                if (boost::geometry::length(segment) < 1e-6) {
                    continue; // Skip very short segments
                }

                // Create swath from segment
                concord::Point start_point = utils::from_boost(segment.front(), datum_);
                concord::Point end_point = utils::from_boost(segment.back(), datum_);

                auto cut_swath = std::make_shared<Swath>(create_swath(start_point, end_point, swath->type, ""));
                result.push_back(cut_swath);

                // Create connection to next segment (if not last)
                if (i < cut_segments.size() - 1) {
                    const auto &next_segment = cut_segments[i + 1];
                    if (boost::geometry::length(next_segment) >= 1e-6) {
                        concord::Point connection_start = end_point;
                        concord::Point connection_end = utils::from_boost(next_segment.front(), datum_);

                        auto connection_swath = std::make_shared<Swath>(
                            create_swath(connection_start, connection_end, SwathType::Around, ""));
                        result.push_back(connection_swath);
                    }
                }
            }

            return result;
        }

        // Sort segments by their position along the original swath direction
        void sort_segments_by_position(std::vector<BLineString> &segments,
                                       const std::shared_ptr<const Swath> &original_swath) const {
            BPoint start_point(original_swath->line.getStart().enu.x, original_swath->line.getStart().enu.y);

            std::sort(segments.begin(), segments.end(), [&start_point](const BLineString &a, const BLineString &b) {
                double dist_a = boost::geometry::distance(start_point, a.front());
                double dist_b = boost::geometry::distance(start_point, b.front());
                return dist_a < dist_b;
            });
        }
    };

} // namespace farmtrax
