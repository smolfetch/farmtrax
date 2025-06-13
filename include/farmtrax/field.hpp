#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <concord/concord.hpp>
#include <concord/geometry/polygon/partitioner.hpp>
#include <entropy/entropy.hpp>

#include "farmtrax/utils/utils.hpp"

namespace farmtrax {
    using BPoint = boost::geometry::model::d2::point_xy<double>;
    using BLineString = boost::geometry::model::linestring<BPoint>;
    using BPolygon = boost::geometry::model::polygon<BPoint>;
    using BBox = boost::geometry::model::box<BPoint>;

    // R-tree value types for different geometry types
    using SwathRTreeValue = std::pair<BBox, std::size_t>;   // Bounding box + swath index
    using RingRTreeValue = std::pair<BBox, std::size_t>;    // Bounding box + ring index
    using PointRTreeValue = std::pair<BPoint, std::size_t>; // Point + index

    // R-tree types
    using SwathRTree = boost::geometry::index::rtree<SwathRTreeValue, boost::geometry::index::quadratic<16>>;
    using RingRTree = boost::geometry::index::rtree<RingRTreeValue, boost::geometry::index::quadratic<16>>;
    using PointRTree = boost::geometry::index::rtree<PointRTreeValue, boost::geometry::index::quadratic<16>>;

    struct Ring {
        concord::Polygon polygon;
        std::string uuid;
        bool finished = false;
        BPolygon b_polygon;
        BBox bounding_box; // Add bounding box for R-tree
    };

    inline Ring create_ring(const concord::Polygon &poly, std::string uuid = "") {
        if (uuid.empty())
            uuid = boost::uuids::to_string(boost::uuids::random_generator()());
        Ring ring = {poly, uuid};
        // Convert to boost polygon and compute bounding box
        ring.b_polygon = boost::geometry::model::polygon<BPoint>();
        for (auto const &pt : poly.getPoints()) {
            ring.b_polygon.outer().emplace_back(pt.enu.x, pt.enu.y);
        }
        if (!boost::geometry::equals(ring.b_polygon.outer().front(), ring.b_polygon.outer().back()))
            ring.b_polygon.outer().push_back(ring.b_polygon.outer().front());
        boost::geometry::correct(ring.b_polygon);
        ring.bounding_box = boost::geometry::return_envelope<BBox>(ring.b_polygon);
        return ring;
    }

    enum class SwathType {
        Swath,
        Connection,
        Around,
        Headland,
    };

    struct Swath {
        concord::Line line;
        std::string uuid;
        SwathType type = SwathType::Swath; // Default type is Swath
        bool finished = false;
        BLineString b_line;
        BBox bounding_box; // Add bounding box for R-tree

        // Additional fields needed for testing
        int id = -1;
        double width = 0.0;
        std::vector<concord::Point> points;
        std::vector<BPoint> centerline;

        // Get the head (start) point
        concord::Point getHead() const { return line.getStart(); }

        // Get the tail (end) point
        concord::Point getTail() const { return line.getEnd(); }

        // Swap head and tail (reverse direction)
        void swapDirection() {
            concord::Point temp = line.getStart();
            line.setStart(line.getEnd());
            line.setEnd(temp);
            // Update boost linestring and bounding box
            b_line.clear();
            b_line.emplace_back(line.getStart().enu.x, line.getStart().enu.y);
            b_line.emplace_back(line.getEnd().enu.x, line.getEnd().enu.y);
            bounding_box = boost::geometry::return_envelope<BBox>(b_line);
        }

        // Create a copy with swapped direction
        Swath withSwappedDirection() const {
            Swath swapped = *this;
            swapped.swapDirection();
            return swapped;
        }
    };

    inline Swath create_swath(const concord::Point &start, const concord::Point &end, SwathType type,
                              std::string uuid = "") {
        concord::Line L;
        L.setStart(start);
        L.setEnd(end);
        if (uuid.empty())
            uuid = boost::uuids::to_string(boost::uuids::random_generator()());

        Swath swath = {L, uuid, type};
        // Convert to boost linestring and compute bounding box
        swath.b_line.emplace_back(start.enu.x, start.enu.y);
        swath.b_line.emplace_back(end.enu.x, end.enu.y);
        swath.bounding_box = boost::geometry::return_envelope<BBox>(swath.b_line);
        return swath;
    }

    struct Part {
        Ring border;
        std::vector<Swath> swaths;
        std::vector<Ring> headlands;

        // R-trees for spatial indexing
        SwathRTree swath_rtree;
        RingRTree headland_rtree;
        PointRTree swath_endpoints_rtree; // For start/end points of swaths

        void rebuild_rtrees() {
            // Clear existing trees
            swath_rtree.clear();
            headland_rtree.clear();
            swath_endpoints_rtree.clear();

            // Rebuild swath R-tree
            for (std::size_t i = 0; i < swaths.size(); ++i) {
                swath_rtree.insert(std::make_pair(swaths[i].bounding_box, i));

                // Add start and end points to point R-tree
                BPoint start_pt(swaths[i].line.getStart().enu.x, swaths[i].line.getStart().enu.y);
                BPoint end_pt(swaths[i].line.getEnd().enu.x, swaths[i].line.getEnd().enu.y);
                swath_endpoints_rtree.insert(std::make_pair(start_pt, i * 2));   // Even indices for start points
                swath_endpoints_rtree.insert(std::make_pair(end_pt, i * 2 + 1)); // Odd indices for end points
            }

            // Rebuild headland R-tree
            for (std::size_t i = 0; i < headlands.size(); ++i) {
                headland_rtree.insert(std::make_pair(headlands[i].bounding_box, i));
            }
        }
    };

    using Grid = concord::Grid<uint8_t>;

    class Field {
      public:
        // Forward declaration of test functions
        friend double get_resolution(const Field &field);
        friend concord::Datum get_field_datum(const Field &field);
        friend double get_total_field_area(const Field &field);

      private:
        concord::Polygon border_;
        std::vector<Grid> grids_;
        std::vector<Part> parts_;
        double resolution_{};

        concord::Partitioner partitioner_;
        concord::Datum datum_{};
        entropy::NoiseGen noiseGen_;
        std::mt19937 rnd_;
        double overlap_threshold_{0.7};

      public:
        Field(const concord::Polygon &border, double resolution, const concord::Datum &datum, bool centred = true,
              double overlap_threshold = 0.7, double area_threshold = 0.5)
            : resolution_(resolution), border_(border), datum_(datum), overlap_threshold_(overlap_threshold) {
            grids_.emplace_back(border_, resolution_, datum_, centred);
            partitioner_ = concord::Partitioner(border_);
            auto divisions = partitioner_.partition(area_threshold);
            std::cout << "Split " << divisions.size() << " parts\n";
            parts_.reserve(divisions.size());
            for (auto const &poly : divisions) {
                Part p;
                p.border = create_ring(poly);
                parts_.push_back(std::move(p));
            }
        }

        Grid &get_grid(std::size_t i) {
            if (i >= grids_.size())
                throw std::out_of_range("Grid index out of range");
            return grids_[i];
        }

        const Grid &get_grid(std::size_t i) const {
            if (i >= grids_.size())
                throw std::out_of_range("Grid index out of range");
            return grids_[i];
        }

        const std::vector<Part> &get_parts() const { return parts_; }
        std::vector<Part> &get_parts() { return parts_; }

        void add_noise(std::size_t i = 0) {
            Grid &g = get_grid(i);
            noiseGen_.SetNoiseType(entropy::NoiseGen::NoiseType_OpenSimplex2);
            noiseGen_.SetFrequency(static_cast<float>(resolution_) / 300.0f);
            noiseGen_.SetSeed(static_cast<int>(rnd_()));
            for (std::size_t r = 0; r < g.rows(); ++r)
                for (std::size_t c = 0; c < g.cols(); ++c) {
                    float n = noiseGen_.GetNoise(static_cast<float>(r), static_cast<float>(c));
                    float val = (n + 1.0f) * 0.5f;
                    g(r, c).second = utils::float_to_byte(val);
                }
        }

        void gen_field(double swath_width, double angle_degrees = 0, int headland_count = 1) {
            for (auto &part : parts_) {
                part.headlands.clear();
                part.swaths.clear();
                part.headlands = generate_headlands(part.border.polygon, swath_width, headland_count);
                concord::Polygon interior = headland_count > 0 ? part.headlands.back().polygon : part.border.polygon;
                part.swaths = generate_swaths(swath_width, angle_degrees, interior);

                // Rebuild R-trees after generating new geometry
                part.rebuild_rtrees();
            }
        }

        // Spatial query methods using R-tree
        std::vector<std::size_t> find_nearby_swaths(std::size_t part_idx, const BPoint &query_point,
                                                    double radius) const {
            if (part_idx >= parts_.size())
                return {};

            const auto &part = parts_[part_idx];
            std::vector<std::size_t> result;

            // Create search box around query point
            BBox search_box(BPoint(query_point.x() - radius, query_point.y() - radius),
                            BPoint(query_point.x() + radius, query_point.y() + radius));

            // Query R-tree
            std::vector<SwathRTreeValue> candidates;
            part.swath_rtree.query(boost::geometry::index::intersects(search_box), std::back_inserter(candidates));

            // Filter by actual distance
            for (const auto &candidate : candidates) {
                std::size_t idx = candidate.second;
                if (idx < part.swaths.size()) {
                    double dist = boost::geometry::distance(query_point, part.swaths[idx].b_line);
                    if (dist <= radius) {
                        result.push_back(idx);
                    }
                }
            }

            return result;
        }

        std::vector<std::size_t> find_nearest_swath_endpoints(std::size_t part_idx, const BPoint &query_point,
                                                              std::size_t k = 1) const {
            if (part_idx >= parts_.size())
                return {};

            const auto &part = parts_[part_idx];
            std::vector<PointRTreeValue> nearest;
            part.swath_endpoints_rtree.query(boost::geometry::index::nearest(query_point, k),
                                             std::back_inserter(nearest));

            std::vector<std::size_t> result;
            for (const auto &point_value : nearest) {
                result.push_back(point_value.second);
            }
            return result;
        }

        // Find optimal swath traversal order using R-tree for nearest neighbor queries
        std::vector<std::size_t> optimize_swath_order(std::size_t part_idx, const BPoint &start_point) const {
            if (part_idx >= parts_.size())
                return {};

            const auto &part = parts_[part_idx];
            if (part.swaths.empty())
                return {};

            std::vector<std::size_t> order;
            std::vector<bool> visited(part.swaths.size(), false);
            BPoint current_pos = start_point;

            for (std::size_t i = 0; i < part.swaths.size(); ++i) {
                double min_dist = std::numeric_limits<double>::max();
                std::size_t next_swath = 0;

                // Use R-tree to find nearby candidates first
                auto nearby = find_nearby_swaths(part_idx, current_pos, min_dist);

                // Find closest unvisited swath
                for (std::size_t j = 0; j < part.swaths.size(); ++j) {
                    if (visited[j])
                        continue;

                    double dist_to_start =
                        boost::geometry::distance(current_pos, BPoint(part.swaths[j].line.getStart().enu.x,
                                                                      part.swaths[j].line.getStart().enu.y));
                    double dist_to_end = boost::geometry::distance(
                        current_pos, BPoint(part.swaths[j].line.getEnd().enu.x, part.swaths[j].line.getEnd().enu.y));

                    double min_swath_dist = std::min(dist_to_start, dist_to_end);
                    if (min_swath_dist < min_dist) {
                        min_dist = min_swath_dist;
                        next_swath = j;
                    }
                }

                visited[next_swath] = true;
                order.push_back(next_swath);

                // Update current position to end of chosen swath
                current_pos =
                    BPoint(part.swaths[next_swath].line.getEnd().enu.x, part.swaths[next_swath].line.getEnd().enu.y);
            }

            return order;
        }

      private:
        std::vector<Ring> generate_headlands(const concord::Polygon &polygon, double shrink_dist, int count) const {
            // Quick input validation
            if (count <= 0)
                return {}; // No headlands requested

            if (shrink_dist < 0)
                throw std::invalid_argument("negative shrink");

            // Ensure shrink distance is positive
            double actual_shrink_dist = std::max(0.1, shrink_dist);

            // Initialize result vector
            std::vector<Ring> H;

            // Ensure the polygon has points
            if (polygon.getPoints().size() < 3) {
                std::cerr << "Warning: Invalid polygon for headland generation" << std::endl;
                return H;
            }

            // Convert to boost polygon with careful error handling
            BPolygon base;
            try {
                base = utils::to_boost(polygon);
                if (base.outer().size() < 3) {
                    std::cerr << "Warning: Invalid boost polygon for headland generation" << std::endl;
                    return H;
                }
            } catch (const std::exception &e) {
                std::cerr << "Boost conversion error: " << e.what() << std::endl;
                return H;
            }

            // Generate each headland as a shrunken version of the previous one
            for (int i = 0; i < count; ++i) {
                // Get the current polygon to shrink (either the base field or the last headland)
                BPolygon current;
                try {
                    current = (i == 0 ? base : utils::to_boost(H.back().polygon));
                } catch (const std::exception &e) {
                    std::cerr << "Error getting polygon for headland " << i << ": " << e.what() << std::endl;
                    break;
                }

                // Set up buffer operation parameters
                boost::geometry::model::multi_polygon<BPolygon> buf;
                boost::geometry::strategy::buffer::distance_symmetric<double> dist(-actual_shrink_dist);
                boost::geometry::strategy::buffer::side_straight side;
                boost::geometry::strategy::buffer::join_miter join;
                boost::geometry::strategy::buffer::end_flat end;
                boost::geometry::strategy::buffer::point_square point;

                // Attempt to buffer (shrink) the polygon
                try {
                    boost::geometry::buffer(current, buf, dist, side, join, end, point);
                } catch (const std::exception &e) {
                    std::cerr << "Buffer operation failed for headland " << i << ": " << e.what() << std::endl;
                    break;
                }

                // Check if buffer operation produced any results
                if (buf.empty()) {
                    std::cerr << "Warning: empty buffer result at headland " << i << std::endl;
                    // Instead of breaking, we can return what we have so far
                    break;
                }

                // Find the polygon with the largest area (likely the main interior)
                const BPolygon *best = nullptr;
                double maxA = -std::numeric_limits<double>::max();

                for (const auto &cand : buf) {
                    try {
                        double a = std::abs(boost::geometry::area(cand)); // Use absolute area
                        if (a > maxA && a > 0.0001) {                     // Ensure minimum area
                            maxA = a;
                            best = &cand;
                        }
                    } catch (const std::exception &e) {
                        std::cerr << "Area calculation error: " << e.what() << std::endl;
                        continue;
                    }
                }

                // Check if we found a valid polygon
                if (!best || maxA <= 0.0001) {
                    std::cerr << "Warning: no valid polygon found at headland " << i << std::endl;
                    break;
                }

                try {
                    // Convert back to concord polygon
                    concord::Polygon tmp = utils::from_boost(*best, datum_);

                    // Clean up the polygon by removing colinear points
                    concord::Polygon simp = utils::remove_colinear_points(tmp, 1e-4);

                    // Ensure polygon is closed
                    if (!simp.getPoints().empty() &&
                        (simp.getPoints().front().enu.x != simp.getPoints().back().enu.x ||
                         simp.getPoints().front().enu.y != simp.getPoints().back().enu.y)) {
                        simp.addPoint(simp.getPoints().front());
                    }

                    // Create a ring from the polygon and add to results
                    H.push_back(create_ring(std::move(simp)));
                } catch (const std::exception &e) {
                    std::cerr << "Error creating headland " << i << ": " << e.what() << std::endl;
                    break;
                }
            }
            return H;
        }

        std::vector<Swath> generate_swaths(double swath_width, double angle_deg, const concord::Polygon &border) const {
            if (angle_deg == 0.0) {
                std::vector<Swath> best_out;
                std::size_t best_count = std::numeric_limits<std::size_t>::max();
                for (int deg = 1; deg < 360; ++deg) {
                    auto out = generate_swaths(swath_width, static_cast<double>(deg), border);
                    if (out.size() < best_count) {
                        best_count = out.size();
                        best_out = std::move(out);
                    }
                }
                return best_out;
            }

            std::vector<Swath> out;
            BPolygon bounds = utils::to_boost(border);
            double rad = angle_deg * M_PI / 180.0;
            BPoint centroid;
            boost::geometry::centroid(bounds, centroid);
            double cx = centroid.x();
            double cy = centroid.y();
            double cosA = std::cos(rad);
            double sinA = std::sin(rad);
            auto bbox = boost::geometry::return_envelope<boost::geometry::model::box<BPoint>>(bounds);
            double width = bbox.max_corner().x() - bbox.min_corner().x();
            double height = bbox.max_corner().y() - bbox.min_corner().y();
            double ext = std::max(width, height) * 2.0;

            for (double offs = -ext; offs <= ext; offs += swath_width) {
                concord::ENU e1{cx + offs * sinA - ext * cosA, cy - offs * cosA - ext * sinA, 0};
                concord::ENU e2{cx + offs * sinA + ext * cosA, cy - offs * cosA + ext * sinA, 0};
                BLineString ray;
                ray.emplace_back(e1.x, e1.y);
                ray.emplace_back(e2.x, e2.y);
                std::vector<BLineString> clips;
                boost::geometry::intersection(ray, bounds, clips);

                for (auto const &seg : clips) {
                    if (boost::geometry::length(seg) < swath_width * 0.1)
                        continue;
                    concord::Point a{concord::ENU{seg.front().x(), seg.front().y(), 0}, datum_};
                    concord::Point b{concord::ENU{seg.back().x(), seg.back().y(), 0}, datum_};
                    out.push_back(create_swath(a, b, SwathType::Swath));
                }
            }

            return out;
        }
    };

} // namespace farmtrax
