#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
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
#include <boost/geometry/strategies/buffer.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <concord/types_basic.hpp>
#include <concord/types_grid.hpp>
#include <concord/types_line.hpp>
#include <concord/types_polygon.hpp>
#include <entropy/noisegen.hpp>

#include "farmtrax/utils/utils.hpp"

namespace farmtrax {
    struct Ring {
        concord::Polygon polygon;
        std::string uuid;
    };

    struct Swath {
        concord::Line line;
        std::string uuid;
        void flip() {
            concord::Line tmp = line;
            tmp.setStart(line.getEnd());
            tmp.setEnd(line.getStart());
            line = tmp;
        }
    };

    inline Swath create_swath(const concord::Point &start, const concord::Point &end, std::string uuid = "") {
        concord::Line L;
        L.setStart(start);
        L.setEnd(end);
        if (uuid.empty())
            uuid = boost::uuids::to_string(boost::uuids::random_generator()());
        return {L, uuid};
    }

    struct Part {
        Ring border;
        std::vector<Swath> swaths;
        std::vector<Ring> headlands;
    };

    using Grid = concord::Grid<uint8_t>;

    class Field {
        std::vector<Grid> grids_;
        std::vector<Part> parts_;
        double resolution_{};

      private:
        concord::Polygon border_;
        concord::Datum datum_{};
        entropy::NoiseGen noiseGen_;
        std::mt19937 rnd_;
        double overlap_threshold_{0.7};

      public:
        Field(const concord::Polygon &border, double resolution, const concord::Datum &datum, bool centred = true,
              double overlap_threshold = 0.7, double area_threshold = 0.5)
            : resolution_(resolution), border_(border), datum_(datum), overlap_threshold_(overlap_threshold) {
            grids_.emplace_back(border_, resolution_, datum_, centred);
            auto divisions = border.split_obb(overlap_threshold_);
            std::cout << "Split " << divisions.size() << " parts\n";
            parts_.reserve(divisions.size());
            for (auto const &poly : divisions) {
                Part p;
                p.border.polygon = poly;
                p.border.uuid = boost::uuids::to_string(boost::uuids::random_generator()());
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

        void gen_field(double swath_width, double angle_degrees, int headland_count = 1) {
            for (auto &part : parts_) {
                part.headlands.clear();
                part.swaths.clear();
                part.headlands = generate_headlands(part.border.polygon, swath_width, headland_count);
                concord::Polygon interior = headland_count > 0 ? part.headlands.back().polygon : part.border.polygon;
                part.swaths = generate_swaths(swath_width, angle_degrees, interior);
            }
        }

      private:
        using BPoint = boost::geometry::model::d2::point_xy<double>;
        using BLineString = boost::geometry::model::linestring<BPoint>;
        using BPolygon = boost::geometry::model::polygon<BPoint>;
        using BMultiPoly = boost::geometry::model::multi_polygon<BPolygon>;

        BPoint to_boost(const concord::Point &in) const { return BPoint{in.enu.x, in.enu.y}; }

        concord::Point from_boost(const BPoint &in) const {
            concord::ENU pt{in.x(), in.y(), 0.0};
            return concord::Point{pt, datum_};
        }

        BLineString to_boost(const concord::Line &L) const {
            BLineString out;
            out.emplace_back(to_boost(L.getStart()));
            out.emplace_back(to_boost(L.getEnd()));
            return out;
        }

        concord::Line from_boost(const BLineString &L) const {
            concord::Line out;
            out.setStart(from_boost(L.front()));
            out.setEnd(from_boost(L.back()));
            return out;
        }

        BPolygon to_boost(const concord::Polygon &poly) const {
            BPolygon out;
            for (auto const &pt : poly.getPoints())
                out.outer().emplace_back(to_boost(pt));
            if (!boost::geometry::equals(out.outer().front(), out.outer().back()))
                out.outer().push_back(out.outer().front());
            boost::geometry::correct(out);
            return out;
        }

        concord::Polygon from_boost(const BPolygon &poly) const {
            concord::Polygon out;
            for (auto const &pt : poly.outer())
                out.addPoint(from_boost(pt));
            if (!out.getPoints().empty())
                out.addPoint(from_boost(poly.outer().front()));
            return out;
        }

        // Fixed: now takes a polygon parameter instead of using member variable
        std::vector<Ring> generate_headlands(const concord::Polygon &polygon, double shrink_dist, int count) const {
            if (shrink_dist <= 0 || count <= 0)
                return {};
            if (shrink_dist < 0)
                throw std::invalid_argument("negative shrink");

            std::vector<Ring> H;
            BPolygon base = to_boost(polygon);

            for (int i = 0; i < count; ++i) {
                BPolygon current = (i == 0 ? base : to_boost(H.back().polygon));
                BMultiPoly buf;
                boost::geometry::strategy::buffer::distance_symmetric<double> dist(-shrink_dist);
                boost::geometry::strategy::buffer::side_straight side;
                boost::geometry::strategy::buffer::join_miter join;
                boost::geometry::strategy::buffer::end_flat end;
                boost::geometry::strategy::buffer::point_square point;

                try {
                    boost::geometry::buffer(current, buf, dist, side, join, end, point);
                } catch (const std::exception &e) {
                    std::cerr << "Buffer operation failed: " << e.what() << std::endl;
                    break;
                }

                if (buf.empty()) {
                    std::cerr << "Warning: empty buffer result at headland " << i << std::endl;
                    break;
                }

                const BPolygon *best = &buf.front();
                double maxA = boost::geometry::area(*best);
                for (auto const &cand : buf) {
                    double a = boost::geometry::area(cand);
                    if (a > maxA) {
                        maxA = a;
                        best = &cand;
                    }
                }

                if (maxA <= 0) {
                    std::cerr << "Warning: zero area polygon at headland " << i << std::endl;
                    break;
                }

                concord::Polygon tmp = from_boost(*best);
                concord::Polygon simp = utils::remove_colinear_points(tmp, 1e-4);
                H.push_back({std::move(simp), boost::uuids::to_string(boost::uuids::random_generator()())});
            }
            return H;
        }

        std::vector<Swath> generate_swaths(const std::vector<Swath> &templ, const concord::Polygon &field) const {
            std::vector<Swath> out;
            BPolygon Bf = to_boost(field);
            for (auto const &s : templ) {
                BLineString L = to_boost(s.line);
                std::vector<BLineString> clips;
                boost::geometry::intersection(L, Bf, clips);
                for (auto const &seg : clips) {
                    if (boost::geometry::length(seg) < 1e-6)
                        continue;
                    concord::Point a = from_boost(seg.front());
                    concord::Point b = from_boost(seg.back());
                    out.push_back(create_swath(a, b, s.uuid));
                }
            }
            return out;
        }

        std::vector<Swath> generate_swaths(double swath_width, double angle_deg, const concord::Polygon &border) const {
            std::vector<Swath> out;
            BPolygon bounds = to_boost(border);
            double rad = angle_deg * M_PI / 180.0;

            // Compute polygon centroid more robustly
            BPoint centroid;
            boost::geometry::centroid(bounds, centroid);
            double cx = centroid.x();
            double cy = centroid.y();

            double cosA = std::cos(rad), sinA = std::sin(rad);

            // Calculate a more reasonable extension based on polygon bounds
            auto bbox = boost::geometry::return_envelope<boost::geometry::model::box<BPoint>>(bounds);
            double width = bbox.max_corner().x() - bbox.min_corner().x();
            double height = bbox.max_corner().y() - bbox.min_corner().y();
            double ext = std::max(width, height) * 2.0; // More reasonable extension

            for (double offs = -ext; offs <= ext; offs += swath_width) {
                concord::ENU e1{cx + offs * sinA - ext * cosA, cy - offs * cosA - ext * sinA, 0};
                concord::ENU e2{cx + offs * sinA + ext * cosA, cy - offs * cosA + ext * sinA, 0};
                BLineString ray;
                ray.emplace_back(e1.x, e1.y);
                ray.emplace_back(e2.x, e2.y);
                std::vector<BLineString> clips;
                boost::geometry::intersection(ray, bounds, clips);
                for (auto const &seg : clips) {
                    if (boost::geometry::length(seg) < swath_width * 0.1) // More reasonable minimum length
                        continue;
                    concord::Point a{concord::ENU{seg.front().x(), seg.front().y(), 0}, datum_};
                    concord::Point b{concord::ENU{seg.back().x(), seg.back().y(), 0}, datum_};
                    out.push_back(create_swath(a, b));
                }
            }
            return out;
        }
    };

} // namespace farmtrax
