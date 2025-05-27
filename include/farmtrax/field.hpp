#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/area.hpp>
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

namespace farmtrax {

    /**
     * @brief Convert [0,1] float to byte in [min,max].
     */
    inline uint8_t float_to_byte(float v, float min = 0.0f, float max = 255.0f) {
        v = std::clamp(v, 0.0f, 1.0f);
        float scaled = v * 255.0f;
        float clamped = std::clamp(scaled, min, max);
        return static_cast<uint8_t>(std::round(clamped));
    }

    /**
     * @brief Return true if three ENU points are colinear within ε.
     */
    inline bool are_colinear(const concord::Point &p1, const concord::Point &p2, const concord::Point &p3,
                             double epsilon = 1e-10) {
        using BPoint = boost::geometry::model::d2::point_xy<double>;
        BPoint a{p1.enu.x, p1.enu.y}, b{p2.enu.x, p2.enu.y}, c{p3.enu.x, p3.enu.y};
        double area = (boost::geometry::get<0>(a) * (boost::geometry::get<1>(b) - boost::geometry::get<1>(c)) +
                       boost::geometry::get<0>(b) * (boost::geometry::get<1>(c) - boost::geometry::get<1>(a)) +
                       boost::geometry::get<0>(c) * (boost::geometry::get<1>(a) - boost::geometry::get<1>(b))) *
                      0.5;
        return std::abs(area) < epsilon;
    }

    /**
     * @brief Remove colinear vertices from a closed concord::Polygon.
     */
    inline concord::Polygon remove_colinear_points(const concord::Polygon &polygon, double epsilon = 0.01) {
        concord::Polygon result;
        auto const &pts = polygon.getPoints();
        if (pts.size() < 4)
            return polygon;
        size_t n = pts.size() - 1;
        for (size_t i = 0; i < n; ++i) {
            auto const &prev = pts[(i + n - 1) % n];
            auto const &curr = pts[i];
            auto const &next = pts[(i + 1) % n];
            if (!are_colinear(prev, curr, next, epsilon)) {
                result.addPoint(curr);
            }
        }
        result.addPoint(pts.front());
        return result;
    }

    struct Ring {
        concord::Polygon polygon;
        std::string uuid;
    };

    struct Swath {
        concord::Line line;
        std::string uuid;

        /**
         * @brief Reverse this swath’s direction.
         */
        void flip() {
            concord::Line tmp = line;
            tmp.setStart(line.getEnd());
            tmp.setEnd(line.getStart());
            line = tmp;
        }
    };

    /**
     * @brief Build a 2-point Swath, generating a UUID if none given.
     */
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
        std::mt19937 rnd_{};

      public:
        Field() = default;

        Field(const concord::Polygon &border, double resolution, const concord::Datum &datum, bool centred = true)
            : resolution_(resolution), border_(border), datum_(datum) {
            grids_.emplace_back(border_, resolution_, datum_, centred);
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

        /**
         * @brief Fill grid i with procedural noise.
         */
        void add_noise(std::size_t i = 0) {
            Grid &g = get_grid(i);
            noiseGen_.SetNoiseType(entropy::NoiseGen::NoiseType_OpenSimplex2);
            noiseGen_.SetFrequency(static_cast<float>(resolution_) / 300.0f);
            noiseGen_.SetSeed(static_cast<int>(rnd_()));
            for (std::size_t r = 0; r < g.rows(); ++r) {
                for (std::size_t c = 0; c < g.cols(); ++c) {
                    float n = noiseGen_.GetNoise(static_cast<float>(r), static_cast<float>(c));
                    float val = (n + 1.0f) * 0.5f;
                    g(r, c).second = float_to_byte(val);
                }
            }
        }

        /**
         * @brief Generate headlands & swaths from a border polygon.
         */
        void gen_field(double swath_width, double angle_degrees, int headland_count = 0) {
            Part P;
            P.border.polygon = border_;
            P.border.uuid = boost::uuids::to_string(boost::uuids::random_generator()());
            P.headlands = generate_headlands(swath_width, headland_count);
            concord::Polygon interior = headland_count > 0 ? P.headlands.back().polygon : border_;
            P.swaths = generate_swaths(swath_width, angle_degrees, interior);
            parts_.push_back(std::move(P));
        }

        /**
         * @brief Clip a template of swaths into the shrunken interior.
         */
        void gen_field(const std::vector<Swath> &template_swaths, double swath_width, int headland_count) {
            Part P;
            P.border.polygon = border_;
            P.border.uuid = boost::uuids::to_string(boost::uuids::random_generator()());
            P.headlands = generate_headlands(swath_width, headland_count);
            concord::Polygon interior = P.headlands.back().polygon;
            P.swaths = generate_swaths(template_swaths, interior);
            parts_.push_back(std::move(P));
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
            out.addPoint(from_boost(poly.outer().front()));
            return out;
        }

        std::vector<Ring> generate_headlands(double shrink_dist, int count) const {
            if (shrink_dist < 0)
                throw std::invalid_argument("negative shrink");
            std::vector<Ring> H;
            BPolygon base = to_boost(border_);

            for (int i = 0; i < count; ++i) {
                BPolygon current = (i == 0 ? base : to_boost(H.back().polygon));
                BMultiPoly buf;
                boost::geometry::strategy::buffer::distance_symmetric<double> dist(-shrink_dist);
                boost::geometry::strategy::buffer::side_straight side;
                boost::geometry::strategy::buffer::join_miter join;
                boost::geometry::strategy::buffer::end_flat end;
                boost::geometry::strategy::buffer::point_square point;
                boost::geometry::buffer(current, buf, dist, side, join, end, point);

                if (buf.empty())
                    throw std::runtime_error("empty after buffer");

                const BPolygon *best = &buf.front();
                double maxA = boost::geometry::area(*best);
                for (auto const &cand : buf) {
                    double a = boost::geometry::area(cand);
                    if (a > maxA) {
                        maxA = a;
                        best = &cand;
                    }
                }

                concord::Polygon tmp = from_boost(*best);
                concord::Polygon simp = remove_colinear_points(tmp, 1e-4);
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
            double cx = 0, cy = 0;
            auto const &pts = border.getPoints();
            for (auto const &pt : pts) {
                cx += pt.enu.x;
                cy += pt.enu.y;
            }
            cx /= pts.size();
            cy /= pts.size();

            double cosA = std::cos(rad), sinA = std::sin(rad);
            double ext = swath_width * 1000.0;

            for (double offs = -ext; offs <= ext; offs += swath_width) {
                concord::ENU e1{cx + offs * sinA - ext * cosA, cy - offs * cosA - ext * sinA, 0};
                concord::ENU e2{cx + offs * sinA + ext * cosA, cy - offs * cosA + ext * sinA, 0};

                BLineString ray;
                ray.emplace_back(e1.x, e1.y);
                ray.emplace_back(e2.x, e2.y);

                std::vector<BLineString> clips;
                boost::geometry::intersection(ray, bounds, clips);

                for (auto const &seg : clips) {
                    if (boost::geometry::length(seg) < swath_width)
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
