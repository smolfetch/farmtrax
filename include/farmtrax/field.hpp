#pragma once

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/expand.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <concord/types_basic.hpp>
#include <concord/types_grid.hpp>
#include <concord/types_line.hpp>
#include <concord/types_polygon.hpp>
#include <entropy/noisegen.hpp>

inline uint8_t float_to_byte(float v, float min = 0.0f, float max = 255.0f) {
    v = std::clamp(v, 0.0f, 1.0f);
    float scaled = v * 255.0f;
    float clamped = std::clamp(scaled, min, max);
    return static_cast<uint8_t>(std::round(clamped));
}

namespace farmtrax {

    bool inline are_colinear(const concord::Point &p1, const concord::Point &p2, const concord::Point &p3,
                             double epsilon = 1e-10) {
        auto geo_p1 = boost::geometry::model::d2::point_xy<double>(p1.enu.x, p1.enu.y);
        auto geo_p2 = boost::geometry::model::d2::point_xy<double>(p2.enu.x, p2.enu.y);
        auto geo_p3 = boost::geometry::model::d2::point_xy<double>(p3.enu.x, p3.enu.y);

        auto area =
            (boost::geometry::get<0>(geo_p1) * (boost::geometry::get<1>(geo_p2) - boost::geometry::get<1>(geo_p3)) +
             boost::geometry::get<0>(geo_p2) * (boost::geometry::get<1>(geo_p3) - boost::geometry::get<1>(geo_p1)) +
             boost::geometry::get<0>(geo_p3) * (boost::geometry::get<1>(geo_p1) - boost::geometry::get<1>(geo_p2))) /
            2.0;

        return std::abs(area) < epsilon;
    }

    concord::Polygon inline remove_colinear_points(concord::Polygon &polygon, double epsilon = 0.01) {
        concord::Polygon new_polygon;
        if (polygon.getPoints().size() < 4)
            return polygon;
        size_t n = polygon.getPoints().size() - 1;
        for (size_t i = 0; i < n; ++i) {
            const auto &prev = polygon.getPoints()[(i + n - 1) % n];
            const auto &curr = polygon.getPoints()[i];
            const auto &next = polygon.getPoints()[(i + 1) % n];
            if (!are_colinear(prev, curr, next, epsilon)) {
                new_polygon.addPoint(curr);
            }
        }
        if (!polygon.getPoints().empty()) {
            new_polygon.addPoint(polygon.getPoints().front());
        }
        return new_polygon;
    }

    struct Ring {
        concord::Polygon polygon;
        std::string uuid;
    };

    struct Swath {
        concord::Line line;
        std::string uuid;

        void flip() {
            concord::Line reversed_swath = line;
            reversed_swath.setStart(line.getEnd());
            reversed_swath.setEnd(line.getStart());
            line = reversed_swath;
        }
    };

    Swath inline create_swath(const concord::Point &start, const concord::Point &end, std::string uuid = "") {
        concord::Line line;
        line.setStart(start);
        line.setEnd(end);
        std::string uuid_ = uuid.empty() ? boost::uuids::to_string(boost::uuids::random_generator()()) : uuid;
        return {line, uuid_};
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

      private:
        double resolution_;
        entropy::NoiseGen noise;
        std::mt19937 rnd;
        concord::Datum datum_;

      public:
        Field() = default;

        Field(const concord::Polygon coordinates, double resolution, concord::Datum datum, bool centred = true)
            : resolution_(resolution), datum_(datum) {
            grids_.emplace_back(coordinates, resolution, datum, centred);
        }

        Grid &get_grid(std::size_t i) {
            if (i >= grids_.size()) {
                throw std::out_of_range("Grid index out of range");
            }
            return grids_[i];
        }

        const Grid &get_grid(std::size_t i) const {
            if (i >= grids_.size()) {
                throw std::out_of_range("Grid index out of range");
            }
            return grids_[i];
        }

        std::vector<Part> get_parts() const { return parts_; }

        void add_noise(std::size_t i = 0) {
            Grid &grid = get_grid(i);
            noise.SetNoiseType(entropy::NoiseGen::NoiseType_OpenSimplex2);
            noise.SetFrequency(static_cast<float>(resolution_) / 300.0f);
            noise.SetSeed(static_cast<int>(rnd()));
            for (std::size_t r = 0; r < grid.rows(); ++r) {
                for (std::size_t c = 0; c < grid.cols(); ++c) {
                    float n = noise.GetNoise(static_cast<float>(r), static_cast<float>(c));
                    float val = (n + 1.0f) / 2.0f;
                    grid(r, c).second = float_to_byte(val);
                }
            }
        }

      private:
        using BPoint = boost::geometry::model::d2::point_xy<double>;
        using BLineString = boost::geometry::model::linestring<BPoint>;
        using BPolygon = boost::geometry::model::polygon<BPoint>;
        using BMultiPoly = boost::geometry::model::multi_polygon<BPolygon>;

        BPoint to_boost(const concord::Point &in) {
            return boost::geometry::model::d2::point_xy<double>(in.enu.x, in.enu.y);
        }

        concord::Point from_boost(BPoint const &in) {
            auto enu = concord::ENU{in.x(), in.y(), 0.0};
            auto pty = concord::Point{enu, datum_};
            return pty;
        }

        BLineString to_boost(const concord::Line &in) {
            BLineString out;
            out.emplace_back(to_boost(in.getStart()));
            out.emplace_back(to_boost(in.getEnd()));
            return out;
        }

        concord::Line from_boost(BLineString const &in) {
            concord::Line out;
            out.setStart(from_boost(in.front()));
            out.setEnd(from_boost(in.back()));
            return out;
        }

        BPolygon to_boost(const concord::Polygon &in) {
            BPolygon out;
            for (auto const &pt : in.getPoints())
                out.outer().emplace_back(to_boost(pt));
            if (!boost::geometry::equals(out.outer().front(), out.outer().back()))
                out.outer().push_back(out.outer().front());
            boost::geometry::correct(out);
            return out;
        }

        concord::Polygon from_boost(BPolygon const &in) {
            concord::Polygon out;
            for (auto const &pt : in.outer()) {
                out.addPoint(from_boost(pt));
            }
            return out;
        }
    };

} // namespace farmtrax
