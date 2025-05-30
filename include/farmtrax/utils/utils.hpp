#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <concord/types_basic.hpp>
#include <concord/types_polygon.hpp>
#include <concord/types_line.hpp>

namespace farmtrax {
    // Type definitions (same as in field.hpp)
    using BPoint = boost::geometry::model::d2::point_xy<double>;
    using BLineString = boost::geometry::model::linestring<BPoint>;
    using BPolygon = boost::geometry::model::polygon<BPoint>;

    namespace utils{
    inline uint8_t float_to_byte(float v, float min = 0.0f, float max = 255.0f) {
        v = std::clamp(v, 0.0f, 1.0f);
        float scaled = v * 255.0f;
        float clamped = std::clamp(scaled, min, max);
        return static_cast<uint8_t>(std::round(clamped));
    }

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
            if (!are_colinear(prev, curr, next, epsilon))
                result.addPoint(curr);
        }
        result.addPoint(pts.front());
        return result;
    }

    // Conversion functions between concord and boost geometry types
    inline BPoint to_boost(const concord::Point &in) { 
        return BPoint{in.enu.x, in.enu.y}; 
    }

    inline concord::Point from_boost(const BPoint &in, const concord::Datum &datum = concord::Datum{}) {
        concord::ENU pt{in.x(), in.y(), 0.0};
        return concord::Point{pt, datum};
    }

    inline BLineString to_boost(const concord::Line &L) {
        BLineString out;
        out.emplace_back(to_boost(L.getStart()));
        out.emplace_back(to_boost(L.getEnd()));
        return out;
    }

    inline concord::Line from_boost(const BLineString &L, const concord::Datum &datum = concord::Datum{}) {
        concord::Line out;
        out.setStart(from_boost(L.front(), datum));
        out.setEnd(from_boost(L.back(), datum));
        return out;
    }

    inline BPolygon to_boost(const concord::Polygon &poly) {
        BPolygon out;
        for (auto const &pt : poly.getPoints())
            out.outer().emplace_back(to_boost(pt));
        if (!boost::geometry::equals(out.outer().front(), out.outer().back()))
            out.outer().push_back(out.outer().front());
        boost::geometry::correct(out);
        return out;
    }

    inline concord::Polygon from_boost(const BPolygon &poly, const concord::Datum &datum = concord::Datum{}) {
        concord::Polygon out;
        for (auto const &pt : poly.outer())
            out.addPoint(from_boost(pt, datum));
        if (!out.getPoints().empty())
            out.addPoint(from_boost(poly.outer().front(), datum));
        return out;
    }
    } // namespace utils

} // namespace farmtrax