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
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <concord/types_basic.hpp>
#include <concord/types_polygon.hpp>

namespace farmtrax {
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
    } // namespace utils

} // namespace farmtrax