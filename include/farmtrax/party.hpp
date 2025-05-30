#pragma once

#include "concord/concord.hpp"

namespace farmtrax {

    class Partitioner {
        concord::Polygon border_;

      public:
        std::vector<concord::Polygon> polygons_;

        Partitioner() = default;
        Partitioner(concord::Polygon poly) : border_(poly) {}

        std::vector<concord::Polygon> partition(double area_threshold) {
            polygons_.push_back(border_);
            return polygons_;
        }
    };

} // namespace farmtrax
