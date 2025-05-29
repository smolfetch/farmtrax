#pragma once

#ifdef HAS_RERUN

// #include "concord/types_polygon.hpp"
#include "../field.hpp"
#include "geoson/parser.hpp"
#include "geoson/writter.hpp"
#include "geotiv/parse.hpp"
#include "geotiv/writter.hpp"
#include "rerun.hpp"
#include <rerun/recording_stream.hpp>

namespace farmtrax {
    namespace visualize {

        inline void show_field(const farmtrax::Field &field, std::shared_ptr<rerun::RecordingStream> rec) {

            std::vector<std::vector<std::array<float, 3>>> borders;
            for (auto const &part : field.get_parts()) {
                std::vector<std::array<float, 3>> border_one;
                auto poy = part.border.polygon;
                for (auto p : poy.getPoints()) {
                    std::array<float, 3> v;
                    v[0] = p.enu.x;
                    v[1] = p.enu.y;
                    v[2] = 0.0f;
                    border_one.push_back(v);
                }
                borders.push_back(border_one);
            }
            for (uint i = 0; i < borders.size(); ++i) {
                auto colorz = rerun::Color(255.0f, 0.0f, 0.0f);
                auto border__ = rerun::components::LineStrip3D(borders[i]);
                rec->log_static("/border" + std::to_string(i),
                                rerun::LineStrips3D(border__).with_colors({{colorz}}).with_radii({{0.2f}}));
            }
            // Headlands
            for (uint i = 0; i < field.get_parts().size(); ++i) {
                std::vector<std::vector<std::array<float, 3>>> headlands;
                for (auto const &headland : field.get_parts()[i].headlands) {
                    std::vector<std::array<float, 3>> headland_one;
                    auto poy = headland.polygon;
                    for (auto p : poy.getPoints()) {
                        std::array<float, 3> v;
                        v[0] = p.enu.x;
                        v[1] = p.enu.y;
                        v[2] = 0.0f;
                        headland_one.push_back(v);
                    }
                    headlands.push_back(headland_one);
                }

                for (uint j = 0; j < headlands.size(); ++j) {
                    auto colorz = rerun::Color(0.0f, 255.0f, 0.0f);
                    auto headland__ = rerun::components::LineStrip3D(headlands[j]);
                    rec->log_static("/headland" + std::to_string(i) + "_" + std::to_string(j),
                                    rerun::LineStrips3D(headland__).with_colors({{colorz}}).with_radii({{0.2f}}));
                }
            }
            // Swaths
            for (uint i = 0; i < field.get_parts().size(); ++i) {
                std::vector<std::vector<std::array<float, 3>>> swaths;
                for (auto const &swath : field.get_parts()[i].swaths) {
                    std::vector<std::array<float, 3>> swath_one;
                    std::array<float, 3> v;
                    v[0] = swath.line.getStart().enu.x;
                    v[1] = swath.line.getStart().enu.y;
                    v[2] = 0.0f;
                    swath_one.push_back(v);
                    v[0] = swath.line.getEnd().enu.x;
                    v[1] = swath.line.getEnd().enu.y;
                    v[2] = 0.0f;
                    swath_one.push_back(v);
                    swaths.push_back(swath_one);
                }
                for (uint j = 0; j < swaths.size(); ++j) {
                    auto colorz = rerun::Color(0.0f, 0.0f, 255.0f);
                    auto swath__ = rerun::components::LineStrip3D(swaths[j]);
                    rec->log_static("/swath" + std::to_string(i) + "_" + std::to_string(j),
                                    rerun::LineStrips3D(swath__).with_colors({{colorz}}).with_radii({{0.2f}}));
                }
            }
        }
    } // namespace visualize
} // namespace farmtrax
  //
#endif
