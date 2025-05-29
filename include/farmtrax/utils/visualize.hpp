#pragma once

#ifdef HAS_RERUN

#include "../divy.hpp"
#include "../field.hpp"
#include "rerun.hpp"
#include <array>
#include <rerun/recording_stream.hpp>
#include <unordered_map>
#include <vector>

namespace farmtrax {
    namespace visualize {

        /**
         * @brief Visualize a Field’s parts (borders, headlands, swaths).
         */
        inline void show_field(const farmtrax::Field &field, std::shared_ptr<rerun::RecordingStream> rec) {
            int mc = 120, sc = 70;
            std::vector<std::vector<std::array<float, 3>>> borders;
            for (auto const &part : field.get_parts()) {
                std::vector<std::array<float, 3>> pts;
                for (auto const &p : part.border.polygon.getPoints())
                    pts.push_back({{float(p.enu.x), float(p.enu.y), 0.0f}});
                borders.push_back(pts);
            }
            for (size_t i = 0; i < borders.size(); ++i) {
                auto c = rerun::Color(mc, sc, sc);
                rec->log_static("/border" + std::to_string(i),
                                rerun::LineStrips3D(rerun::components::LineStrip3D(borders[i]))
                                    .with_colors({{c}})
                                    .with_radii({{0.2f}}));
            }

            for (size_t i = 0; i < field.get_parts().size(); ++i) {
                std::vector<std::vector<std::array<float, 3>>> headlands;
                for (auto const &h : field.get_parts()[i].headlands) {
                    std::vector<std::array<float, 3>> pts;
                    for (auto const &p : h.polygon.getPoints())
                        pts.push_back({{float(p.enu.x), float(p.enu.y), 0.0f}});
                    headlands.push_back(pts);
                }
                for (size_t j = 0; j < headlands.size(); ++j) {
                    auto c = rerun::Color(sc, mc, sc);
                    rec->log_static("/headland" + std::to_string(i) + "_" + std::to_string(j),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D(headlands[j]))
                                        .with_colors({{c}})
                                        .with_radii({{0.2f}}));
                }
            }

            for (size_t i = 0; i < field.get_parts().size(); ++i) {
                std::vector<std::vector<std::array<float, 3>>> swaths;
                for (auto const &s : field.get_parts()[i].swaths) {
                    std::vector<std::array<float, 3>> pts = {
                        {{float(s.line.getStart().enu.x), float(s.line.getStart().enu.y), 0.0f},
                         {float(s.line.getEnd().enu.x), float(s.line.getEnd().enu.y), 0.0f}}};
                    swaths.push_back(pts);
                }
                for (size_t j = 0; j < swaths.size(); ++j) {
                    auto c = rerun::Color(sc, sc, mc);
                    rec->log_static("/swath" + std::to_string(i) + "_" + std::to_string(j),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D(swaths[j]))
                                        .with_colors({{c}})
                                        .with_radii({{0.2f}}));
                }
            }
        }

        /**
         * @brief Visualize a DivisionResult by coloring each machine’s assignments.
         */
        inline void show_divisions(const DivisionResult &div, std::shared_ptr<rerun::RecordingStream> rec,
                                   float radius = 0.2f) {
            static const std::vector<rerun::Color> palette = {{255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                                                              {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};

            for (auto const &kv : div.headlands_per_machine) {
                size_t m = kv.first;
                auto const &headlands = kv.second;
                auto color = palette[m % palette.size()];
                for (size_t j = 0; j < headlands.size(); ++j) {
                    std::vector<std::array<float, 3>> pts;
                    for (auto const &p : headlands[j].polygon.getPoints())
                        pts.push_back({{float(p.enu.x), float(p.enu.y), 0.1f}});
                    rec->log_static("/division/headland" + std::to_string(m) + "_" + std::to_string(j),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D(pts))
                                        .with_colors({{color}})
                                        .with_radii({{radius}}));
                }
            }

            for (auto const &kv : div.swaths_per_machine) {
                size_t m = kv.first;
                auto const &swaths = kv.second;
                auto color = palette[m % palette.size()];
                for (size_t j = 0; j < swaths.size(); ++j) {
                    std::vector<std::array<float, 3>> pts = {
                        {{float(swaths[j].line.getStart().enu.x), float(swaths[j].line.getStart().enu.y), 0.1f},
                         {float(swaths[j].line.getEnd().enu.x), float(swaths[j].line.getEnd().enu.y), 0.0f}}};
                    rec->log_static("/division/swath" + std::to_string(m) + "_" + std::to_string(j),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D(pts))
                                        .with_colors({{color}})
                                        .with_radii({{radius}}));
                }
            }
        }

    } // namespace visualize
} // namespace farmtrax

#endif
