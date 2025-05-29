#pragma once

#ifdef HAS_RERUN

#include "../divy.hpp"
#include "../field.hpp"
#include "rerun.hpp"
#include <array>
#include <rerun/recording_stream.hpp>
#include <vector>

namespace farmtrax {
    namespace visualize {

        /**
         * @brief Visualize a Field’s parts (borders, headlands, swaths).
         */
        inline void show_field(const farmtrax::Field &field, std::shared_ptr<rerun::RecordingStream> rec) {
            for (size_t i = 0; i < field.get_parts().size(); ++i) {
                std::vector<std::array<float, 3>> pts;
                for (auto const &p : field.get_parts()[i].border.polygon.getPoints())
                    pts.push_back({{float(p.enu.x), float(p.enu.y), 0.0f}});
                auto c = rerun::Color(120, 70, 70);
                rec->log_static(
                    "/border" + std::to_string(i),
                    rerun::LineStrips3D(rerun::components::LineStrip3D(pts)).with_colors({{c}}).with_radii({{0.2f}}));
            }

            for (size_t i = 0; i < field.get_parts().size(); ++i) {
                for (size_t j = 0; j < field.get_parts()[i].headlands.size(); ++j) {
                    std::vector<std::array<float, 3>> pts;
                    for (auto const &p : field.get_parts()[i].headlands[j].polygon.getPoints())
                        pts.push_back({{float(p.enu.x), float(p.enu.y), 0.0f}});
                    auto c = rerun::Color(70, 120, 70);
                    rec->log_static("/headland" + std::to_string(i) + "_" + std::to_string(j),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D(pts))
                                        .with_colors({{c}})
                                        .with_radii({{0.2f}}));
                }
            }

            for (size_t i = 0; i < field.get_parts().size(); ++i) {
                for (size_t j = 0; j < field.get_parts()[i].swaths.size(); ++j) {
                    auto const &s = field.get_parts()[i].swaths[j];
                    std::vector<std::array<float, 3>> pts = {
                        {{float(s.line.getStart().enu.x), float(s.line.getStart().enu.y), 0.0f},
                         {float(s.line.getEnd().enu.x), float(s.line.getEnd().enu.y), 0.0f}}};
                    auto c = rerun::Color(70, 70, 120);
                    rec->log_static("/swath" + std::to_string(i) + "_" + std::to_string(j),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D(pts))
                                        .with_colors({{c}})
                                        .with_radii({{0.2f}}));
                }
            }
        }

        /**
         * @brief Visualize a Divy’s latest division by coloring each machine’s assignments.
         */
        inline void show_divisions(const Divy &divy, std::shared_ptr<rerun::RecordingStream> rec, float radius = 0.2f) {
            static const std::vector<rerun::Color> palette = {{255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                                                              {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};

            const auto &div = divy.result();
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
