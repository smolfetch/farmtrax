#pragma once

#ifdef HAS_RERUN

#include "../divy.hpp"
#include "../field.hpp"
#include "../graph.hpp"
#include "rerun.hpp"
#include <array>
#include <rerun/recording_stream.hpp>
#include <vector>

namespace farmtrax {
    namespace visualize {

        inline void show_field(const farmtrax::Field &field, std::shared_ptr<rerun::RecordingStream> rec) {
            for (size_t i = 0; i < field.get_parts().size(); ++i) {
                std::vector<std::array<float, 3>> pts;
                for (auto const &p : field.get_parts()[i].border.polygon.getPoints())
                    pts.push_back({float(p.enu.x), float(p.enu.y), 0.0f});
                rec->log_static("/border" + std::to_string(i), rerun::LineStrips3D(rerun::components::LineStrip3D(pts))
                                                                   .with_colors({{rerun::Color(120, 70, 70)}})
                                                                   .with_radii({{0.2f}}));
            }
            for (size_t i = 0; i < field.get_parts().size(); ++i) {
                for (size_t j = 0; j < field.get_parts()[i].headlands.size(); ++j) {
                    std::vector<std::array<float, 3>> pts;
                    for (auto const &p : field.get_parts()[i].headlands[j].polygon.getPoints())
                        pts.push_back({float(p.enu.x), float(p.enu.y), 0.0f});
                    rec->log_static("/headland" + std::to_string(i) + "_" + std::to_string(j),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D(pts))
                                        .with_colors({{rerun::Color(70, 120, 70)}})
                                        .with_radii({{0.2f}}));
                }
            }
            for (size_t i = 0; i < field.get_parts().size(); ++i) {
                for (size_t j = 0; j < field.get_parts()[i].swaths.size(); ++j) {
                    auto const &s = field.get_parts()[i].swaths[j];
                    std::vector<std::array<float, 3>> pts = {
                        {float(s.line.getStart().enu.x), float(s.line.getStart().enu.y), 0.0f},
                        {float(s.line.getEnd().enu.x), float(s.line.getEnd().enu.y), 0.0f}};
                    rec->log_static("/swath" + std::to_string(i) + "_" + std::to_string(j),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D(pts))
                                        .with_colors({{rerun::Color(70, 70, 120)}})
                                        .with_radii({{0.2f}}));
                }
            }
        }

        inline void show_divisions(const Divy &divy, std::shared_ptr<rerun::RecordingStream> rec, float radius = 0.2f) {
            static const std::vector<rerun::Color> palette = {{255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                                                              {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
            auto const &div = divy.result();
            for (auto const &kv : div.headlands_per_machine) {
                size_t m = kv.first;
                auto const &headlands = kv.second;
                auto color = palette[m % palette.size()];
                for (size_t j = 0; j < headlands.size(); ++j) {
                    auto ring_ptr = headlands[j];
                    std::vector<std::array<float, 3>> pts;
                    for (auto const &p : ring_ptr->polygon.getPoints())
                        pts.push_back({float(p.enu.x), float(p.enu.y), 0.1f});
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
                    auto sw_ptr = swaths[j];
                    std::vector<std::array<float, 3>> pts = {
                        {float(sw_ptr->line.getStart().enu.x), float(sw_ptr->line.getStart().enu.y), 0.1f},
                        {float(sw_ptr->line.getEnd().enu.x), float(sw_ptr->line.getEnd().enu.y), 0.0f}};
                    rec->log_static("/division/swath" + std::to_string(m) + "_" + std::to_string(j),
                                    rerun::LineStrips3D(rerun::components::LineStrip3D(pts))
                                        .with_colors({{color}})
                                        .with_radii({{radius}}));
                }
            }
        }


        // Version that takes swaths directly (assumes they are already in the correct traversal order)
        inline void show_swath_tour(const std::vector<std::shared_ptr<const Swath>> &swaths,
                                   std::shared_ptr<rerun::RecordingStream> rec, 
                                   size_t machine_id = 0,
                                   float swath_radius = 0.4f,
                                   float connection_radius = 0.2f) {
            static const std::vector<rerun::Color> palette = {{255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                                                              {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
            
            auto base_color = palette[machine_id % palette.size()];
            auto connection_color = rerun::Color(base_color.r() * 0.6, base_color.g() * 0.6, base_color.b() * 0.6);
            
            if (swaths.empty()) return;
            
            // Visualize each swath based on its type
            for (size_t i = 0; i < swaths.size(); ++i) {
                const auto& swath = swaths[i];
                
                // Choose color and radius based on swath type
                rerun::Color swath_color;
                float radius;
                std::string prefix;
                
                if (swath->type == SwathType::Connection) {
                    swath_color = connection_color;
                    radius = connection_radius;
                    prefix = "connection";
                } else {
                    swath_color = base_color;
                    radius = swath_radius;
                    prefix = "swath";
                }
                
                // Visualize the swath line
                std::vector<std::array<float, 3>> line_points = {
                    {float(swath->getHead().enu.x), float(swath->getHead().enu.y), 0.3f},
                    {float(swath->getTail().enu.x), float(swath->getTail().enu.y), 0.3f}
                };
                
                rec->log_static("/tour/machine" + std::to_string(machine_id) + "/" + prefix + std::to_string(i),
                              rerun::LineStrips3D(rerun::components::LineStrip3D(line_points))
                                  .with_colors({{swath_color}})
                                  .with_radii({{radius}}));
            }
        }

        // Convenience overload that uses the reordered swaths from Nety
        inline void show_swath_tour(const farmtrax::Nety &nety,
                                   std::shared_ptr<rerun::RecordingStream> rec, 
                                   size_t machine_id = 0,
                                   float swath_radius = 0.4f,
                                   float connection_radius = 0.2f) {
            show_swath_tour(nety.get_swaths(), rec, machine_id, swath_radius, connection_radius);
        }

    } // namespace visualize
} // namespace farmtrax

#endif
