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


        // Overloaded version for Nety class
        inline void show_swath_tour(const farmtrax::Nety &nety, 
                                   const std::vector<farmtrax::Vertex> &path,
                                   std::shared_ptr<rerun::RecordingStream> rec, 
                                   size_t machine_id = 0,
                                   float swath_radius = 0.4f,
                                   float connection_radius = 0.2f) {
            static const std::vector<rerun::Color> palette = {{255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                                                              {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
            
            auto base_color = palette[machine_id % palette.size()];
            auto connection_color = rerun::Color(base_color.r() * 0.6, base_color.g() * 0.6, base_color.b() * 0.6);
            
            const auto& ab_lines = nety.get_ab_lines();
            const auto& graph = nety.get_graph();
            
            // Helper to get vertex position
            auto get_vertex_position = [&](farmtrax::Vertex v) -> std::array<float, 3> {
                auto pos = boost::get(boost::vertex_name, graph)[v];
                return {float(pos.x()), float(pos.y()), 0.3f};
            };
            
            // Helper to find which AB line a vertex belongs to and whether it's A or B endpoint
            auto get_ab_line_info = [&](farmtrax::Vertex v) -> std::pair<size_t, bool> {
                auto pos = boost::get(boost::vertex_name, graph)[v];
                for (size_t i = 0; i < ab_lines.size(); ++i) {
                    const double eps = 1e-6;
                    if (boost::geometry::distance(pos, ab_lines[i].A) < eps) {
                        return {i, true}; // A endpoint of line i
                    }
                    if (boost::geometry::distance(pos, ab_lines[i].B) < eps) {
                        return {i, false}; // B endpoint of line i
                    }
                }
                return {SIZE_MAX, false}; // not found
            };
            
            // Track which AB lines are used and in what order
            std::vector<bool> line_used(ab_lines.size(), false);
            std::vector<std::array<float, 3>> tour_points;
            
            // Process the path to visualize AB lines and connections
            for (size_t i = 0; i < path.size(); ++i) {
                auto [line_idx, is_A] = get_ab_line_info(path[i]);
                
                if (line_idx != SIZE_MAX && !line_used[line_idx]) {
                    // Mark this AB line as used
                    line_used[line_idx] = true;
                    
                    // Visualize the AB line itself
                    const auto& ab_line = ab_lines[line_idx];
                    std::vector<std::array<float, 3>> line_points = {
                        {float(ab_line.A.x()), float(ab_line.A.y()), 0.3f},
                        {float(ab_line.B.x()), float(ab_line.B.y()), 0.3f}
                    };
                    
                    rec->log_static("/tour/machine" + std::to_string(machine_id) + "/ab_line" + std::to_string(line_idx),
                                  rerun::LineStrips3D(rerun::components::LineStrip3D(line_points))
                                      .with_colors({{base_color}})
                                      .with_radii({{swath_radius}}));
                }
                
                // Add vertex position to tour for connection lines
                tour_points.push_back(get_vertex_position(path[i]));
            }
            
            // Visualize connection lines between different AB lines only
            for (size_t i = 0; i < path.size() - 1; ++i) {
                auto [line_idx1, is_A1] = get_ab_line_info(path[i]);
                auto [line_idx2, is_A2] = get_ab_line_info(path[i + 1]);
                
                // Only draw connection if vertices belong to different AB lines
                if (line_idx1 != line_idx2 && line_idx1 != SIZE_MAX && line_idx2 != SIZE_MAX) {
                    std::vector<std::array<float, 3>> connection_segment = {
                        get_vertex_position(path[i]),
                        get_vertex_position(path[i + 1])
                    };
                    
                    rec->log_static("/tour/machine" + std::to_string(machine_id) + "/connection" + std::to_string(i),
                                  rerun::LineStrips3D(rerun::components::LineStrip3D(connection_segment))
                                      .with_colors({{connection_color}})
                                      .with_radii({{connection_radius}}));
                }
            }
            
            // Add start and end markers
            if (!tour_points.empty()) {
                rec->log_static("/tour/machine" + std::to_string(machine_id) + "/start",
                              rerun::Points3D({{tour_points[0][0], tour_points[0][1], tour_points[0][2]}})
                                  .with_colors({{rerun::Color(0, 255, 0)}})  // Green for start
                                  .with_radii({{swath_radius * 2.0f}}));
                                  
                rec->log_static("/tour/machine" + std::to_string(machine_id) + "/end",
                              rerun::Points3D({{tour_points.back()[0], tour_points.back()[1], tour_points.back()[2]}})
                                  .with_colors({{rerun::Color(255, 0, 0)}})  // Red for end
                                  .with_radii({{swath_radius * 2.0f}}));
            }
        }

    } // namespace visualize
} // namespace farmtrax

#endif
