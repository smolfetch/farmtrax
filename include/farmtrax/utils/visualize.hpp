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

        inline void show_optimized_path(const farmtrax::SwathNetwork &network, 
                                      const std::vector<farmtrax::Vertex> &path,
                                      std::shared_ptr<rerun::RecordingStream> rec, 
                                      size_t machine_id = 0,
                                      float radius = 0.3f) {
            static const std::vector<rerun::Color> palette = {{255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                                                              {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
            
            auto color = palette[machine_id % palette.size()];
            
            // Get vertex positions from the graph
            auto get_vertex_position = [&](farmtrax::Vertex v) -> std::array<float, 3> {
                auto pos = boost::get(boost::vertex_name, network.get_graph())[v];
                return {float(pos.x()), float(pos.y()), 0.2f};
            };
            
            // Visualize the path as a connected line
            std::vector<std::array<float, 3>> path_points;
            for (const auto& vertex : path) {
                path_points.push_back(get_vertex_position(vertex));
            }
            
            if (!path_points.empty()) {
                rec->log_static("/optimized_path/machine" + std::to_string(machine_id),
                              rerun::LineStrips3D(rerun::components::LineStrip3D(path_points))
                                  .with_colors({{color}})
                                  .with_radii({{radius}}));
            }
            
            // Visualize individual vertices as points
            for (size_t i = 0; i < path.size(); ++i) {
                auto pos = get_vertex_position(path[i]);
                rec->log_static("/optimized_path/vertex" + std::to_string(machine_id) + "_" + std::to_string(i),
                              rerun::Points3D({{pos[0], pos[1], pos[2]}})
                                  .with_colors({{color}})
                                  .with_radii({{radius * 1.5f}}));
            }
        }

        inline void show_swath_tour(const farmtrax::SwathNetwork &network, 
                                   const std::vector<farmtrax::Vertex> &path,
                                   std::shared_ptr<rerun::RecordingStream> rec, 
                                   size_t machine_id = 0,
                                   float swath_radius = 0.4f,
                                   float connection_radius = 0.2f) {
            static const std::vector<rerun::Color> palette = {{255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                                                              {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
            
            auto base_color = palette[machine_id % palette.size()];
            auto connection_color = rerun::Color(base_color.r() * 0.6, base_color.g() * 0.6, base_color.b() * 0.6);
            
            const auto& swaths = network.get_swaths();
            const auto& graph = network.get_graph();
            
            // Helper to get vertex position
            auto get_vertex_position = [&](farmtrax::Vertex v) -> std::array<float, 3> {
                auto pos = boost::get(boost::vertex_name, graph)[v];
                return {float(pos.x()), float(pos.y()), 0.3f};
            };
            
            // Helper to find which swath a vertex belongs to and whether it's start or end
            auto get_swath_info = [&](farmtrax::Vertex v) -> std::pair<size_t, bool> {
                auto pos = boost::get(boost::vertex_name, graph)[v];
                for (size_t i = 0; i < swaths.size(); ++i) {
                    auto start_pos = swaths[i]->b_line.front();
                    auto end_pos = swaths[i]->b_line.back();
                    const double eps = 1e-6;
                    if (boost::geometry::distance(pos, start_pos) < eps) {
                        return {i, true}; // start of swath i
                    }
                    if (boost::geometry::distance(pos, end_pos) < eps) {
                        return {i, false}; // end of swath i
                    }
                }
                return {SIZE_MAX, false}; // not found
            };
            
            // Track which swaths are used and in what order
            std::vector<bool> swath_used(swaths.size(), false);
            std::vector<std::array<float, 3>> tour_points;
            
            // Process the path to visualize swaths and connections
            for (size_t i = 0; i < path.size(); ++i) {
                auto [swath_idx, is_start] = get_swath_info(path[i]);
                
                if (swath_idx != SIZE_MAX && !swath_used[swath_idx]) {
                    // Mark this swath as used
                    swath_used[swath_idx] = true;
                    
                    // Visualize the swath itself
                    const auto& swath = swaths[swath_idx];
                    std::vector<std::array<float, 3>> swath_points = {
                        {float(swath->line.getStart().enu.x), float(swath->line.getStart().enu.y), 0.3f},
                        {float(swath->line.getEnd().enu.x), float(swath->line.getEnd().enu.y), 0.3f}
                    };
                    
                    rec->log_static("/tour/machine" + std::to_string(machine_id) + "/swath" + std::to_string(swath_idx),
                                  rerun::LineStrips3D(rerun::components::LineStrip3D(swath_points))
                                      .with_colors({{base_color}})
                                      .with_radii({{swath_radius}}));
                }
                
                // Add vertex position to tour for connection lines
                tour_points.push_back(get_vertex_position(path[i]));
            }
            
            // Visualize connection lines between different swaths only
            for (size_t i = 0; i < path.size() - 1; ++i) {
                auto [swath_idx1, is_start1] = get_swath_info(path[i]);
                auto [swath_idx2, is_start2] = get_swath_info(path[i + 1]);
                
                // Only draw connection if vertices belong to different swaths
                if (swath_idx1 != swath_idx2 && swath_idx1 != SIZE_MAX && swath_idx2 != SIZE_MAX) {
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
