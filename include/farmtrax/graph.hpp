#pragma once

#include "farmtrax/field.hpp"
#include "farmtrax/utils/utils.hpp"
#include <boost/geometry/index/rtree.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace farmtrax {

    using VertexProps = boost::property<boost::vertex_name_t, farmtrax::BPoint>;
    using EdgeProps = boost::property<boost::edge_weight_t, double>;
    using Graph = boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, VertexProps, EdgeProps>;
    using Vertex = Graph::vertex_descriptor;
    using PointRTreeValue = std::pair<farmtrax::BPoint, std::size_t>;
    using EndpointTree = boost::geometry::index::rtree<PointRTreeValue, boost::geometry::index::quadratic<16>>;

    // Represents an AB line with two endpoints
    struct ABLine {
        farmtrax::BPoint A;
        farmtrax::BPoint B;
        std::size_t line_id;
        
        ABLine(const farmtrax::BPoint& a, const farmtrax::BPoint& b, std::size_t id) 
            : A(a), B(b), line_id(id) {}
            
        // Constructor from concord::Point
        ABLine(const concord::Point& a, const concord::Point& b, std::size_t id)
            : A(utils::to_boost(a)), B(utils::to_boost(b)), line_id(id) {}
            
        double length() const {
            return boost::geometry::distance(A, B);
        }
    };

    class Nety {
    public:
        // Constructor from AB lines (farmtrax::BPoint)
        explicit Nety(const std::vector<ABLine>& lines) {
            ab_lines_ = lines;
            build_graph();
        }
        
        // Constructor from vector of point pairs (farmtrax::BPoint)
        explicit Nety(const std::vector<std::pair<farmtrax::BPoint, farmtrax::BPoint>>& point_pairs) {
            ab_lines_.reserve(point_pairs.size());
            for (size_t i = 0; i < point_pairs.size(); ++i) {
                ab_lines_.emplace_back(point_pairs[i].first, point_pairs[i].second, i);
            }
            build_graph();
        }
        
        // Constructor from vector of point pairs (concord::Point)
        explicit Nety(const std::vector<std::pair<concord::Point, concord::Point>>& point_pairs) {
            ab_lines_.reserve(point_pairs.size());
            for (size_t i = 0; i < point_pairs.size(); ++i) {
                ab_lines_.emplace_back(point_pairs[i].first, point_pairs[i].second, i);
            }
            build_graph();
        }
        
        // Constructor from Field
        Nety(const Field& field, std::size_t part_idx) {
            const auto& p = field.get_parts()[part_idx];
            ab_lines_.reserve(p.swaths.size());
            for (size_t i = 0; i < p.swaths.size(); ++i) {
                const auto& swath = p.swaths[i];
                ab_lines_.emplace_back(swath.b_line.front(), swath.b_line.back(), i);
            }
            build_graph();
        }
        
        // Main traversal function - creates agricultural field pattern
        std::vector<Vertex> field_traversal(const farmtrax::BPoint& start_point) const {
            if (ab_lines_.empty()) return {};
            
            std::vector<Vertex> path;
            std::vector<bool> visited_lines(ab_lines_.size(), false);
            
            // Find starting line and endpoint
            auto [current_line_id, current_is_A] = find_closest_endpoint(start_point);
            if (current_line_id == SIZE_MAX) return {};
            
            while (true) {
                // Mark current line as visited
                visited_lines[current_line_id] = true;
                
                // Add both vertices of current line to path (A->B or B->A)
                Vertex start_vertex = current_is_A ? vertex_A_[current_line_id] : vertex_B_[current_line_id];
                Vertex end_vertex = current_is_A ? vertex_B_[current_line_id] : vertex_A_[current_line_id];
                
                path.push_back(start_vertex);
                path.push_back(end_vertex);
                
                // Find next unvisited line
                farmtrax::BPoint current_position = current_is_A ? ab_lines_[current_line_id].B : ab_lines_[current_line_id].A;
                auto [next_line_id, next_is_A] = find_closest_unvisited_endpoint(current_position, visited_lines);
                
                if (next_line_id == SIZE_MAX) break; // No more unvisited lines
                
                current_line_id = next_line_id;
                current_is_A = next_is_A;
            }
            
            return path;
        }
        
        // Overload for concord::Point
        std::vector<Vertex> field_traversal(const concord::Point& start_point) const {
            return field_traversal(utils::to_boost(start_point));
        }
        
        // Calculate shortest path between two points using Dijkstra
        std::vector<Vertex> shortest_path(const farmtrax::BPoint& start, const farmtrax::BPoint& goal) const {
            auto [start_line, start_is_A] = find_closest_endpoint(start);
            auto [goal_line, goal_is_A] = find_closest_endpoint(goal);
            
            if (start_line == SIZE_MAX || goal_line == SIZE_MAX) return {};
            
            Vertex start_vertex = start_is_A ? vertex_A_[start_line] : vertex_B_[start_line];
            Vertex goal_vertex = goal_is_A ? vertex_A_[goal_line] : vertex_B_[goal_line];
            
            std::vector<double> distances(boost::num_vertices(graph_));
            std::vector<Vertex> predecessors(boost::num_vertices(graph_));
            
            boost::dijkstra_shortest_paths(graph_, start_vertex,
                boost::predecessor_map(&predecessors[0]).distance_map(&distances[0]));
            
            // Reconstruct path
            std::vector<Vertex> path;
            for (Vertex v = goal_vertex; v != start_vertex; v = predecessors[v]) {
                path.push_back(v);
                if (v == predecessors[v]) break; // Disconnected
            }
            path.push_back(start_vertex);
            std::reverse(path.begin(), path.end());
            
            return path;
        }
        
        // Overload for concord::Point
        std::vector<Vertex> shortest_path(const concord::Point& start, const concord::Point& goal) const {
            return shortest_path(utils::to_boost(start), utils::to_boost(goal));
        }
        
        // Get total path distance
        double calculate_path_distance(const std::vector<Vertex>& path) const {
            double total_distance = 0.0;
            for (size_t i = 1; i < path.size(); ++i) {
                auto p1 = boost::get(boost::vertex_name, graph_)[path[i-1]];
                auto p2 = boost::get(boost::vertex_name, graph_)[path[i]];
                total_distance += boost::geometry::distance(p1, p2);
            }
            return total_distance;
        }
        
        // Access methods for visualization
        const Graph& get_graph() const { return graph_; }
        const std::vector<ABLine>& get_ab_lines() const { return ab_lines_; }
        
        // Get line sequence from vertex path (for visualization)
        std::vector<std::pair<size_t, bool>> get_line_sequence(const std::vector<Vertex>& path) const {
            std::vector<std::pair<size_t, bool>> sequence;
            
            for (const auto& vertex : path) {
                auto point = boost::get(boost::vertex_name, graph_)[vertex];
                
                // Find which line and endpoint this vertex represents
                for (size_t i = 0; i < ab_lines_.size(); ++i) {
                    const double eps = 1e-6;
                    if (boost::geometry::distance(point, ab_lines_[i].A) < eps) {
                        sequence.emplace_back(i, true); // A endpoint
                        break;
                    } else if (boost::geometry::distance(point, ab_lines_[i].B) < eps) {
                        sequence.emplace_back(i, false); // B endpoint
                        break;
                    }
                }
            }
            
            return sequence;
        }
        
    private:
        std::vector<ABLine> ab_lines_;
        Graph graph_;
        EndpointTree endpoint_tree_;
        std::vector<Vertex> vertex_A_; // A endpoints for each line
        std::vector<Vertex> vertex_B_; // B endpoints for each line
        
        void build_graph() {
            if (ab_lines_.empty()) return;
            
            // Clear existing data
            graph_.clear();
            endpoint_tree_.clear();
            vertex_A_.clear();
            vertex_B_.clear();
            
            vertex_A_.resize(ab_lines_.size());
            vertex_B_.resize(ab_lines_.size());
            
            // Add vertices for all endpoints
            for (size_t i = 0; i < ab_lines_.size(); ++i) {
                const auto& line = ab_lines_[i];
                
                // Add A endpoint
                vertex_A_[i] = boost::add_vertex(VertexProps{line.A}, graph_);
                endpoint_tree_.insert({line.A, i * 2}); // Even indices for A endpoints
                
                // Add B endpoint
                vertex_B_[i] = boost::add_vertex(VertexProps{line.B}, graph_);
                endpoint_tree_.insert({line.B, i * 2 + 1}); // Odd indices for B endpoints
                
                // Add edge between A and B (the work line)
                boost::add_edge(vertex_A_[i], vertex_B_[i], EdgeProps{line.length()}, graph_);
            }
            
            // Add connections between closest endpoints of different lines
            for (size_t i = 0; i < ab_lines_.size(); ++i) {
                // Connect A endpoint to closest endpoints of other lines
                add_nearest_connections(vertex_A_[i], ab_lines_[i].A, i);
                // Connect B endpoint to closest endpoints of other lines  
                add_nearest_connections(vertex_B_[i], ab_lines_[i].B, i);
            }
        }
        
        void add_nearest_connections(Vertex vertex, const farmtrax::BPoint& point, size_t current_line_id) {
            std::vector<PointRTreeValue> nearest;
            endpoint_tree_.query(boost::geometry::index::nearest(point, 5), std::back_inserter(nearest));
            
            for (const auto& near : nearest) {
                size_t endpoint_index = near.second;
                size_t line_id = endpoint_index / 2;
                
                // Don't connect to same line
                if (line_id == current_line_id) continue;
                
                bool is_A = (endpoint_index % 2 == 0);
                Vertex target_vertex = is_A ? vertex_A_[line_id] : vertex_B_[line_id];
                
                double distance = boost::geometry::distance(point, near.first);
                boost::add_edge(vertex, target_vertex, EdgeProps{distance}, graph_);
            }
        }
        
        std::pair<size_t, bool> find_closest_endpoint(const farmtrax::BPoint& point) const {
            std::vector<PointRTreeValue> nearest;
            endpoint_tree_.query(boost::geometry::index::nearest(point, 1), std::back_inserter(nearest));
            
            if (nearest.empty()) return {SIZE_MAX, false};
            
            size_t endpoint_index = nearest[0].second;
            size_t line_id = endpoint_index / 2;
            bool is_A = (endpoint_index % 2 == 0);
            
            return {line_id, is_A};
        }
        
        std::pair<size_t, bool> find_closest_unvisited_endpoint(const farmtrax::BPoint& point, 
                                                               const std::vector<bool>& visited_lines) const {
            std::vector<PointRTreeValue> candidates;
            endpoint_tree_.query(boost::geometry::index::nearest(point, ab_lines_.size() * 2), 
                                std::back_inserter(candidates));
            
            for (const auto& candidate : candidates) {
                size_t endpoint_index = candidate.second;
                size_t line_id = endpoint_index / 2;
                
                if (!visited_lines[line_id]) {
                    bool is_A = (endpoint_index % 2 == 0);
                    return {line_id, is_A};
                }
            }
            
            return {SIZE_MAX, false};
        }
    };

} // namespace farmtrax
