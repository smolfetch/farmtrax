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

    class SwathNetwork {
      public:
        SwathNetwork(const Field &field, std::size_t part_idx, std::size_t k_nearest = 5, double max_dist = 1e6)
            : k_(k_nearest), max_d_(max_dist) {
            const auto &p = field.get_parts()[part_idx];
            swaths_.reserve(p.swaths.size());
            for (auto const &s : p.swaths)
                swaths_.push_back(std::make_shared<Swath>(s));
            build_all();
        }

        SwathNetwork(const std::vector<std::shared_ptr<const Swath>> &swaths, std::size_t k_nearest = 5,
                     double max_dist = 1e6)
            : swaths_(swaths), k_(k_nearest), max_d_(max_dist) {
            build_all();
        }

        std::vector<Vertex> greedy_tour() const {
            double sx = 0, sy = 0;
            std::size_t cnt = 0;
            for (auto const &pr : endpoint_tree_) {
                sx += pr.first.x();
                sy += pr.first.y();
                ++cnt;
            }
            farmtrax::BPoint center{sx / cnt, sy / cnt};
            return greedy_cover(center);
        }

        std::vector<Vertex> greedy_tour(const farmtrax::BPoint &start) const { 
            return greedy_cover(start); 
        }

        // Overload to accept concord::Point
        std::vector<Vertex> greedy_tour(const concord::Point &start) const {
            return greedy_cover(farmtrax::utils::to_boost(start));
        }

        // Improved TSP solver using 2-opt local search
        std::vector<Vertex> optimized_tour(const farmtrax::BPoint &start) const {
            auto tour = greedy_cover(start);
            return two_opt_improve(tour);
        }

        // Overload to accept concord::Point
        std::vector<Vertex> optimized_tour(const concord::Point &start) const {
            auto tour = greedy_cover(farmtrax::utils::to_boost(start));
            return two_opt_improve(tour);
        }

        std::vector<Vertex> shortest_path(const farmtrax::BPoint &start, const farmtrax::BPoint &goal) const {
            std::vector<PointRTreeValue> ns, ng;
            endpoint_tree_.query(boost::geometry::index::nearest(start, 1), std::back_inserter(ns));
            endpoint_tree_.query(boost::geometry::index::nearest(goal, 1), std::back_inserter(ng));
            if (ns.empty() || ng.empty())
                return {};
            Vertex vs = endpoint_to_vertex_.at(ns[0].second);
            Vertex vg = endpoint_to_vertex_.at(ng[0].second);
            std::vector<double> dist(boost::num_vertices(g_));
            std::vector<Vertex> pred(boost::num_vertices(g_));
            boost::dijkstra_shortest_paths(g_, vs, boost::predecessor_map(&pred[0]).distance_map(&dist[0]));
            std::vector<Vertex> path;
            for (Vertex v = vg;; v = pred[v]) {
                path.push_back(v);
                if (v == vs)
                    break;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

      private:
        std::vector<std::shared_ptr<const Swath>> swaths_;
        std::size_t k_;
        double max_d_;
        Graph g_;
        EndpointTree endpoint_tree_;
        std::unordered_map<std::size_t, Vertex> endpoint_to_vertex_;

        void build_all() {
            endpoint_tree_.clear();
            endpoint_to_vertex_.clear();
            for (std::size_t i = 0; i < swaths_.size(); ++i) {
                auto const &s = swaths_[i];
                endpoint_tree_.insert({s->b_line.front(), i * 2});
                endpoint_tree_.insert({s->b_line.back(), i * 2 + 1});
            }
            g_.clear();
            for (auto const &pr : endpoint_tree_) {
                auto idx = pr.second;
                Vertex v = boost::add_vertex(VertexProps{pr.first}, g_);
                endpoint_to_vertex_[idx] = v;
            }
            for (std::size_t i = 0; i < swaths_.size(); ++i) {
                Vertex v0 = endpoint_to_vertex_.at(i * 2);
                Vertex v1 = endpoint_to_vertex_.at(i * 2 + 1);
                double w = boost::geometry::length(swaths_[i]->b_line);
                boost::add_edge(v0, v1, EdgeProps{w}, g_);
            }
            for (auto const &kv : endpoint_to_vertex_) {
                auto idx = kv.first;
                Vertex vs = kv.second;
                BPoint c = boost::get(boost::vertex_name, g_)[vs];
                std::vector<PointRTreeValue> nbrs;
                endpoint_tree_.query(boost::geometry::index::nearest(c, k_ + 1), std::back_inserter(nbrs));
                std::size_t added = 0;
                for (auto const &p : nbrs) {
                    if (p.second == idx)
                        continue;
                    if (++added > k_)
                        break;
                    Vertex vj = endpoint_to_vertex_.at(p.second);
                    double d = boost::geometry::distance(c, p.first);
                    if (d <= max_d_)
                        boost::add_edge(vs, vj, EdgeProps{d}, g_);
                }
            }
        }

        std::vector<Vertex> greedy_cover(farmtrax::BPoint cur) const {
            std::size_t N = swaths_.size();
            std::vector<bool> used(N, false);
            std::vector<Vertex> seq;
            for (std::size_t it = 0; it < N; ++it) {
                double best_d = std::numeric_limits<double>::max();
                std::size_t bi = 0;
                bool rev = false;
                for (std::size_t i = 0; i < N; ++i) {
                    if (used[i])
                        continue;
                    auto &L = swaths_[i]->b_line;
                    auto p0 = L.front(), p1 = L.back();
                    double d0 = boost::geometry::distance(cur, p0);
                    double d1 = boost::geometry::distance(cur, p1);
                    if (d0 < best_d) {
                        best_d = d0;
                        bi = i;
                        rev = false;
                    }
                    if (d1 < best_d) {
                        best_d = d1;
                        bi = i;
                        rev = true;
                    }
                }
                used[bi] = true;
                auto a = bi * 2 + (rev ? 1 : 0);
                auto b = bi * 2 + (rev ? 0 : 1);
                seq.push_back(endpoint_to_vertex_.at(a));
                seq.push_back(endpoint_to_vertex_.at(b));
                cur = rev ? swaths_[bi]->b_line.front() : swaths_[bi]->b_line.back();
            }
            return seq;
        }

        std::vector<Vertex> two_opt_improve(std::vector<Vertex> tour) const {
            bool improved = true;
            std::size_t n = tour.size();
            while (improved) {
                improved = false;
                for (std::size_t i = 1; i < n - 1; ++i) {
                    for (std::size_t j = i + 1; j < n; ++j) {
                        double delta = calculate_distance(tour[i - 1], tour[i]) + calculate_distance(tour[j], tour[(j + 1) % n])
                                       - calculate_distance(tour[i - 1], tour[j]) - calculate_distance(tour[i], tour[(j + 1) % n]);
                        if (delta < 0) {
                            std::reverse(tour.begin() + i, tour.begin() + j + 1);
                            improved = true;
                        }
                    }
                }
            }
            return tour;
        }

        double calculate_distance(Vertex u, Vertex v) const {
            auto pu = boost::get(boost::vertex_name, g_)[u];
            auto pv = boost::get(boost::vertex_name, g_)[v];
            return boost::geometry::distance(pu, pv);
        }
    };

} // namespace farmtrax
