#pragma once
#include "farmtrax/field.hpp"
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace farmtrax {

    enum class DivisionType { BLOCK, ALTERNATE, SPATIAL_RTREE, LENGTH_BALANCED };

    struct DivisionResult {
        std::unordered_map<std::size_t, std::vector<std::shared_ptr<const Ring>>> headlands_per_machine;
        std::unordered_map<std::size_t, std::vector<std::shared_ptr<const Swath>>> swaths_per_machine;
    };

    class Divy {
      public:
        Divy(std::shared_ptr<Field> field, DivisionType type, std::size_t machines);
        void set_machine_count(std::size_t machines);
        void set_division_type(DivisionType type);
        const DivisionResult &result() const;

      private:
        void recompute();

        std::shared_ptr<Field> field_;
        DivisionType division_type_;
        std::size_t machine_count_;
        DivisionResult division_;
    };

    inline Divy::Divy(std::shared_ptr<Field> field, DivisionType type, std::size_t machines)
        : field_(std::move(field)), division_type_(type), machine_count_(machines) {
        if (!field_)
            throw std::invalid_argument("null field");
        if (machine_count_ == 0)
            throw std::invalid_argument("machine count > 0");
        recompute();
    }

    inline void Divy::set_machine_count(std::size_t machines) {
        if (machines == 0)
            throw std::invalid_argument("machine count > 0");
        machine_count_ = machines;
        recompute();
    }

    inline void Divy::set_division_type(DivisionType type) {
        division_type_ = type;
        recompute();
    }

    inline const DivisionResult &Divy::result() const { return division_; }

    inline void Divy::recompute() {
        division_.headlands_per_machine.clear();
        division_.swaths_per_machine.clear();

        std::vector<std::shared_ptr<const Ring>> all_headlands;
        for (auto &part : field_->get_parts())
            for (auto &h : part.headlands)
                if (!h.finished)
                    all_headlands.emplace_back(field_, &h);

        std::size_t H = all_headlands.size();
        std::size_t base_h = H / machine_count_;
        std::size_t rem_h = H % machine_count_;
        std::size_t idx_h = 0;
        for (std::size_t m = 0; m < machine_count_; ++m) {
            std::size_t count = base_h + (m < rem_h ? 1 : 0);
            for (std::size_t i = 0; i < count; ++i)
                division_.headlands_per_machine[m].push_back(all_headlands[idx_h++]);
        }

        std::vector<std::shared_ptr<const Swath>> all_swaths;
        for (auto &part : field_->get_parts())
            for (auto &s : part.swaths)
                if (!s.finished)
                    all_swaths.emplace_back(field_, &s);

        if (division_type_ == DivisionType::SPATIAL_RTREE) {
            SwathRTree rtree;
            for (std::size_t i = 0; i < all_swaths.size(); ++i)
                rtree.insert(std::make_pair(all_swaths[i]->bounding_box, i));
            std::size_t S = all_swaths.size();
            std::size_t base_s = S / machine_count_;
            std::size_t rem_s = S % machine_count_;
            for (std::size_t m = 0; m < machine_count_; ++m) {
                std::size_t count = base_s + (m < rem_s ? 1 : 0);
                std::size_t last_idx = 0;
                for (std::size_t i = 0; i < count; ++i) {
                    std::size_t idx;
                    if (i == 0) {
                        auto it = rtree.begin();
                        idx = it->second;
                    } else {
                        BBox bb = all_swaths[last_idx]->bounding_box;
                        BPoint center((bb.min_corner().x() + bb.max_corner().x()) / 2.0,
                                      (bb.min_corner().y() + bb.max_corner().y()) / 2.0);
                        std::vector<SwathRTreeValue> result;
                        rtree.query(boost::geometry::index::nearest(center, 1), std::back_inserter(result));
                        idx = result[0].second;
                    }
                    division_.swaths_per_machine[m].push_back(all_swaths[idx]);
                    rtree.remove(std::make_pair(all_swaths[idx]->bounding_box, idx));
                    last_idx = idx;
                }
            }
        } else if (division_type_ == DivisionType::LENGTH_BALANCED) {
            std::vector<double> loads(machine_count_, 0.0);
            std::vector<std::vector<std::shared_ptr<const Swath>>> assign(machine_count_);
            std::vector<std::pair<double, std::shared_ptr<const Swath>>> by_length;
            for (auto &sw : all_swaths)
                by_length.emplace_back(sw->line.length(), sw);
            std::sort(by_length.begin(), by_length.end(), [](auto &a, auto &b) { return a.first > b.first; });
            for (auto &p : by_length) {
                auto it = std::min_element(loads.begin(), loads.end());
                std::size_t m = std::distance(loads.begin(), it);
                assign[m].push_back(p.second);
                loads[m] += p.first;
            }
            for (std::size_t m = 0; m < machine_count_; ++m)
                division_.swaths_per_machine[m] = std::move(assign[m]);
        } else if (division_type_ == DivisionType::BLOCK) {
            std::size_t S = all_swaths.size();
            std::size_t base_s = S / machine_count_;
            std::size_t rem_s = S % machine_count_;
            std::size_t idx_s = 0;
            for (std::size_t m = 0; m < machine_count_; ++m) {
                std::size_t count = base_s + (m < rem_s ? 1 : 0);
                for (std::size_t i = 0; i < count; ++i)
                    division_.swaths_per_machine[m].push_back(all_swaths[idx_s++]);
            }
        } else {
            for (std::size_t i = 0; i < all_swaths.size(); ++i)
                division_.swaths_per_machine[i % machine_count_].push_back(all_swaths[i]);
        }
    }

} // namespace farmtrax
