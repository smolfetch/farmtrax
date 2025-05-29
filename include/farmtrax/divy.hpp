#pragma once

#include "farmtrax/field.hpp"
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace farmtrax {

    enum class DivisionType { BLOCK, STRIP };

    struct DivisionResult {
        std::unordered_map<std::size_t, std::vector<Ring>> headlands_per_machine;
        std::unordered_map<std::size_t, std::vector<Swath>> swaths_per_machine;
    };

    class Divy {
      public:
        explicit Divy(std::shared_ptr<Field> field, DivisionType type);
        DivisionResult divide(std::size_t machine_count) const;

      private:
        std::shared_ptr<Field> field_;
        DivisionType division_type_;
    };

    inline Divy::Divy(std::shared_ptr<Field> field, DivisionType type)
        : field_(std::move(field)), division_type_(type) {
        if (!field_)
            throw std::invalid_argument("field pointer must not be null");
    }

    inline DivisionResult Divy::divide(std::size_t machine_count) const {
        if (machine_count == 0)
            throw std::invalid_argument("machine_count must be > 0");

        DivisionResult result;

        std::vector<Ring> all_headlands;
        for (auto const &part : field_->get_parts())
            for (auto const &h : part.headlands)
                if (!h.finished)
                    all_headlands.push_back(h);

        {
            const std::size_t H = all_headlands.size();
            const std::size_t base = H / machine_count;
            const std::size_t rem = H % machine_count;
            std::size_t idx = 0;
            for (std::size_t m = 0; m < machine_count; ++m) {
                std::size_t count = base + (m < rem ? 1 : 0);
                for (std::size_t i = 0; i < count; ++i)
                    result.headlands_per_machine[m].push_back(all_headlands[idx++]);
            }
        }

        std::vector<Swath> all_swaths;
        for (auto const &part : field_->get_parts())
            for (auto const &s : part.swaths)
                if (!s.finished)
                    all_swaths.push_back(s);

        if (division_type_ == DivisionType::BLOCK) {
            const std::size_t S = all_swaths.size();
            const std::size_t base = S / machine_count;
            const std::size_t rem = S % machine_count;
            std::size_t idx = 0;
            for (std::size_t m = 0; m < machine_count; ++m) {
                std::size_t count = base + (m < rem ? 1 : 0);
                for (std::size_t i = 0; i < count; ++i)
                    result.swaths_per_machine[m].push_back(all_swaths[idx++]);
            }
        } else { // STRIP
            for (std::size_t i = 0; i < all_swaths.size(); ++i) {
                result.swaths_per_machine[i % machine_count].push_back(all_swaths[i]);
            }
        }

        return result;
    }

} // namespace farmtrax
