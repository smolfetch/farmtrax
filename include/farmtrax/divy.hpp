#pragma once

#include "farmtrax/field.hpp"
#include <memory>
#include <stdexcept>
#include <vector>

namespace farmtrax {

    /**
     * @brief Holds per-machine allocation of headlands and swaths.
     */
    struct DivisionResult {
        std::vector<std::vector<Ring>> headlands_per_machine;
        std::vector<std::vector<Swath>> swaths_per_machine;
    };

    /**
     * @brief Divides a Field’s unfinished headlands and swaths among machines.
     */
    class Divy {
      public:
        explicit Divy(std::shared_ptr<Field> field);

        DivisionResult divide(std::size_t machine_count) const;

      private:
        std::shared_ptr<Field> field_;
    };

    inline Divy::Divy(std::shared_ptr<Field> field) : field_(std::move(field)) {
        if (!field_)
            throw std::invalid_argument("field pointer must not be null");
    }

    inline DivisionResult Divy::divide(std::size_t machine_count) const {
        if (machine_count == 0)
            throw std::invalid_argument("machine_count must be > 0");

        DivisionResult result;
        result.headlands_per_machine.resize(machine_count);
        result.swaths_per_machine.resize(machine_count);

        std::vector<Ring> all_headlands;
        for (auto const &part : field_->get_parts()) {
            for (auto const &h : part.headlands) {
                if (!h.finished)
                    all_headlands.push_back(h);
            }
        }

        const std::size_t H = all_headlands.size();
        const std::size_t base_h = H / machine_count;
        const std::size_t rem_h = H % machine_count;
        std::size_t idx = 0;
        for (std::size_t m = 0; m < machine_count; ++m) {
            const std::size_t count = base_h + (m < rem_h ? 1 : 0);
            for (std::size_t i = 0; i < count; ++i)
                result.headlands_per_machine[m].push_back(all_headlands[idx++]);
        }

        std::vector<Swath> all_swaths;
        for (auto const &part : field_->get_parts()) {
            for (auto const &s : part.swaths) {
                if (!s.finished)
                    all_swaths.push_back(s);
            }
        }

        const std::size_t S = all_swaths.size();
        const std::size_t base_s = S / machine_count;
        const std::size_t rem_s = S % machine_count;
        idx = 0;
        for (std::size_t m = 0; m < machine_count; ++m) {
            const std::size_t count = base_s + (m < rem_s ? 1 : 0);
            for (std::size_t i = 0; i < count; ++i)
                result.swaths_per_machine[m].push_back(all_swaths[idx++]);
        }

        return result;
    }

} // namespace farmtrax
