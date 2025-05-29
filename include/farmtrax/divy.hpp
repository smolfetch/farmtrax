#pragma once

#include "farmtrax/field.hpp"
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace farmtrax {

    /**
     * @brief Strategy for dividing swaths among machines.
     */
    enum class DivisionType { BLOCK, STRIP };

    /**
     * @brief Holds per-machine allocation of headlands and swaths using machine ID as key.
     */
    struct DivisionResult {
        std::unordered_map<std::size_t, std::vector<Ring>> headlands_per_machine;
        std::unordered_map<std::size_t, std::vector<Swath>> swaths_per_machine;
    };

    /**
     * @brief Divides a Field’s unfinished headlands and swaths among machines.
     *
     * Headlands are always assigned in BLOCK fashion. Swaths follow the
     * DivisionType specified at construction. The result is cached and
     * can be recomputed if the Field or machine count changes.
     */
    class Divy {
      public:
        /**
         * @brief Construct with a shared pointer to a Field, a division strategy, and initial machine count.
         * @param field Shared pointer to the Field to be divided.
         * @param type  Strategy for dividing swaths (BLOCK or STRIP).
         * @param machines Number of machines (must be > 0).
         * @throws std::invalid_argument if field is null or machines is zero.
         */
        Divy(std::shared_ptr<Field> field, DivisionType type, std::size_t machines)
            : field_(std::move(field)), division_type_(type), machine_count_(machines) {
            if (!field_)
                throw std::invalid_argument("field pointer must not be null");
            if (machine_count_ == 0)
                throw std::invalid_argument("machine_count must be > 0");
            recompute();
        }

        /**
         * @brief Change the number of machines and recompute the division.
         * @param machines New machine count (must be > 0).
         * @throws std::invalid_argument if machines is zero.
         */
        void set_machine_count(std::size_t machines) {
            if (machines == 0)
                throw std::invalid_argument("machine_count must be > 0");
            machine_count_ = machines;
            recompute();
        }

        /**
         * @brief Change the swath division strategy and recompute.
         * @param type New DivisionType.
         */
        void set_division_type(DivisionType type) {
            division_type_ = type;
            recompute();
        }

        /**
         * @brief Recompute the division based on current field, type, and machine count.
         */
        void recompute() {
            division_.headlands_per_machine.clear();
            division_.swaths_per_machine.clear();

            std::vector<Ring> all_headlands;
            for (auto const &part : field_->get_parts())
                for (auto const &h : part.headlands)
                    if (!h.finished)
                        all_headlands.push_back(h);

            {
                const std::size_t H = all_headlands.size();
                const std::size_t base = H / machine_count_;
                const std::size_t rem = H % machine_count_;
                std::size_t idx = 0;
                for (std::size_t m = 0; m < machine_count_; ++m) {
                    std::size_t count = base + (m < rem ? 1 : 0);
                    for (std::size_t i = 0; i < count; ++i)
                        division_.headlands_per_machine[m].push_back(all_headlands[idx++]);
                }
            }

            std::vector<Swath> all_swaths;
            for (auto const &part : field_->get_parts())
                for (auto const &s : part.swaths)
                    if (!s.finished)
                        all_swaths.push_back(s);

            if (division_type_ == DivisionType::BLOCK) {
                const std::size_t S = all_swaths.size();
                const std::size_t base = S / machine_count_;
                const std::size_t rem = S % machine_count_;
                std::size_t idx = 0;
                for (std::size_t m = 0; m < machine_count_; ++m) {
                    std::size_t count = base + (m < rem ? 1 : 0);
                    for (std::size_t i = 0; i < count; ++i)
                        division_.swaths_per_machine[m].push_back(all_swaths[idx++]);
                }
            } else {
                for (std::size_t i = 0; i < all_swaths.size(); ++i)
                    division_.swaths_per_machine[i % machine_count_].push_back(all_swaths[i]);
            }
        }

        /**
         * @brief Retrieve the most recently computed DivisionResult.
         */
        const DivisionResult &result() const { return division_; }

      private:
        std::shared_ptr<Field> field_;
        DivisionType division_type_;
        std::size_t machine_count_;
        DivisionResult division_;
    };

} // namespace farmtrax
