#ifndef PLAN_HPP
#define PLAN_HPP

#include "field.hpp"
#include "mesh.hpp"
#include <algorithm>
#include <boost/graph/graph_traits.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <rclcpp/visibility_control.hpp>
#include <stack>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace farmtrax {
    class Plan {
      private:
        std::vector<std::vector<Swath>> swaths_vec_; // Holds Swath structs

      public:
        Plan() = default;

        void plan_out(std::vector<Swath> swaths, int alternate_freq, bool only_one) {
            // reverse every other swath based on alternate_freq
            for (size_t i = 0; i < swaths.size(); i++) {
                if (((i / alternate_freq) % 2) == 1) {
                    swaths[i].flip();
                }
            }

            if (only_one) {
                std::vector<Swath> temp;
                for (int i = 0; i < alternate_freq; ++i) {
                    std::vector<Swath> group; // Temporary group to collect elements
                    // Iterate through the vector, stepping by n
                    for (size_t j = i; j < swaths.size(); j += alternate_freq) {
                        group.push_back(swaths[j]);
                    }
                    // If the group index is odd, reverse the group
                    if (i % 2 != 0) {
                        std::reverse(group.begin(), group.end());
                    }
                    // Append the group to the temp vector
                    temp.insert(temp.end(), group.begin(), group.end());
                }
                swaths_vec_.push_back(temp);
            } else {
                for (int i = 0; i < alternate_freq; ++i) {
                    // Iterate through the vector, stepping by n
                    std::vector<Swath> temp;
                    for (size_t j = i; j < swaths.size(); j += alternate_freq) {
                        temp.push_back(swaths[j]);
                    }
                    swaths_vec_.push_back(temp);
                }
            }
        }

        std::vector<std::vector<Swath>> get_swaths_vec() { return swaths_vec_; }

      private:
        LineString reverse_line(const LineString &line) {
            LineString reversed_line;
            for (auto it = line.rbegin(); it != line.rend(); it++) {
                reversed_line.push_back(*it);
            }
            return reversed_line;
        }
    };
} // namespace farmtrax

#endif // ROUTE_HPP
