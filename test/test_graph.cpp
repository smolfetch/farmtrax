#include "doctest/doctest.h"
#include "farmtrax/field.hpp"
#include "farmtrax/graph.hpp"
#include <memory>
#include <vector>

// Helper to create some test swaths
std::vector<std::shared_ptr<farmtrax::Swath>> create_test_swaths(const concord::Datum &datum = concord::Datum{}) {
    // Create parallel swaths
    std::vector<std::shared_ptr<farmtrax::Swath>> swaths;

    for (int i = 0; i < 5; i++) {
        auto swath = std::make_shared<farmtrax::Swath>();
        swath->id = i;
        swath->uuid = "swath_" + std::to_string(i);
        swath->type = farmtrax::SwathType::Swath;
        swath->width = 5.0;

        // Create a straight line for the swath
        double x = i * 10.0; // Each swath 10m apart

        // Set up the b_line (boost linestring)
        swath->b_line.push_back(farmtrax::BPoint(x, 0.0));
        swath->b_line.push_back(farmtrax::BPoint(x, 100.0));

        // Set up the centerline (for tests)
        swath->centerline.push_back(farmtrax::BPoint(x, 0.0));
        swath->centerline.push_back(farmtrax::BPoint(x, 100.0));

        // Create concord line
        concord::Point start(concord::ENU{x, 0.0, 0.0}, datum);
        concord::Point end(concord::ENU{x, 100.0, 0.0}, datum);
        swath->line.setStart(start);
        swath->line.setEnd(end);

        // Create concord points for the swath
        swath->points.push_back(start);
        swath->points.push_back(end);

        swaths.push_back(swath);
    }

    return swaths;
}

TEST_CASE("Nety Construction") {
    concord::Datum datum{51.0, 5.0, 0.0};

    // Create test swaths
    auto swaths = create_test_swaths(datum);

    // Convert to const pointers
    std::vector<std::shared_ptr<const farmtrax::Swath>> const_swaths;
    for (const auto &swath : swaths) {
        const_swaths.push_back(std::static_pointer_cast<const farmtrax::Swath>(swath));
    }

    // Create Nety instance
    farmtrax::Nety nety(const_swaths);

    // Check that swaths were properly added
    CHECK(nety.get_swaths().size() == const_swaths.size());
}

TEST_CASE("ABLine Creation and Properties") {
    concord::Datum datum{51.0, 5.0, 0.0};

    // Create ABLine from boost points
    farmtrax::BPoint a(0.0, 0.0);
    farmtrax::BPoint b(10.0, 0.0);
    farmtrax::ABLine line1(a, b, "test_line_1", 1);

    // Create ABLine from concord points
    concord::Point p1(concord::ENU{0.0, 0.0, 0.0}, datum);
    concord::Point p2(concord::ENU{10.0, 0.0, 0.0}, datum);
    farmtrax::ABLine line2(p1, p2, "test_line_2", 2);

    // Check properties
    CHECK(line1.length() == doctest::Approx(10.0));
    CHECK(line2.length() == doctest::Approx(10.0));
    CHECK(line1.uuid == "test_line_1");
    CHECK(line2.uuid == "test_line_2");
}

TEST_CASE("Field Traversal Optimization") {
    concord::Datum datum{51.0, 5.0, 0.0};

    // Create test swaths
    auto swaths = create_test_swaths(datum);

    // Convert to const pointers
    std::vector<std::shared_ptr<const farmtrax::Swath>> const_swaths;
    for (const auto &swath : swaths) {
        const_swaths.push_back(std::static_pointer_cast<const farmtrax::Swath>(swath));
    }

    // Create Nety instance
    farmtrax::Nety nety(const_swaths);

    // Run field traversal to optimize the path
    nety.field_traversal();

    // Get the optimized swaths
    auto optimized_swaths = nety.get_swaths();

    // The optimized swaths will include the original swaths plus connection swaths
    // so its size will be larger than the original count
    CHECK(optimized_swaths.size() >= const_swaths.size());

    // Count how many of the optimized swaths are actual working swaths vs connection swaths
    size_t working_swaths_count = 0;
    for (const auto &swath : optimized_swaths) {
        if (swath->type == farmtrax::SwathType::Swath) {
            working_swaths_count++;
        }
    }
    // Check that the number of working swaths equals the original count
    CHECK(working_swaths_count == const_swaths.size());

    // Check that each original swath is still present (but potentially reordered)
    for (const auto &original : const_swaths) {
        bool found = false;
        for (const auto &optimized : optimized_swaths) {
            if (original->uuid == optimized->uuid) {
                found = true;
                break;
            }
        }
        CHECK(found);
    }
}
