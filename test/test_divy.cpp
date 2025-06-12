#include "doctest/doctest.h"
#include "farmtrax/divy.hpp"
#include "farmtrax/field.hpp"
#include <memory>

// Helper to create a test field with a single part
std::shared_ptr<farmtrax::Part> create_test_part(const concord::Datum &datum = concord::Datum{}) {
    // Create a simple rectangular polygon
    concord::Polygon poly;
    poly.addPoint(concord::Point{concord::ENU{0.0, 0.0, 0.0}, datum});
    poly.addPoint(concord::Point{concord::ENU{100.0, 0.0, 0.0}, datum});
    poly.addPoint(concord::Point{concord::ENU{100.0, 50.0, 0.0}, datum});
    poly.addPoint(concord::Point{concord::ENU{0.0, 50.0, 0.0}, datum});
    poly.addPoint(concord::Point{concord::ENU{0.0, 0.0, 0.0}, datum}); // Close the polygon

    // Create a field with the polygon
    farmtrax::Field field(poly, 0.5, datum);

    // Generate field with swaths
    field.gen_field(5.0, 0.0, 1); // 5m track width, 0 angle, 1 headland

    // Return a copy of the first part
    if (!field.get_parts().empty()) {
        auto part = std::make_shared<farmtrax::Part>(field.get_parts()[0]);
        // Initialize centerline for each swath
        for (auto &swath : part->swaths) {
            // Use the b_line points to initialize the centerline if empty
            if (swath.b_line.size() > 0 && swath.centerline.empty()) {
                for (const auto &bpoint : swath.b_line) {
                    swath.centerline.push_back(bpoint);
                }
            }
        }
        return part;
    }

    // Should never happen with valid input
    throw std::runtime_error("Failed to create test part");
}

TEST_CASE("Divy Constructor and Basic Parameters") {
    concord::Datum datum{51.0, 5.0, 0.0};
    auto part = create_test_part(datum);

    // Test constructor
    farmtrax::Divy divy(part, farmtrax::DivisionType::ALTERNATE, 2);

    // Test parameter modification
    divy.set_machine_count(4);
    divy.set_division_type(farmtrax::DivisionType::BLOCK);

    // Check invalid parameters
    CHECK_THROWS_AS(divy.set_machine_count(0), std::invalid_argument);

    // Initially, division result should be empty
    const auto &result = divy.result();
    CHECK(result.swaths_per_machine.empty());
}

TEST_CASE("Division Computation with Alternating Assignment") {
    concord::Datum datum{51.0, 5.0, 0.0};
    auto part = create_test_part(datum);

    // Create divider with alternating assignment
    farmtrax::Divy divy(part, farmtrax::DivisionType::ALTERNATE, 2);

    // Compute the division
    divy.compute_division();

    // Check the result
    const auto &result = divy.result();

    // Should have entries for both machines
    CHECK(result.swaths_per_machine.count(0) > 0);
    CHECK(result.swaths_per_machine.count(1) > 0);

    // Total swaths should equal the original number of swaths
    size_t total_assigned_swaths = 0;
    for (const auto &[machine_id, swaths] : result.swaths_per_machine) {
        total_assigned_swaths += swaths.size();
    }

    size_t original_swath_count = 0;
    for (const auto &swath : part->swaths) {
        if (swath.type == farmtrax::SwathType::Swath) {
            original_swath_count++;
        }
    }

    CHECK(total_assigned_swaths == original_swath_count);
}

TEST_CASE("Division Computation with Block Assignment") {
    concord::Datum datum{51.0, 5.0, 0.0};
    auto part = create_test_part(datum);

    // Create divider with block assignment
    farmtrax::Divy divy(part, farmtrax::DivisionType::BLOCK, 2);

    // Compute the division
    divy.compute_division();

    // Check the result
    const auto &result = divy.result();

    // Should have entries for both machines
    CHECK(result.swaths_per_machine.count(0) > 0);
    CHECK(result.swaths_per_machine.count(1) > 0);

    // Check spatial continuity - swaths for a machine should be in contiguous blocks
    // This is hard to test precisely, but we can check for rough clustering
    if (!result.swaths_per_machine.at(0).empty() && !result.swaths_per_machine.at(1).empty()) {
        // Get average X position for each machine's swaths
        double avg_x_m0 = 0.0;
        double avg_x_m1 = 0.0;

        for (const auto &swath : result.swaths_per_machine.at(0)) {
            // Use the line start point's x coordinate
            avg_x_m0 += swath->line.getStart().enu.x;
        }
        avg_x_m0 /= result.swaths_per_machine.at(0).size();

        for (const auto &swath : result.swaths_per_machine.at(1)) {
            // Use the line start point's x coordinate
            avg_x_m1 += swath->line.getStart().enu.x;
        }
        avg_x_m1 /= result.swaths_per_machine.at(1).size();

        // In block division, the averages should differ significantly
        // (indicating spatial grouping)
        CHECK(std::abs(avg_x_m0 - avg_x_m1) > 5.0);
    }
}

TEST_CASE("Changing Machine Count") {
    concord::Datum datum{51.0, 5.0, 0.0};
    auto part = create_test_part(datum);

    // Create divider with 2 machines
    farmtrax::Divy divy(part, farmtrax::DivisionType::ALTERNATE, 2);
    divy.compute_division();

    // Check initial division
    const auto &result1 = divy.result();
    CHECK(result1.swaths_per_machine.size() == 2);

    // Change to 3 machines
    divy.set_machine_count(3);
    divy.compute_division();

    // Check new division
    const auto &result2 = divy.result();
    CHECK(result2.swaths_per_machine.size() == 3);

    // Total assigned swaths should remain the same
    size_t total_swaths1 = 0;
    for (const auto &[machine_id, swaths] : result1.swaths_per_machine) {
        total_swaths1 += swaths.size();
    }

    size_t total_swaths2 = 0;
    for (const auto &[machine_id, swaths] : result2.swaths_per_machine) {
        total_swaths2 += swaths.size();
    }

    CHECK(total_swaths1 == total_swaths2);
}
