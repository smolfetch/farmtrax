#include "doctest/doctest.h"
#include "farmtrax/avoid.hpp"
#include "farmtrax/field.hpp"

// Helper function to create a simple obstacle polygon
concord::Polygon create_obstacle(const concord::Datum &datum = concord::Datum{}) {
    concord::Polygon obstacle;
    // Create a simple square obstacle at (50,25) with 10m sides
    obstacle.addPoint(concord::Point{concord::ENU{45.0, 20.0, 0.0}, datum});
    obstacle.addPoint(concord::Point{concord::ENU{55.0, 20.0, 0.0}, datum});
    obstacle.addPoint(concord::Point{concord::ENU{55.0, 30.0, 0.0}, datum});
    obstacle.addPoint(concord::Point{concord::ENU{45.0, 30.0, 0.0}, datum});
    obstacle.addPoint(concord::Point{concord::ENU{45.0, 20.0, 0.0}, datum}); // Close the polygon
    return obstacle;
}

// Helper to create a field with swaths going through an obstacle
std::pair<farmtrax::Field, concord::Polygon>
create_field_with_obstacle(const concord::Datum &datum = concord::Datum{}) {
    // Create a rectangular field
    concord::Polygon field_poly;
    field_poly.addPoint(concord::Point{concord::ENU{0.0, 0.0, 0.0}, datum});
    field_poly.addPoint(concord::Point{concord::ENU{100.0, 0.0, 0.0}, datum});
    field_poly.addPoint(concord::Point{concord::ENU{100.0, 50.0, 0.0}, datum});
    field_poly.addPoint(concord::Point{concord::ENU{0.0, 50.0, 0.0}, datum});
    field_poly.addPoint(concord::Point{concord::ENU{0.0, 0.0, 0.0}, datum}); // Close the polygon

    // Create the field with N-S swaths (will go through the obstacle)
    farmtrax::Field field(field_poly, 0.5, datum);
    field.gen_field(5.0, 0.0, 1); // 5m swath width, 0 angle, 1 headland

    // Create obstacle
    concord::Polygon obstacle = create_obstacle(datum);

    return {field, obstacle};
}

TEST_CASE("ObstacleAvoider Construction") {
    concord::Datum datum{51.0, 5.0, 0.0};

    // Create an obstacle
    concord::Polygon obstacle = create_obstacle(datum);
    std::vector<concord::Polygon> obstacles = {obstacle};

    // Create avoider
    farmtrax::ObstacleAvoider avoider(obstacles, datum);

    // No real way to test internal state, but at least we verify construction doesn't throw
}

TEST_CASE("Obstacle Avoidance with Concord Polygons") {
    concord::Datum datum{51.0, 5.0, 0.0};

    // Create field with obstacle
    auto [field, obstacle] = create_field_with_obstacle(datum);
    std::vector<concord::Polygon> obstacles = {obstacle};

    // Create avoider
    farmtrax::ObstacleAvoider avoider(obstacles, datum);

    // Extract swaths from the field
    std::vector<std::shared_ptr<const farmtrax::Swath>> input_swaths;
    for (const auto &part : field.get_parts()) {
        for (const auto &swath : part.swaths) {
            if (swath.type == farmtrax::SwathType::Swath) {
                auto swath_ptr = std::make_shared<farmtrax::Swath>(swath);
                input_swaths.push_back(swath_ptr);
            }
        }
    }

    // Apply avoidance with 2.0m inflation
    auto avoided_swaths = avoider.avoid(input_swaths, 2.0);

    // Verify avoidance results
    CHECK(!avoided_swaths.empty());

    // Should have more output swaths than input if splits occurred
    CHECK(avoided_swaths.size() >= input_swaths.size());

    // Each avoided swath should be a different pointer than input swaths
    bool found_new_swath = false;
    for (const auto &out_swath : avoided_swaths) {
        bool is_original = false;
        for (const auto &in_swath : input_swaths) {
            if (out_swath.get() == in_swath.get()) {
                is_original = true;
                break;
            }
        }
        if (!is_original) {
            found_new_swath = true;
            break;
        }
    }
    CHECK(found_new_swath);
}

TEST_CASE("Obstacle Avoidance with Different Inflation Distances") {
    concord::Datum datum{51.0, 5.0, 0.0};

    // Create field with obstacle
    auto [field, obstacle] = create_field_with_obstacle(datum);
    std::vector<concord::Polygon> obstacles = {obstacle};

    // Create avoider
    farmtrax::ObstacleAvoider avoider(obstacles, datum);

    // Get swaths from the field
    std::vector<std::shared_ptr<const farmtrax::Swath>> input_swaths;
    for (const auto &part : field.get_parts()) {
        for (const auto &swath : part.swaths) {
            if (swath.type == farmtrax::SwathType::Swath) {
                auto swath_ptr = std::make_shared<farmtrax::Swath>(swath);
                input_swaths.push_back(swath_ptr);
            }
        }
    }

    // Test with different inflation distances
    SUBCASE("No inflation") {
        auto avoided_swaths = avoider.avoid(input_swaths, 0.0);
        // With no inflation, the result should be similar to input
        // (might have some changes due to exact intersections)
        CHECK(avoided_swaths.size() >= input_swaths.size());
    }

    SUBCASE("Small inflation") {
        auto avoided_swaths = avoider.avoid(input_swaths, 1.0);
        // With small inflation, expect moderate changes
        CHECK(avoided_swaths.size() >= input_swaths.size());
    }

    SUBCASE("Large inflation") {
        auto avoided_swaths = avoider.avoid(input_swaths, 5.0);
        // With large inflation, expect more significant changes
        // (more swaths affected, potentially more splits)
        CHECK(avoided_swaths.size() >= input_swaths.size());
    }
}
