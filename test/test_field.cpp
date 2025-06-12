#include "doctest/doctest.h"
#include "farmtrax/field.hpp"
#include <boost/geometry/io/wkt/wkt.hpp>

// Extension methods for Field class to help with testing
// These need to be at the global namespace level as they are declared as friends in Field class

namespace farmtrax {
    // Friend functions declared in Field class
    inline double get_resolution(const Field &field) { return field.resolution_; }

    inline concord::Datum get_field_datum(const Field &field) { return field.datum_; }

    inline double get_total_field_area(const Field &field) {
        double total_area = 0.0;
        const auto &border = field.border_;
        auto boost_poly = farmtrax::utils::to_boost(border);
        return boost::geometry::area(boost_poly);
    }
} // namespace farmtrax

// Helper to create a simple test polygon
concord::Polygon create_test_polygon(const concord::Datum &datum = concord::Datum{}) {
    // Create a simple rectangular polygon
    concord::Polygon poly;
    poly.addPoint(concord::Point{concord::ENU{0.0, 0.0, 0.0}, datum});
    poly.addPoint(concord::Point{concord::ENU{100.0, 0.0, 0.0}, datum});
    poly.addPoint(concord::Point{concord::ENU{100.0, 50.0, 0.0}, datum});
    poly.addPoint(concord::Point{concord::ENU{0.0, 50.0, 0.0}, datum});
    poly.addPoint(concord::Point{concord::ENU{0.0, 0.0, 0.0}, datum}); // Close the polygon
    return poly;
}

TEST_CASE("Field Constructor and Basic Properties") {
    concord::Datum datum{51.0, 5.0, 0.0};
    concord::Polygon poly = create_test_polygon(datum);

    // Test basic field creation with default parameters
    farmtrax::Field field(poly, 0.5, datum);

    // Verify the field was created
    CHECK(field.get_parts().size() > 0);

    // Check basic field properties
    CHECK(farmtrax::get_resolution(field) == 0.5);
    CHECK(farmtrax::get_field_datum(field).lat == datum.lat);
    CHECK(farmtrax::get_field_datum(field).lon == datum.lon);

    // Test area calculation
    double expected_area = 100.0 * 50.0; // 100 x 50 rectangle
    double actual_area = farmtrax::get_total_field_area(field);
    CHECK(actual_area == doctest::Approx(expected_area).epsilon(0.1));
}

TEST_CASE("Field Partitioning") {
    concord::Datum datum{51.0, 5.0, 0.0};
    concord::Polygon poly = create_test_polygon(datum);

    // Create a field with partitioning parameters
    double area_threshold = 1000.0; // Small enough to force partitioning of our 5000 sq.m field
    farmtrax::Field field(poly, 0.5, datum, true, 0.7, area_threshold);

    // Verify that field was partitioned
    CHECK(field.get_parts().size() > 1);

    // Each part should have area less than or equal to our threshold
    for (const auto &part : field.get_parts()) {
        double part_area = boost::geometry::area(part.border.b_polygon);
        CHECK(part_area <= area_threshold);
    }
}

TEST_CASE("Field Generation Methods") {
    concord::Datum datum{51.0, 5.0, 0.0};
    concord::Polygon poly = create_test_polygon(datum);

    // Create a field
    farmtrax::Field field(poly, 0.5, datum);

    // Test field generation with different parameters
    SUBCASE("Standard field generation") {
        field.gen_field(5.0, 0.0, 2); // 5m track width, 0 angle, 2 headlands

        // Check results
        for (const auto &part : field.get_parts()) {
            // Check that headlands were created
            CHECK(part.headlands.size() == 2);

            // Check that swaths were created
            CHECK(!part.swaths.empty());
        }
    }

    SUBCASE("Field generation with different angle") {
        field.gen_field(5.0, 45.0, 1); // 5m track width, 45 degree angle, 1 headland

        // Check that the field was generated with the specified parameters
        for (const auto &part : field.get_parts()) {
            CHECK(part.headlands.size() == 1);
            CHECK(!part.swaths.empty());
        }
    }
}

TEST_CASE("Field Noise Addition") {
    concord::Datum datum{51.0, 5.0, 0.0};
    concord::Polygon poly = create_test_polygon(datum);

    // Create a field
    farmtrax::Field field(poly, 0.5, datum);

    // Generate the field
    field.gen_field(5.0, 0.0, 1);

    // Get swath points before noise
    std::vector<farmtrax::BPoint> original_points;
    if (!field.get_parts().empty() && !field.get_parts()[0].swaths.empty()) {
        const auto &first_swath = field.get_parts()[0].swaths[0];
        original_points = first_swath.centerline;
    }

    // Add noise
    field.add_noise(0.1); // 10% noise

    // Get swath points after noise
    std::vector<farmtrax::BPoint> noisy_points;
    if (!field.get_parts().empty() && !field.get_parts()[0].swaths.empty()) {
        const auto &first_swath = field.get_parts()[0].swaths[0];
        noisy_points = first_swath.centerline;
    }

    // Check that noise was added (points should be different)
    if (!original_points.empty() && !noisy_points.empty()) {
        bool points_changed = false;
        for (size_t i = 0; i < std::min(original_points.size(), noisy_points.size()); ++i) {
            if (boost::geometry::distance(original_points[i], noisy_points[i]) > 1e-6) {
                points_changed = true;
                break;
            }
        }
        CHECK(points_changed);
    }
}
