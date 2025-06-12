#include "doctest/doctest.h"
#include "farmtrax/utils/utils.hpp"

TEST_CASE("Conversion Functions") {
    SUBCASE("float_to_byte") {
        // Test standard range [0.0, 1.0] -> [0, 255]
        CHECK(farmtrax::utils::float_to_byte(0.0f) == 0);
        CHECK(farmtrax::utils::float_to_byte(1.0f) == 255);
        CHECK(farmtrax::utils::float_to_byte(0.5f) == 128);

        // Test clamping
        CHECK(farmtrax::utils::float_to_byte(-1.0f) == 0);
        CHECK(farmtrax::utils::float_to_byte(2.0f) == 255);

        // Test custom range
        CHECK(farmtrax::utils::float_to_byte(0.5f, 100.0f, 200.0f) == 128);
    }
}

TEST_CASE("Geometry Utilities") {
    concord::Datum datum{51.0, 5.0, 0.0};

    SUBCASE("are_colinear") {
        // Test colinear points
        concord::Point p1(concord::ENU{0.0, 0.0, 0.0}, datum);
        concord::Point p2(concord::ENU{1.0, 1.0, 0.0}, datum);
        concord::Point p3(concord::ENU{2.0, 2.0, 0.0}, datum);
        CHECK(farmtrax::utils::are_colinear(p1, p2, p3));

        // Test non-colinear points
        concord::Point p4(concord::ENU{0.0, 0.0, 0.0}, datum);
        concord::Point p5(concord::ENU{1.0, 1.0, 0.0}, datum);
        concord::Point p6(concord::ENU{2.0, 1.0, 0.0}, datum);
        CHECK_FALSE(farmtrax::utils::are_colinear(p4, p5, p6));

        // Test with epsilon
        concord::Point p7(concord::ENU{0.0, 0.0, 0.0}, datum);
        concord::Point p8(concord::ENU{1.0, 1.0, 0.0}, datum);
        concord::Point p9(concord::ENU{2.0, 2.0001, 0.0}, datum); // Very slight deviation
        CHECK(farmtrax::utils::are_colinear(p7, p8, p9, 0.001));  // Should be considered colinear with large epsilon
        CHECK_FALSE(farmtrax::utils::are_colinear(p7, p8, p9, 1e-10)); // Should not be colinear with small epsilon
    }

    SUBCASE("remove_colinear_points") {
        // Create a polygon with some colinear points
        concord::Polygon poly;
        poly.addPoint(concord::Point{concord::ENU{0.0, 0.0, 0.0}, datum});
        poly.addPoint(concord::Point{concord::ENU{1.0, 0.0, 0.0}, datum});
        poly.addPoint(concord::Point{concord::ENU{2.0, 0.0, 0.0}, datum}); // Colinear with the previous two
        poly.addPoint(concord::Point{concord::ENU{2.0, 1.0, 0.0}, datum});
        poly.addPoint(concord::Point{concord::ENU{0.0, 1.0, 0.0}, datum});
        poly.addPoint(concord::Point{concord::ENU{0.0, 0.0, 0.0}, datum}); // Close the polygon

        // Remove colinear points
        concord::Polygon simplified = farmtrax::utils::remove_colinear_points(poly, 0.01);

        // The simplified polygon should have fewer points
        // (original had 6, should be reduced because of the colinear points)
        CHECK(simplified.getPoints().size() < poly.getPoints().size());
    }

    SUBCASE("polygon_to_boost and from_boost") {
        // Create a concord polygon
        concord::Polygon poly;
        poly.addPoint(concord::Point{concord::ENU{0.0, 0.0, 0.0}, datum});
        poly.addPoint(concord::Point{concord::ENU{1.0, 0.0, 0.0}, datum});
        poly.addPoint(concord::Point{concord::ENU{1.0, 1.0, 0.0}, datum});
        poly.addPoint(concord::Point{concord::ENU{0.0, 1.0, 0.0}, datum});
        poly.addPoint(concord::Point{concord::ENU{0.0, 0.0, 0.0}, datum}); // Close the polygon

        // Convert to boost polygon
        farmtrax::BPolygon boost_poly = farmtrax::utils::to_boost(poly);

        // Check dimensions
        double area = boost::geometry::area(boost_poly);
        CHECK(area == doctest::Approx(1.0));

        // Convert back to concord polygon
        concord::Polygon converted_poly = farmtrax::utils::from_boost(boost_poly, datum);

        // Check the points are preserved
        CHECK(converted_poly.getPoints().size() == poly.getPoints().size());

        // Check specific points
        const auto &original_pts = poly.getPoints();
        const auto &converted_pts = converted_poly.getPoints();
        for (size_t i = 0; i < std::min(original_pts.size(), converted_pts.size()); ++i) {
            CHECK(original_pts[i].enu.x == doctest::Approx(converted_pts[i].enu.x));
            CHECK(original_pts[i].enu.y == doctest::Approx(converted_pts[i].enu.y));
        }
    }
}

TEST_CASE("Distance Calculation") {
    concord::Datum datum{51.0, 5.0, 0.0};

    SUBCASE("point_to_line_distance") {
        // Create a vertical line at x = 1.0
        farmtrax::BPoint line_start(1.0, 0.0);
        farmtrax::BPoint line_end(1.0, 10.0);

        // Test point at (0, 5) - should be 1.0 unit away
        farmtrax::BPoint point(0.0, 5.0);
        double distance = farmtrax::utils::point_to_line_distance(point, line_start, line_end);
        CHECK(distance == doctest::Approx(1.0));

        // Test point at (1, 5) - should be 0.0 units away (on the line)
        farmtrax::BPoint point2(1.0, 5.0);
        double distance2 = farmtrax::utils::point_to_line_distance(point2, line_start, line_end);
        CHECK(distance2 == doctest::Approx(0.0));

        // Test point at (1, 15) - should use point-to-point distance to the end point
        farmtrax::BPoint point3(1.0, 15.0);
        double distance3 = farmtrax::utils::point_to_line_distance(point3, line_start, line_end);
        CHECK(distance3 == doctest::Approx(5.0));
    }

    SUBCASE("angle_between") {
        // Test parallel vectors in the same direction
        farmtrax::BPoint v1(1.0, 0.0);
        farmtrax::BPoint v2(2.0, 0.0);
        double angle1 = farmtrax::utils::angle_between(v1, v2);
        CHECK(angle1 == doctest::Approx(0.0));

        // Test perpendicular vectors
        farmtrax::BPoint v3(0.0, 1.0);
        double angle2 = farmtrax::utils::angle_between(v1, v3);
        CHECK(angle2 == doctest::Approx(M_PI / 2.0));

        // Test opposite directions
        farmtrax::BPoint v4(-1.0, 0.0);
        double angle3 = farmtrax::utils::angle_between(v1, v4);
        CHECK(angle3 == doctest::Approx(M_PI));
    }
}
