#include <iostream>
#include <iomanip>
#include <thread>

#include "concord/concord.hpp"
#include "geoson/geoson.hpp"
#include "geotiv/geotiv.hpp"

#include "rerun/recording_stream.hpp"

#include "farmtrax/avoid.hpp"
#include "farmtrax/divy.hpp"
#include "farmtrax/field.hpp"
#include "farmtrax/graph.hpp"
#include "rerun.hpp"

#include "farmtrax/utils/visualize.hpp"

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("farmtrax", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    concord::Datum datum{51.98954034749562, 5.6584737410504715, 53.80182312011719};

    concord::Polygon poly;
    try {
        auto fc = geoson::ReadFeatureCollection("misc/field4.geojson");
        for (auto &f : fc.features) {
            if (std::get_if<concord::Polygon>(&f.geometry)) {
                poly = std::get<concord::Polygon>(f.geometry);
                break;
            }
        }
    } catch (std::exception &e) {
        std::cerr << "Failed to parse geojson: " << e.what() << "\n";
        return 1;
    }

    for (auto &p : poly.getPoints()) {
        std::cout << "x: " << p.enu.x << ", y: " << p.enu.y << ", z: " << p.enu.z << "\n";
    }

    // === ADVANCED PARTITIONING DEMONSTRATION ===
    std::cout << "\n=== Advanced Field Partitioning Demo ===\n";
    
    // 1. Test basic partitioning with default criteria
    std::cout << "1. Testing default partitioning criteria:\n";
    farmtrax::Partitioner basic_partitioner(poly, datum);
    auto basic_parts = basic_partitioner.partition(20000.0); // 2 hectares
    std::cout << "   - Default criteria: " << basic_parts.size() << " parts\n";
    
    // 2. Test advanced multi-criteria partitioning
    std::cout << "\n2. Testing advanced multi-criteria partitioning:\n";
    farmtrax::Partitioner advanced_partitioner(poly, datum);
    
    // Configure strict criteria for agricultural optimization
    farmtrax::Partitioner::PartitionCriteria strict_criteria;
    strict_criteria.max_area = 50000.0;           // 0.8 hectares max for tight control
    strict_criteria.min_convexity = 0.75;        // 75% convexity requirement
    strict_criteria.max_aspect_ratio = 2.5;      // 2.5:1 max ratio for better machinery access
    strict_criteria.min_bridge_width = 12.0;     // 12m minimum bridge width
    strict_criteria.tooth_threshold = 0.2;       // 20% area threshold for teeth
    strict_criteria.enable_bridge_detection = true;
    strict_criteria.enable_tooth_detection = true;
    strict_criteria.enable_aspect_splitting = true;
    strict_criteria.max_recursion_depth = 7;     // Allow deeper recursion for complex fields
    
    auto advanced_parts = advanced_partitioner.partition(8000.0, strict_criteria);
    std::cout << "   - Strict criteria: " << advanced_parts.size() << " parts\n";
    
    // 3. Test different strategy combinations
    std::cout << "\n3. Testing individual partitioning strategies:\n";
    
    // Bridge-only strategy
    farmtrax::Partitioner::PartitionCriteria bridge_only;
    bridge_only.max_area = 50000.0;              // Large area to focus on bridges
    bridge_only.min_bridge_width = 8.0;          // Aggressive bridge detection
    bridge_only.enable_bridge_detection = true;
    bridge_only.enable_tooth_detection = false;
    bridge_only.enable_aspect_splitting = false;
    
    farmtrax::Partitioner bridge_partitioner(poly, datum);
    auto bridge_parts = bridge_partitioner.partition(50000.0, bridge_only);
    std::cout << "   - Bridge-only strategy: " << bridge_parts.size() << " parts\n";
    
    // Shape quality strategy
    farmtrax::Partitioner::PartitionCriteria shape_quality;
    shape_quality.max_area = 50000.0;
    shape_quality.min_convexity = 0.85;          // Very strict convexity
    shape_quality.max_aspect_ratio = 2.0;        // Compact shapes only
    shape_quality.enable_bridge_detection = false;
    shape_quality.enable_tooth_detection = true;
    shape_quality.enable_aspect_splitting = true;
    
    farmtrax::Partitioner shape_partitioner(poly, datum);
    auto shape_parts = shape_partitioner.partition(50000.0, shape_quality);
    std::cout << "   - Shape quality strategy: " << shape_parts.size() << " parts\n";
    
    std::cout << "\n4. Creating field with optimal partitioning:\n";
    // Use the advanced partitioning for the actual field processing
    farmtrax::Field field(poly, 0.1, datum, true, 0.7, 8000.0); // Use strict 0.8 hectare threshold
    std::cout << "   - Field created with " << field.get_parts().size() << " optimally partitioned parts\n";
    
    field.add_noise();

    // geotiv::Layer layer;
    // layer.grid = field.get_grid(0);
    // layer.samplesPerPixel = 1;
    // layer.planarConfig = 1;
    // geotiv::RasterCollection rc;
    // rc.crs = concord::CRS::WGS;
    // rc.datum = concord::Datum();
    // rc.heading = concord::Euler{0.0, 0.0, 0.0};
    // rc.resolution = 0.1;
    // rc.layers.push_back(layer);
    //
    // std::filesystem::path outPath = "output.tif";
    // geotiv::WriteRasterCollection(rc, outPath);

    field.gen_field(4.0, 0.0, 3);
    auto num_machines = 2;

    // Create some example obstacles (e.g., trees, buildings, water bodies)
    std::vector<concord::Polygon> obstacles;

    // Calculate the actual center of the field based on its bounds
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto &point : poly.getPoints()) {
        min_x = std::min(min_x, point.enu.x);
        max_x = std::max(max_x, point.enu.x);
        min_y = std::min(min_y, point.enu.y);
        max_y = std::max(max_y, point.enu.y);
    }

    double center_x = (min_x + max_x) / 2.0;
    double center_y = (min_y + max_y) / 2.0;

    std::cout << "Field bounds: x[" << min_x << ", " << max_x << "], y[" << min_y << ", " << max_y << "]\n";
    std::cout << "Field center: (" << center_x << ", " << center_y << ")\n";

    // Create a square obstacle at the center of the field
    concord::Polygon obstacle1;
    double obstacle_size = 15.0; // 15 meter square obstacle

    obstacle1.addPoint(concord::Point{concord::ENU{center_x - obstacle_size, center_y - obstacle_size, 0}, datum});
    obstacle1.addPoint(concord::Point{concord::ENU{center_x + obstacle_size, center_y - obstacle_size, 0}, datum});
    obstacle1.addPoint(concord::Point{concord::ENU{center_x + obstacle_size, center_y + obstacle_size, 0}, datum});
    obstacle1.addPoint(concord::Point{concord::ENU{center_x - obstacle_size, center_y + obstacle_size, 0}, datum});
    obstacle1.addPoint(
        concord::Point{concord::ENU{center_x - obstacle_size, center_y - obstacle_size, 0}, datum}); // Close polygon

    obstacles.push_back(obstacle1);

    // Create obstacle avoider
    farmtrax::ObstacleAvoider avoider(obstacles, datum);

    std::cout << "Created " << obstacles.size() << " obstacles\n";

    // Visualize obstacles
    farmtrax::visualize::show_obstacles(obstacles, rec);

    auto part_cnt = field.get_parts().size();
    std::cout << "\n=== Field Processing with Optimized Partitioning ===\n";
    std::cout << "Total field parts from intelligent partitioning: " << part_cnt << "\n";

    farmtrax::visualize::show_field(field, rec);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    for (size_t f = 0; f < field.get_parts().size(); f++) {
        std::cout << "\n--- Processing Field Part " << (f + 1) << " of " << part_cnt << " ---\n";
        
        const auto& part = field.get_parts()[f];
        std::cout << "Part " << (f + 1) << " statistics:\n";
        std::cout << "  - Headland rings: " << part.headlands.size() << "\n";
        std::cout << "  - Work swaths: " << part.swaths.size() << "\n";
        
        // Calculate part area for partitioning validation
        auto part_area = boost::geometry::area(part.border.b_polygon);
        std::cout << "  - Calculated area: " << std::fixed << std::setprecision(1) << part_area << " sq.m (" 
                  << (part_area / 10000.0) << " hectares)\n";

        auto fieldPtr = std::make_shared<farmtrax::Part>(field.get_parts()[f]);
        farmtrax::Divy divy(fieldPtr, farmtrax::DivisionType::ALTERNATE, num_machines);
        divy.compute_division();

        // farmtrax::visualize::show_divisions(divy, rec);

        std::this_thread::sleep_for(std::chrono::seconds(10));

        num_machines = 4;
        divy.set_machine_count(num_machines);
        divy.compute_division();

        // farmtrax::visualize::show_divisions(divy, rec);

        auto &res = divy.result();
        for (std::size_t m = 0; m < num_machines; ++m) {
            if (res.swaths_per_machine.at(m).empty()) {
                std::cout << "Machine " << m << " has no swaths assigned\n";
                continue;
            }

            // Apply obstacle avoidance to the swaths
            std::cout << "Machine " << m << " original swaths: " << res.swaths_per_machine.at(m).size() << "\n";

            // Apply obstacle avoidance with 2.0 meter inflation distance
            auto avoided_swaths = avoider.avoid(res.swaths_per_machine.at(m), 2.0f);

            // Create Nety instance from obstacle-avoided swaths (now filters to only SwathType::Swath)
            farmtrax::Nety nety(avoided_swaths);
            nety.field_traversal(); // This reorders the swaths internally

            std::cout << "Machine " << m << " has " << nety.get_swaths().size()
                      << " swaths in Nety after filtering (only regular swaths)\n";

            // Visualize the optimized swath tour using the reordered swaths
            farmtrax::visualize::show_swath_tour(nety, rec, m);
        }
    }

    return 0;
}
