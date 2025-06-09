// Example: Advanced Field Partitioning with Multi-Criteria System
// This example demonstrates how to use the sophisticated partitioning capabilities

#include "farmtrax/field.hpp"
#include "farmtrax/party.hpp"
#include <iostream>

int main() {
    // Example: Complex field boundary (L-shaped field with narrow bridge)
    concord::Polygon complex_field;
    concord::Datum datum{51.989, 5.658, 53.801};
    
    // Add points for an L-shaped field with a narrow connection
    complex_field.addPoint(concord::Point{concord::ENU{0, 0, 0}, datum});
    complex_field.addPoint(concord::Point{concord::ENU{100, 0, 0}, datum});
    complex_field.addPoint(concord::Point{concord::ENU{100, 200, 0}, datum});
    complex_field.addPoint(concord::Point{concord::ENU{150, 200, 0}, datum});
    complex_field.addPoint(concord::Point{concord::ENU{150, 300, 0}, datum});
    complex_field.addPoint(concord::Point{concord::ENU{120, 300, 0}, datum});
    complex_field.addPoint(concord::Point{concord::ENU{120, 220, 0}, datum});
    complex_field.addPoint(concord::Point{concord::ENU{0, 220, 0}, datum});
    complex_field.addPoint(concord::Point{concord::ENU{0, 0, 0}, datum}); // Close polygon
    
    std::cout << "=== Advanced Field Partitioning Demo ===\n\n";
    
    // 1. Basic partitioning with default criteria
    std::cout << "1. Basic Partitioning (default criteria):\n";
    farmtrax::Partitioner basic_partitioner(complex_field, datum);
    auto basic_parts = basic_partitioner.partition(15000.0); // 1.5 hectares max
    std::cout << "   - Split into " << basic_parts.size() << " parts\n";
    
    // 2. Advanced partitioning with custom criteria
    std::cout << "\n2. Advanced Multi-Criteria Partitioning:\n";
    farmtrax::Partitioner advanced_partitioner(complex_field, datum);
    
    // Configure custom partitioning criteria
    farmtrax::Partitioner::PartitionCriteria custom_criteria;
    custom_criteria.max_area = 10000.0;           // 1 hectare max
    custom_criteria.min_convexity = 0.7;          // Require 70% convexity
    custom_criteria.max_aspect_ratio = 3.0;       // Max 3:1 length/width ratio
    custom_criteria.min_bridge_width = 15.0;      // Detect bridges < 15m
    custom_criteria.tooth_threshold = 0.2;        // Split teeth > 20% of area
    custom_criteria.enable_bridge_detection = true;
    custom_criteria.enable_tooth_detection = true;
    custom_criteria.enable_aspect_splitting = true;
    custom_criteria.max_recursion_depth = 6;
    
    auto advanced_parts = advanced_partitioner.partition(10000.0, custom_criteria);
    std::cout << "   - Custom criteria split into " << advanced_parts.size() << " parts\n";
    
    // 3. Demonstrate different splitting strategies
    std::cout << "\n3. Testing Individual Splitting Strategies:\n";
    
    // Bridge detection focused
    farmtrax::Partitioner::PartitionCriteria bridge_criteria;
    bridge_criteria.max_area = 50000.0;          // Large area threshold
    bridge_criteria.min_bridge_width = 10.0;     // Aggressive bridge detection
    bridge_criteria.enable_bridge_detection = true;
    bridge_criteria.enable_tooth_detection = false;
    bridge_criteria.enable_aspect_splitting = false;
    
    farmtrax::Partitioner bridge_partitioner(complex_field, datum);
    auto bridge_parts = bridge_partitioner.partition(50000.0, bridge_criteria);
    std::cout << "   - Bridge-focused: " << bridge_parts.size() << " parts\n";
    
    // Convexity focused
    farmtrax::Partitioner::PartitionCriteria convexity_criteria;
    convexity_criteria.max_area = 50000.0;
    convexity_criteria.min_convexity = 0.8;      // Very strict convexity
    convexity_criteria.enable_bridge_detection = false;
    convexity_criteria.enable_tooth_detection = true;
    convexity_criteria.enable_aspect_splitting = false;
    
    farmtrax::Partitioner convexity_partitioner(complex_field, datum);
    auto convexity_parts = convexity_partitioner.partition(50000.0, convexity_criteria);
    std::cout << "   - Convexity-focused: " << convexity_parts.size() << " parts\n";
    
    // 4. Integration with Field class
    std::cout << "\n4. Integration with Complete Processing Pipeline:\n";
    farmtrax::Field field(complex_field, 0.1, datum, true, 0.7, 8000.0); // 0.8 hectare threshold
    std::cout << "   - Field automatically partitioned into " << field.get_parts().size() << " parts\n";
    
    // Generate agricultural features
    field.gen_field(4.0, 0.0, 2); // 4m swath width, 0° angle, 2 headland rings
    
    for (size_t i = 0; i < field.get_parts().size(); ++i) {
        const auto& part = field.get_parts()[i];
        std::cout << "   - Part " << i << ": " 
                  << part.headlands.size() << " headlands, " 
                  << part.swaths.size() << " swaths\n";
    }
    
    // 5. Access partitioning criteria
    std::cout << "\n5. Current Partitioning Criteria:\n";
    const auto& criteria = advanced_partitioner.getCriteria();
    std::cout << "   - Max area: " << criteria.max_area << " sq.m\n";
    std::cout << "   - Min convexity: " << criteria.min_convexity << "\n";
    std::cout << "   - Max aspect ratio: " << criteria.max_aspect_ratio << "\n";
    std::cout << "   - Min bridge width: " << criteria.min_bridge_width << "m\n";
    std::cout << "   - Bridge detection: " << (criteria.enable_bridge_detection ? "enabled" : "disabled") << "\n";
    std::cout << "   - Tooth detection: " << (criteria.enable_tooth_detection ? "enabled" : "disabled") << "\n";
    
    std::cout << "\n=== Partitioning Complete ===\n";
    
    return 0;
}
